import threading,unittest
from types import SimpleNamespace
from unittest.mock import patch
from app import Controller,Hardware,held_speeds,encode_velocity,jaw_calibration
class Controls(unittest.TestCase):
 def test_jaw_reversal_preserves_other_settings(self):
  old={'id':6,'homing_offset':247,'drive_mode':0,'range_min':1315,'range_max':2788}
  result=jaw_calibration(2000,1500,old)
  self.assertEqual(result,{'id':6,'homing_offset':247,'drive_mode':1,'range_min':1508,'range_max':1992})
  self.assertEqual(old['range_min'],1315)
 def test_jaw_invalid_endpoints(self):
  for closed,opened in [(1500,1505),(1,4090),(-1,500)]:
   with self.assertRaises(ValueError):jaw_calibration(closed,opened,{})
 def test_release_stops_socket(self):
  self.assertEqual(held_speeds({'tire':0,'socket':0},10,10),{'socket':0})
 def test_stale_hold_stops_socket(self):
  self.assertEqual(held_speeds({'tire':1,'socket':-1},10,9),{'socket':0})
 def test_socket_direction_and_speed_ignores_disconnected_tire(self):
  h={'tire':-1,'tire_speed':1000,'socket':1,'socket_speed':500}
  self.assertEqual(held_speeds(h,10,10),{'socket':500})
  self.assertEqual(encode_velocity(-1000),32768+1000)
  self.assertEqual(encode_velocity(0),0)
 def test_cleanup_attempts_remaining_motors_after_fault(self):
  from types import SimpleNamespace
  writes=[]
  class Packet:
   def write1ByteTxRx(self,p,i,a,v):
    writes.append(i)
    if i==1:raise OSError('Disconnected motor')
    return 0,0
   def read1ByteTxRx(self,p,i,a):return 0,0,0
  bus=SimpleNamespace(is_connected=True,port='test',port_handler=SimpleNamespace(closePort=lambda:None),packet_handler=Packet(),motors={'a':SimpleNamespace(id=1),'b':SimpleNamespace(id=2)},disconnect=lambda x:None)
  h=Hardware();h.follower=SimpleNamespace(bus=bus)
  errors=h.close();self.assertIn(2,writes);self.assertTrue(any('motor 1' in e for e in errors))
 def test_old_page_cannot_command_disconnected_tire(self):
  with patch.object(threading.Thread,'start'):controller=Controller()
  controller.controls({'tire':1,'tire_speed':2000,'socket':0})
  self.assertEqual(controller.held,{'socket':0,'socket_speed':600})
 def test_only_socket_required_and_missing_socket_blocks_start(self):
  calls=[]
  h=Hardware();h.port=object()
  h.packet=SimpleNamespace(ping=lambda p,i:(calls.append(i) or (777,0,0)))
  h.check_tools();self.assertEqual(calls,[8])
  h.packet.ping=lambda p,i:(0,-3001,0)
  with self.assertRaisesRegex(RuntimeError,'replacement must have a unique ID 8'):h.check_tools()
 def test_socket_cleanup_verifies_off_and_never_addresses_tire(self):
  calls=[]
  h=Hardware();h.port=SimpleNamespace(is_open=True);h.tool_ids_checked={8}
  h.packet=SimpleNamespace(read1ByteTxRx=lambda p,i,a:(calls.append(('read',i,a)) or (1,0,0)),write1ByteTxRx=lambda p,i,a,v:(calls.append(('write',i,a)) or (0,0)),write2ByteTxRx=lambda p,i,a,v:(calls.append(('write',i,a)) or (0,0)))
  errors=h.close()
  self.assertTrue(any('socket motor 8' in e for e in errors))
  self.assertTrue(all(c[1]==8 for c in calls))
 def test_stop_keeps_original_error_and_retains_unconfirmed_state(self):
  with patch.object(threading.Thread,'start'):controller=Controller()
  controller.status['error']='Original failure: motor IDs missing'
  controller.hw=SimpleNamespace(close=lambda:['Torque off not verified'])
  controller.stop()
  self.assertEqual(controller.status['error'],'Original failure: motor IDs missing')
  self.assertIn('Torque off not verified',controller.status['stop_error'])
  controller.stop()
  self.assertIn('Torque off not verified',controller.status['stop_error'])
 def test_connection_check_has_no_motor_writes_and_closes_ports(self):
  opened=[];pinged=[]
  class Port:
   def __init__(self,path):self.path=path;self.is_open=False;opened.append(self)
   def openPort(self):self.is_open=True;return True
   def setBaudRate(self,b):return b==1000000
   def closePort(self):self.is_open=False
  class Packet:
   def ping(self,p,i):pinged.append(i);return 777,0,0
   def __getattr__(self,name):raise AssertionError('Unexpected motor operation '+name)
  with patch('scservo_sdk.PortHandler',Port),patch('scservo_sdk.PacketHandler',lambda v:Packet()),patch.object(Hardware,'idle_lelab'):
   report=Hardware().check_connections()
  self.assertTrue(report['all_expected_responded'])
  self.assertNotIn(7,pinged)
  self.assertTrue(all(not p.is_open for p in opened))
 def test_failed_connection_check_does_not_claim_healthy(self):
  class Port:
   is_open=False
   def __init__(self,path):pass
   def openPort(self):raise PermissionError('USB blocked')
  with patch('scservo_sdk.PortHandler',Port),patch.object(Hardware,'idle_lelab'):
   report=Hardware().check_connections()
  self.assertFalse(report['all_expected_responded'])
  self.assertEqual(report['ports']['follower']['error'],'USB blocked')
 def test_initial_hold_and_zero_socket_speed_precede_torque_enable(self):
  events=[]
  class Bus:
   def __init__(self,role):
    self.role=role;self.motors={'shoulder_pan':None,'gripper':None}
    self.port_handler=object();self.packet_handler=SimpleNamespace(ping=lambda p,i:(777,0,0))
   def connect(self):events.append((self.role,'connect'))
   def disable_torque(self):events.append((self.role,'torque_off'))
   def write_calibration(self,c):events.append((self.role,'calibration'))
   def configure_motors(self):events.append((self.role,'configure'))
   def sync_read(self,*a,**kw):return {'shoulder_pan':0,'gripper':50}
   def sync_write(self,name,data):events.append((self.role,name))
   def write(self,name,motor,value):events.append((self.role,name))
   def enable_torque(self):events.append((self.role,'torque_on'))
  class Follower:
   def __init__(self,c):self.bus=Bus('follower');self.calibration={}
  class Leader:
   def __init__(self,c):self.bus=Bus('leader');self.calibration={}
   def configure(self):events.append(('leader','configure'))
  h=Hardware();h.write=lambda mid,a,v,size=2:events.append(('tool',mid,a,v))
  with patch('lerobot.robots.so_follower.SO101Follower',Follower),patch('lerobot.teleoperators.so_leader.SO101Leader',Leader),patch.object(Hardware,'idle_lelab'):
   h.connect_teleop()
  self.assertLess(events.index(('follower','torque_off')),events.index(('follower','calibration')))
  self.assertLess(events.index(('leader','torque_off')),events.index(('leader','calibration')))
  self.assertLess(events.index(('follower','Goal_Position')),events.index(('follower','torque_on')))
  self.assertLess(events.index(('tool',8,46,0)),events.index(('tool',8,40,1)))
  self.assertTrue(all(e[1]==8 for e in events if e[0]=='tool'))

if __name__=='__main__':unittest.main()
