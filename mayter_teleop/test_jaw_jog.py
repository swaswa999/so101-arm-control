import threading,unittest,tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from app import Hardware,Controller

class JawJog(unittest.TestCase):
 def hardware(self):
  h=Hardware();self.events=[]
  self.regs={9:374,11:1766,33:0,41:0,46:0,48:500,55:1,40:0,56:331,42:331}
  h.read=lambda mid,addr,size=2:self.regs[addr]
  def write(mid,addr,value,size=2):
   self.events.append((mid,addr,value));self.regs[addr]=value
  h.write=write
  h.packet=SimpleNamespace(ping=lambda p,i:(777,0,0))
  h.port=SimpleNamespace(is_open=True,closePort=lambda:None);h.raw=True
  return h
 def prepare(self,h):
  with tempfile.TemporaryDirectory() as tmp,patch('app.ROOT',Path(tmp)):
   (Path(tmp)/'calibration').mkdir();h.prepare_jaw()
   self.assertEqual(len(list((Path(tmp)/'calibration').glob('*.backup.json'))),1)
 def test_step_seeds_current_before_torque_and_does_not_snap_to_old_limit(self):
  h=self.hardware();self.prepare(h);self.events=[]
  self.assertEqual(h.jog_jaw(-1),321)
  self.assertLess(self.events.index((6,42,331)),self.events.index((6,40,1)))
  self.assertLess(self.events.index((6,48,150)),self.events.index((6,40,1)))
  self.assertEqual(self.events[-1],(6,42,321))
  self.assertTrue(all(e[0]==6 for e in self.events))
 def test_expired_heartbeat_releases_jaw(self):
  h=self.hardware();self.prepare(h);h.jog_jaw(1)
  with patch('app.time.monotonic',return_value=100):h.update_jaw(99)
  self.assertEqual(self.regs[40],0);self.assertEqual(h.jaw_deadline,0)
 def test_deadline_releases_even_with_live_heartbeat(self):
  h=self.hardware();self.prepare(h);h.jog_jaw(1)
  with patch('app.time.monotonic',return_value=h.jaw_deadline+.01) as now:h.update_jaw(now.return_value)
  self.assertEqual(self.regs[40],0)
 def test_boundary_rejection_makes_no_writes(self):
  h=self.hardware();self.prepare(h);self.regs[56]=4090;self.events=[]
  with self.assertRaises(ValueError):h.jog_jaw(1)
  self.assertEqual(self.events,[])
 def test_no_overlapping_steps(self):
  h=self.hardware();self.prepare(h);h.jog_jaw(1);self.events=[]
  with self.assertRaises(ValueError):h.jog_jaw(1)
  self.assertEqual(self.events,[])
 def test_stop_restores_original_settings_after_step(self):
  h=self.hardware();self.prepare(h);before=dict(h.jaw_settings);h.jog_jaw(1)
  self.assertEqual(h.close(),[])
  for addr,(_,value) in before.items():self.assertEqual(self.regs[addr],value)
  self.assertEqual(self.regs[40],0)
 def test_saved_limits_survive_cleanup(self):
  h=self.hardware();self.prepare(h);h.jog_jaw(1)
  h.jaw_settings[9]=(2,300);h.jaw_settings[11]=(2,1800)
  self.assertEqual(h.close(),[]);self.assertEqual((self.regs[9],self.regs[11]),(300,1800))
 def test_jog_only_in_calibration_and_fresh_heartbeat(self):
  with patch.object(threading.Thread,'start'):c=Controller()
  with self.assertRaises(ValueError):c.command('jaw_plus')
  c.status['mode']='jaw_calibrating';c.hw=SimpleNamespace(jog_jaw=lambda d:self.fail('stale jog executed'))
  with self.assertRaises(ValueError):c.command('jaw_plus')
 def test_tools_only_does_not_connect_arms(self):
  h=self.hardware();calls=[];h.connect_cal=lambda:calls.append('connect');h.configure_tools=lambda:calls.append('tools')
  h.connect_tools();self.assertEqual(calls,['connect','tools']);self.assertIsNone(h.follower);self.assertIsNone(h.leader)
 def test_tools_watchdog_zeroes_both_velocities_without_tracking(self):
  with patch.object(threading.Thread,'start'):c=Controller()
  writes=[];c.hw=SimpleNamespace(write=lambda *args:writes.append(args),track=lambda:self.fail('Arm tracking used in tools-only mode'))
  c.status['mode']='tools';c.last_heartbeat=9.5;c.held={'tire':1,'socket':-1};c.last_speeds={'tire':300,'socket':-300}
  with patch('app.time.monotonic',return_value=10),patch('app.time.sleep',side_effect=InterruptedError):
   with self.assertRaises(InterruptedError):c.run()
  self.assertEqual(writes,[(7,46,0),(8,46,0)])
 def test_lost_connection_closes_tools_only_session(self):
  with patch.object(threading.Thread,'start'):c=Controller()
  closed=[];c.hw=SimpleNamespace(close=lambda:(closed.append(True) or []))
  c.status['mode']='tools';c.last_heartbeat=8
  with patch('app.time.monotonic',return_value=10),patch('app.time.sleep',side_effect=InterruptedError):
   with self.assertRaises(InterruptedError):c.run()
  self.assertEqual(closed,[True]);self.assertEqual(c.status['mode'],'idle')
if __name__=='__main__':unittest.main()
