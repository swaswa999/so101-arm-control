"""Direct arm teleop and one click-to-run socket motor. Tire motor disconnected."""
import json,queue,threading,time,urllib.request,atexit,shutil,datetime
from pathlib import Path
from concurrent.futures import Future
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
ROOT=Path(__file__).resolve().parent.parent
FOLLOWER='/dev/cu.usbmodem5C821107831';LEADER='/dev/cu.usbmodem5B3E0915611'
CONTROLLER_VERSION='socket-only-20261020'
TOOL_IDS={'socket':8}

def held_speeds(held,now,last):
 if now-last>=.35:return {name:0 for name in TOOL_IDS}
 return {name:held.get(name,0)*held.get(name+'_speed',600) for name in TOOL_IDS}

def encode_velocity(value):return abs(value)|(32768 if value<0 else 0)

def jaw_calibration(closed,opened,old):
 closed,opened=int(closed),int(opened)
 if not 0<=min(closed,opened)<max(closed,opened)<=4095:raise ValueError('Invalid jaw encoder readings')
 if not 40<=abs(opened-closed)<=2048:raise ValueError('Jaw endpoints are too close or cross encoder zero')
 return {**old,'id':6,'range_min':min(closed,opened)+8,'range_max':max(closed,opened)-8,'drive_mode':int(closed>opened)}

class Hardware:
 def __init__(self):self.follower=None;self.leader=None;self.port=None;self.packet=None;self.raw=False;self.tool_ids_checked=set()
 def idle_lelab(self):
  for endpoint,key in [('teleoperation-status','teleoperation_active'),('recording-status','recording_active'),('inference-status','inference_active')]:
   try:
    with urllib.request.urlopen('http://127.0.0.1:8000/'+endpoint,timeout=.5) as r:d=json.load(r)
   except OSError:continue
   if d.get(key):raise RuntimeError('Stop the active LeLab session first.')
 def connect_cal(self):
  import scservo_sdk as sdk
  self.idle_lelab();self.port=sdk.PortHandler(FOLLOWER);self.packet=sdk.PacketHandler(0);self.raw=True
  if not self.port.openPort():raise RuntimeError('Cannot open follower USB connection')
  self.port.setBaudRate(1000000);self.check_tools()
  for mid in TOOL_IDS.values():self.write(mid,40,0,1)
 def check_tools(self):
  for name,mid in TOOL_IDS.items():
   m,c,e=self.packet.ping(self.port,mid)
   if c or e or m!=777:
    raise RuntimeError(f'{name.title()} motor at ID {mid} did not identify as a healthy STS3215 (model={m}, communication={c}, status={e}). The previous socket used ID 8; the replacement must have a unique ID 8 at 1000000 baud. Use Check motor connections. Assign a replacement ID only with that motor connected by itself.')
   self.tool_ids_checked.add(mid)
 def check_connections(self):
  """Read-only check: never change IDs, modes, calibration, goals, or torque."""
  import scservo_sdk as sdk
  self.idle_lelab()
  report={'baudrate':1000000,'read_only':True,'expected_socket_id':TOOL_IDS['socket'],'ports':{}}
  for role,path,ids in [('follower',FOLLOWER,[1,2,3,4,5,6,*TOOL_IDS.values()]),('leader',LEADER,list(range(1,7)))]:
   port=sdk.PortHandler(path);packet=sdk.PacketHandler(0)
   result={'port':path,'motors':{},'error':None};report['ports'][role]=result
   try:
    if not port.openPort():raise RuntimeError('Cannot open USB connection')
    if not port.setBaudRate(1000000):raise RuntimeError('Cannot set USB baudrate')
    for mid in ids:
     attempts=[]
     for attempt in range(3):
      model,comm,error=packet.ping(port,mid)
      attempts.append({'model':model,'communication':comm,'status':error})
     healthy=all(p['communication']==0 and p['status']==0 and p['model']==777 for p in attempts)
     result['motors'][str(mid)]={'healthy':healthy,'attempts':attempts}
   except Exception as e:result['error']=str(e)
   finally:
    if port.is_open:port.closePort()
  report['all_expected_responded']=all(not r['error'] and len(r['motors'])==(7 if role=='follower' else 6) and all(m['healthy'] for m in r['motors'].values()) for role,r in report['ports'].items())
  return report
 def read(self,mid,address,size=2):
  fn=self.packet.read1ByteTxRx if size==1 else self.packet.read2ByteTxRx
  for attempt in range(3):
   v,c,e=fn(self.port,mid,address)
   if not c and not e:return v
  raise RuntimeError(f'Motor {mid} read failed at register {address}: communication {c}, status {e}')
 def write(self,mid,address,value,size=2):
  fn=self.packet.write1ByteTxRx if size==1 else self.packet.write2ByteTxRx
  c,e=fn(self.port,mid,address,int(value))
  if c or e:raise RuntimeError(f'Motor {mid} write failed at register {address}: communication {c}, status {e}')
 def connect_teleop(self):
  from lerobot.robots.so_follower import SO101Follower,SO101FollowerConfig
  from lerobot.teleoperators.so_leader import SO101Leader,SO101LeaderConfig
  self.idle_lelab()
  self.follower=SO101Follower(SO101FollowerConfig(port=FOLLOWER,id='hiwonder_follower_fresh',calibration_dir=ROOT/'calibration/follower',use_degrees=True,max_relative_target=None,cameras={}))
  self.leader=SO101Leader(SO101LeaderConfig(port=LEADER,id='hiwonder_leader_fresh',calibration_dir=ROOT/'calibration/leader',use_degrees=True))
  self.follower.bus.connect();self.port=self.follower.bus.port_handler;self.packet=self.follower.bus.packet_handler
  self.check_tools()
  self.leader.bus.connect()
  self.follower.bus.disable_torque();self.leader.bus.disable_torque()
  self.follower.bus.write_calibration(self.follower.calibration)
  self.leader.bus.write_calibration(self.leader.calibration);self.leader.configure()
  # Match the installed follower configuration without its torque_disabled context,
  # which enables torque on exit before the hold targets below have been seeded.
  self.follower.bus.configure_motors()
  for name in self.follower.bus.motors:
   for register,value in [('Operating_Mode',0),('P_Coefficient',16),('I_Coefficient',0),('D_Coefficient',32)]:
    self.follower.bus.write(register,name,value)
   if name=='gripper':
    for register,value in [('Max_Torque_Limit',500),('Protection_Current',250),('Overload_Torque',25)]:
     self.follower.bus.write(register,name,value)
  # Seed hold targets before enabling torque. Leader tracking remains direct.
  initial=self.follower.bus.sync_read('Present_Position',num_retry=2)
  self.follower.bus.sync_write('Goal_Position',initial)
  for name in self.follower.bus.motors:self.follower.bus.write('Goal_Velocity',name,0)
  for mid in TOOL_IDS.values():
   self.write(mid,40,0,1);self.write(mid,55,0,1);self.write(mid,33,1,1)
   self.write(mid,41,100,1);self.write(mid,48,1000)
   self.write(mid,46,0)
   self.write(mid,55,1,1);self.write(mid,40,1,1)
  self.follower.bus.enable_torque()
 def track(self):
  p=self.leader.bus.sync_read('Present_Position',num_retry=2)
  self.follower.send_action({name+'.pos':v for name,v in p.items()})
 def close(self):
  errors=[]
  if self.port and self.port.is_open:
   for mid in self.tool_ids_checked:
    try:
     if self.read(mid,33,1)==1:self.write(mid,46,0)
    except Exception as e:errors.append(str(e))
    confirmed=False
    for attempt in range(3):
     try:
      self.packet.write1ByteTxRx(self.port,mid,40,0)
      value,comm,error=self.packet.read1ByteTxRx(self.port,mid,40)
      if comm==0 and value==0:confirmed=True;break
     except Exception:pass
    if not confirmed:errors.append(f'Torque off not verified on {FOLLOWER} socket motor {mid}')
  for device in (self.follower,self.leader):
   if device and device.bus.is_connected:
    bus=device.bus
    # Attempt every motor independently even if an earlier motor stops replying.
    for motor in bus.motors.values():
     confirmed=False
     for attempt in range(3):
      try:
       c,e=bus.packet_handler.write1ByteTxRx(bus.port_handler,motor.id,40,0)
       value,rc,re=bus.packet_handler.read1ByteTxRx(bus.port_handler,motor.id,40)
       if rc==0 and value==0:confirmed=True;break
      except Exception:pass
     if not confirmed:errors.append(f'Torque off not verified on {bus.port} motor {motor.id}')
    try:bus.disconnect(False)
    except Exception as e:errors.append(str(e));bus.port_handler.closePort()
  if self.raw and self.port and self.port.is_open:self.port.closePort()
  return errors

class Controller:
 def __init__(self):
  self.q=queue.Queue();self.lock=threading.Lock();self.held={'socket':0};self.last_heartbeat=0.;self.hw=None
  self.status={'controller_version':CONTROLLER_VERSION,'mode':'idle','error':None,'stop_error':None,'hz':0,'socket_speed':0,'connection_check':None,'jaw_position':None,'jaw_closed':None,'jaw_open':None}
  threading.Thread(target=self.run,daemon=True,name='motor-owner').start()
 def snapshot(self):
  with self.lock:return dict(self.status)
 def request(self,action):
  f=Future();self.q.put((action,f));return f.result(timeout=15)
 def controls(self,d):
  held={}
  for name in TOOL_IDS:
   direction=int(d.get(name,0));speed=int(d.get(name+'_speed',600))
   if direction not in (-1,0,1):raise ValueError('Invalid tool direction')
   if not 50<=speed<=2000:raise ValueError('Tool speed must be 50–2000')
   held[name]=direction;held[name+'_speed']=speed
  with self.lock:self.held=held;self.last_heartbeat=time.monotonic()
 def stop(self):
  with self.lock:self.held={'socket':0}
  errors=[]
  if self.hw and self.status['mode']=='jaw_calibrating':
   try:self.hw.write(6,40,0,1)
   except Exception as e:errors.append(str(e))
  had_hardware=self.hw is not None
  if self.hw:errors.extend(self.hw.close())
  self.hw=None
  self.status.update(mode='idle',hz=0,socket_speed=0)
  if errors:self.status['stop_error']='Stop verification incomplete: '+'; '.join(errors)
  elif had_hardware:self.status['stop_error']=None
 def command(self,a):
  if a=='stop':self.stop()
  elif a=='check_connections':
   if self.status['mode']!='idle':raise ValueError('Stop the current session first')
   self.status['connection_check']=None
   self.status['connection_check']=Hardware().check_connections()
  elif a=='jaw_calibrate':
   if self.status['mode']!='idle':raise ValueError('Stop teleoperation first')
   self.hw=Hardware();self.hw.connect_cal();self.hw.write(6,40,0,1)
   self.status.update(mode='jaw_calibrating',error=None,jaw_closed=None,jaw_open=None)
  elif a in ('jaw_closed','jaw_open'):
   if self.status['mode']!='jaw_calibrating':raise ValueError('Begin jaw calibration first')
   self.status[a]=self.hw.read(6,56)
  elif a=='jaw_save':
   if self.status['mode']!='jaw_calibrating':raise ValueError('Begin jaw calibration first')
   if self.status['jaw_closed'] is None or self.status['jaw_open'] is None:raise ValueError('Mark closed and open positions first')
   path=ROOT/'calibration/follower/hiwonder_follower_fresh.json'
   data=json.loads(path.read_text());old=data['gripper'];new=jaw_calibration(self.status['jaw_closed'],self.status['jaw_open'],old)
   offset=self.hw.read(6,31);offset=-(offset&2047) if offset&2048 else offset
   if offset!=old['homing_offset']:raise ValueError('Jaw homing offset differs from file; stop and review before saving')
   stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
   shutil.copy2(path,path.with_name(path.stem+'_'+stamp+'.backup.json'))
   self.hw.write(6,55,0,1);self.hw.write(6,9,new['range_min']);self.hw.write(6,11,new['range_max']);self.hw.write(6,55,1,1)
   if self.hw.read(6,9)!=new['range_min'] or self.hw.read(6,11)!=new['range_max']:raise RuntimeError('Jaw calibration write did not verify')
   data['gripper']=new;tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2)+'\n');tmp.replace(path)
   cache=Path.home()/'.cache/huggingface/lerobot/calibration/robots/so_follower/hiwonder_follower_fresh.json'
   if cache.exists():
    shutil.copy2(cache,cache.with_name(cache.stem+'_'+stamp+'.backup.json'))
    cached=json.loads(cache.read_text());cached['gripper']=new;cache.write_text(json.dumps(cached,indent=2)+'\n')
   self.status['jaw_saved']=new;self.stop()
  elif a=='start':
   if self.status['mode']!='idle':raise ValueError('Stop the current session first')
   with self.lock:self.held={'socket':0}
   self.hw=Hardware();self.hw.connect_teleop()
   self.last_heartbeat=time.monotonic();self.last_speeds={'socket':0}
   self.status.update(mode='teleop',error=None,stop_error=None)
  else:raise ValueError('Unknown action')
  return self.snapshot()
 def run(self):
  while True:
   tick=time.monotonic()
   try:
    try:a,f=self.q.get_nowait()
    except queue.Empty:a=f=None
    if a:
     try:f.set_result(self.command(a))
     except ValueError as e:self.status['error']=str(e);f.set_exception(e)
     except Exception as e:self.status['error']=str(e);self.stop();f.set_exception(e)
    if self.status['mode']=='jaw_calibrating':
     self.status['jaw_position']=self.hw.read(6,56)
    elif self.status['mode']=='teleop':
     with self.lock:held=dict(self.held);last=self.last_heartbeat
     if tick-last>1.0:raise RuntimeError('Control page disconnected; teleoperation stopped')
     self.hw.track()
     speeds=held_speeds(held,tick,last)
     for name,mid in TOOL_IDS.items():
      if speeds[name]!=self.last_speeds[name]:
       self.hw.write(mid,46,encode_velocity(speeds[name]));self.last_speeds[name]=speeds[name]
      self.status[name+'_speed']=speeds[name]
     self.status['hz']=round(1/max(.0001,time.monotonic()-tick),1)
   except Exception as e:self.status['error']=str(e);self.stop()
   time.sleep(max(0,1/30-(time.monotonic()-tick)))

controller=None
class Handler(BaseHTTPRequestHandler):
 def log_message(self,*args):pass
 def send(self,data,status=200,kind='application/json'):
  body=data.encode() if isinstance(data,str) else json.dumps(data).encode()
  self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
 def do_GET(self):
  if self.path=='/':self.send((Path(__file__).parent/'index.html').read_text(),kind='text/html; charset=utf-8')
  elif self.path=='/status':self.send(controller.snapshot())
  else:self.send({'error':'Not found'},404)
 def do_POST(self):
  origin=self.headers.get('Origin')
  if origin and origin not in ('http://127.0.0.1:8010','http://localhost:8010'):return self.send({'error':'Origin rejected'},403)
  try:
   n=int(self.headers.get('Content-Length','0'))
   if n>2048:raise ValueError('Request too large')
   d=json.loads(self.rfile.read(n))
   if self.path=='/controls':controller.controls(d);result={'ok':True}
   elif self.path=='/action':result=controller.request(d['action'])
   else:raise ValueError('Unknown endpoint')
   self.send(result)
  except Exception as e:self.send({'error':str(e)},400)

if __name__=='__main__':
 controller=Controller();atexit.register(controller.stop)
 server=ThreadingHTTPServer(('127.0.0.1',8010),Handler)
 print('Mayter Teleop: http://127.0.0.1:8010 — motors idle until Start is pressed',flush=True)
 try:server.serve_forever()
 finally:controller.request('stop');server.server_close()
