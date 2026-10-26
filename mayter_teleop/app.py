"""Direct arm teleop, two tool motors, and bounded jaw calibration steps."""
import json,queue,threading,time,urllib.request,atexit,shutil,datetime
from pathlib import Path
from concurrent.futures import Future
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
ROOT=Path(__file__).resolve().parent.parent
FOLLOWER='/dev/cu.usbmodem5C821107831';LEADER='/dev/cu.usbmodem5C821101051'
CONTROLLER_VERSION='two-tools-jaw-jog-20261025'
TOOL_IDS={'tire':7,'socket':8}

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
 def __init__(self):self.follower=None;self.leader=None;self.port=None;self.packet=None;self.raw=False;self.tool_ids_checked=set();self.jaw_settings=None;self.jaw_deadline=0.;self.jaw_origin=None
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
    raise RuntimeError(f'{name.title()} motor at ID {mid} did not identify as a healthy STS3215 (model={m}, communication={c}, status={e}). Each tool must have its unique configured ID at 1000000 baud. Use Check motor connections. Assign a replacement ID only with that motor connected by itself.')
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
  report['all_expected_responded']=all(not r['error'] and len(r['motors'])==(6+len(TOOL_IDS) if role=='follower' else 6) and all(m['healthy'] for m in r['motors'].values()) for role,r in report['ports'].items())
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
 def prepare_jaw(self):
  model,comm,error=self.packet.ping(self.port,6)
  if (model,comm,error)!=(777,0,0):raise RuntimeError('Jaw motor ID 6 did not respond correctly')
  self.write(6,40,0,1)
  if self.read(6,40,1)!=0:raise RuntimeError('Jaw torque off could not be verified')
  self.jaw_settings={address:(size,self.read(6,address,size)) for address,size in [(9,2),(11,2),(33,1),(41,1),(46,2),(48,2),(55,1)]}
  self.jaw_origin=self.read(6,56)
  stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
  backup=ROOT/'calibration'/('jaw_jog_'+stamp+'.backup.json')
  backup.write_text(json.dumps(self.jaw_settings,indent=2))
 def release_jaw(self):
  self.write(6,40,0,1)
  if self.read(6,40,1)!=0:raise RuntimeError('Jaw torque off could not be verified')
  self.jaw_deadline=0.
 def jog_jaw(self,direction):
  if direction not in (-1,1):raise ValueError('Invalid jaw step')
  if self.jaw_settings is None:raise ValueError('Begin jaw calibration first')
  if self.jaw_deadline:raise ValueError('Wait for the current jaw step to finish')
  current=self.read(6,56);target=current+direction*10
  if not 0<=target<=4095 or abs(target-self.jaw_origin)>2048:raise ValueError('Jaw step would cross the calibration travel limit')
  self.release_jaw()
  # Temporary calibration limits allow finding endpoints beyond the old range.
  # Original registers are backed up and restored when calibration ends.
  self.write(6,55,0,1)
  for address,value,size in [(9,0,2),(11,4095,2),(33,0,1),(41,10,1),(48,150,2),(46,100,2),(42,current,2)]:self.write(6,address,value,size)
  self.write(6,55,1,1)
  self.write(6,40,1,1);self.jaw_deadline=time.monotonic()+.6
  self.write(6,42,target)
  return target
 def update_jaw(self,last_heartbeat):
  if self.jaw_deadline and (time.monotonic()>=self.jaw_deadline or time.monotonic()-last_heartbeat>=.35):self.release_jaw()
  return self.read(6,56)
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
  self.configure_tools()
  self.follower.bus.enable_torque()
 def configure_tools(self):
  for mid in TOOL_IDS.values():
   self.write(mid,40,0,1);self.write(mid,55,0,1);self.write(mid,33,1,1)
   self.write(mid,41,100,1);self.write(mid,48,1000)
   self.write(mid,46,0)
   self.write(mid,55,1,1);self.write(mid,40,1,1)
 def connect_tools(self):
  self.connect_cal();self.configure_tools()
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
    if not confirmed:errors.append(f'Torque off not verified on {FOLLOWER} tool motor {mid}')
  if self.jaw_settings is not None and self.port and self.port.is_open:
   try:
    self.release_jaw();self.write(6,55,0,1)
    for address,(size,value) in self.jaw_settings.items():
     if address!=55:
      self.write(6,address,value,size)
      if self.read(6,address,size)!=value:raise RuntimeError(f'Jaw restore failed at register {address}')
    self.write(6,55,self.jaw_settings[55][1],1)
    self.jaw_settings=None
   except Exception as e:errors.append(str(e))
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
  self.q=queue.Queue();self.lock=threading.Lock();self.held={name:0 for name in TOOL_IDS};self.last_heartbeat=0.;self.hw=None
  self.status={'controller_version':CONTROLLER_VERSION,'mode':'idle','error':None,'stop_error':None,'hz':0,'socket_speed':0,'tire_speed':0,'jaw_target':None,'connection_check':None,'jaw_position':None,'jaw_closed':None,'jaw_open':None}
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
  with self.lock:self.held={name:0 for name in TOOL_IDS}
  errors=[]
  if self.hw and self.status['mode']=='jaw_calibrating':
   try:self.hw.write(6,40,0,1)
   except Exception as e:errors.append(str(e))
  had_hardware=self.hw is not None
  if self.hw:errors.extend(self.hw.close())
  self.hw=None
  self.status.update(mode='idle',hz=0,socket_speed=0,tire_speed=0,jaw_target=None)
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
   self.hw=Hardware();self.hw.connect_cal();self.hw.prepare_jaw()
   self.last_heartbeat=time.monotonic()
   self.status.update(mode='jaw_calibrating',error=None,jaw_closed=None,jaw_open=None)
  elif a in ('jaw_minus','jaw_plus','jaw_release'):
   if self.status['mode']!='jaw_calibrating':raise ValueError('Begin jaw calibration first')
   if a=='jaw_release':self.hw.release_jaw();self.status['jaw_target']=None
   else:
    if time.monotonic()-self.last_heartbeat>=.35:raise ValueError('Control page heartbeat expired')
    self.status['jaw_target']=self.hw.jog_jaw(-1 if a=='jaw_minus' else 1)
  elif a in ('jaw_closed','jaw_open'):
   if self.status['mode']!='jaw_calibrating':raise ValueError('Begin jaw calibration first')
   self.hw.release_jaw();self.status['jaw_target']=None
   self.status[a]=self.hw.read(6,56)
  elif a=='jaw_save':
   if self.status['mode']!='jaw_calibrating':raise ValueError('Begin jaw calibration first')
   if self.status['jaw_closed'] is None or self.status['jaw_open'] is None:raise ValueError('Mark closed and open positions first')
   self.hw.release_jaw()
   path=ROOT/'calibration/follower/hiwonder_follower_fresh.json'
   data=json.loads(path.read_text());old=data['gripper'];new=jaw_calibration(self.status['jaw_closed'],self.status['jaw_open'],old)
   offset=self.hw.read(6,31);offset=-(offset&2047) if offset&2048 else offset
   if offset!=old['homing_offset']:raise ValueError('Jaw homing offset differs from file; stop and review before saving')
   stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
   shutil.copy2(path,path.with_name(path.stem+'_'+stamp+'.backup.json'))
   self.hw.write(6,55,0,1);self.hw.write(6,9,new['range_min']);self.hw.write(6,11,new['range_max']);self.hw.write(6,55,1,1)
   if self.hw.read(6,9)!=new['range_min'] or self.hw.read(6,11)!=new['range_max']:raise RuntimeError('Jaw calibration write did not verify')
   if self.hw.jaw_settings is not None:
    self.hw.jaw_settings[9]=(2,new['range_min']);self.hw.jaw_settings[11]=(2,new['range_max'])
   data['gripper']=new;tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2)+'\n');tmp.replace(path)
   cache=Path.home()/'.cache/huggingface/lerobot/calibration/robots/so_follower/hiwonder_follower_fresh.json'
   if cache.exists():
    shutil.copy2(cache,cache.with_name(cache.stem+'_'+stamp+'.backup.json'))
    cached=json.loads(cache.read_text());cached['gripper']=new;cache.write_text(json.dumps(cached,indent=2)+'\n')
   self.status['jaw_saved']=new;self.stop()
  elif a in ('start','tools'):
   if self.status['mode']!='idle':raise ValueError('Stop the current session first')
   with self.lock:self.held={name:0 for name in TOOL_IDS}
   self.hw=Hardware()
   if a=='tools':self.hw.connect_tools()
   else:self.hw.connect_teleop()
   self.last_heartbeat=time.monotonic();self.last_speeds={name:0 for name in TOOL_IDS}
   self.status.update(mode='tools' if a=='tools' else 'teleop',error=None,stop_error=None)
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
     self.status['jaw_position']=self.hw.update_jaw(self.last_heartbeat)
     if not self.hw.jaw_deadline:self.status['jaw_target']=None
    elif self.status['mode'] in ('teleop','tools'):
     with self.lock:held=dict(self.held);last=self.last_heartbeat
     if tick-last>1.0:raise RuntimeError('Control page disconnected; teleoperation stopped')
     if self.status['mode']=='teleop':self.hw.track()
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
