"""Local leader/follower teleop with two separately held tool controls.
One worker exclusively owns serial I/O. Importing or launching opens no motor ports.
"""
import json,math,queue,threading,time,urllib.request,atexit
from pathlib import Path
from concurrent.futures import Future
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
ROOT=Path(__file__).resolve().parent.parent
CAL=ROOT/'calibration/tiretool_limits.json'
FOLLOWER='/dev/cu.usbmodem5C821107831';LEADER='/dev/cu.usbmodem5C821073271'

def safe_limits(a,b,margin=20):
 a,b=int(a),int(b)
 if not (0<=a<=4095 and 0<=b<=4095):raise ValueError('Position outside a single turn. Calibration needs review.')
 lo,hi=sorted([a,b])
 if hi-lo<60:raise ValueError('Endpoints are too close; capture both ends of the usable travel.')
 if hi-lo>2048:raise ValueError('Range crosses zero or exceeds half a turn. Stop and review before saving.')
 return {'min':lo+margin,'max':hi-margin,'measured_min':lo,'measured_max':hi,'margin':margin,'motor_id':7,'model':'sts3215'}

def tool_targets(held,now,last,position,limits,dt):
 live=now-last<.35
 tire=held.get('tire',0) if live else 0
 socket=held.get('socket',0) if live else 0
 target=max(limits['min'],min(limits['max'],position+tire*350*min(dt,.05)))
 return target,socket

def jog_target(position,target,direction,dt):
 # Keep a finite directional goal ahead of feedback, with no software slew limit.
 # Goal_Velocity=0 selects the motor maximum in position mode.
 return max(5,min(4090,position+direction*512))

class Hardware:
 def __init__(self):self.follower=None;self.leader=None;self.port=None;self.packet=None;self.raw=False
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
  self.write(7,40,0,1);self.write(8,40,0,1)
 def check_tools(self):
  for mid in (7,8):
   m,c,e=self.packet.ping(self.port,mid)
   if c or e or m!=777:raise RuntimeError(f'Tool motor {mid} did not identify as STS3215')
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
 def connect_teleop(self,limits):
  from lerobot.robots.so_follower import SO101Follower,SO101FollowerConfig
  from lerobot.teleoperators.so_leader import SO101Leader,SO101LeaderConfig
  self.idle_lelab()
  self.follower=SO101Follower(SO101FollowerConfig(port=FOLLOWER,id='hiwonder_follower_fresh',calibration_dir=ROOT/'calibration/follower',use_degrees=True,max_relative_target=None,cameras={}))
  self.leader=SO101Leader(SO101LeaderConfig(port=LEADER,id='hiwonder_leader_fresh',calibration_dir=ROOT/'calibration/leader',use_degrees=True))
  self.follower.bus.connect();self.port=self.follower.bus.port_handler;self.packet=self.follower.bus.packet_handler
  self.check_tools()
  self.leader.bus.connect()
  self.follower.bus.disable_torque();self.follower.bus.write_calibration(self.follower.calibration)
  self.leader.bus.write_calibration(self.leader.calibration);self.leader.configure();self.follower.configure()
  # Seed hold targets, then enable. First leader action is still direct, without slow-test slew limiting.
  initial=self.follower.bus.sync_read('Present_Position',num_retry=2)
  self.follower.bus.sync_write('Goal_Position',initial)
  for name in self.follower.bus.motors:self.follower.bus.write('Goal_Velocity',name,0)
  pos=self.read(7,56)
  if not limits['min']<=pos<=limits['max']:raise RuntimeError('Tire tool is outside its calibrated working range. Place it inside the range first.')
  for mid,mode in [(7,0),(8,1)]:
   self.write(mid,40,0,1);self.write(mid,55,0,1);self.write(mid,33,mode,1)
   self.write(mid,41,100,1);self.write(mid,48,300)
   if mid==7:self.write(mid,42,pos);self.write(mid,46,350)
   else:self.write(mid,46,0)
   self.write(mid,55,1,1);self.write(mid,40,1,1)
  self.follower.bus.enable_torque();return pos
 def track(self):
  p=self.leader.bus.sync_read('Present_Position',num_retry=2)
  self.follower.send_action({name+'.pos':v for name,v in p.items()})
 def close(self):
  errors=[]
  if self.port and self.port.is_open:
   for mid in (8,7):
    try:
     if mid==8 and self.read(8,33,1)==1:self.write(8,46,0)
    except Exception as e:errors.append(str(e))
    try:self.write(mid,40,0,1)
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
  if self.raw and self.port:self.port.closePort()
  return errors

class Controller:
 def __init__(self):
  self.q=queue.Queue();self.lock=threading.Lock();self.held={'tire':0,'socket':0};self.last_heartbeat=0.;self.hw=None
  self.status={'mode':'idle','error':None,'position':None,'end_a':None,'end_b':None,'calibration':self.load_cal(),'hz':0}
  threading.Thread(target=self.run,daemon=True,name='motor-owner').start()
 def load_cal(self):return json.loads(CAL.read_text()) if CAL.exists() else None
 def snapshot(self):
  with self.lock:return dict(self.status)
 def request(self,action):
  f=Future();self.q.put((action,f));return f.result(timeout=15)
 def controls(self,d):
  tire=int(d.get('tire',0));socket=int(d.get('socket',0));jog=int(d.get('jog',0))
  if tire not in (-1,0,1) or socket not in (-1,0,1) or jog not in (-1,0,1):raise ValueError('Invalid tool direction')
  speed=int(d.get('socket_speed',600))
  if not 50<=speed<=2000:raise ValueError('Socket speed must be 50–2000')
  with self.lock:self.held={'tire':tire,'socket':socket,'jog':jog,'socket_speed':speed};self.last_heartbeat=time.monotonic()
 def stop(self):
  with self.lock:self.held={'tire':0,'socket':0}
  errors=self.hw.close() if self.hw else [];self.hw=None
  self.status.update(mode='idle',hz=0)
  if errors:self.status['error']='Stop verification incomplete: '+'; '.join(errors)
 def command(self,a):
  if a=='stop':self.stop()
  elif a=='calibrate':
   if self.status['mode']!='idle':raise ValueError('Stop the current session first')
   self.hw=Hardware();self.hw.connect_cal();self.status.update(mode='calibrating',error=None,end_a=None,end_b=None)
   self.jog_active=False;self.jog_blocked=False;self.jog_tick=time.monotonic()
  elif a in ('mark_min','mark_max'):
   if self.status['mode']!='calibrating':raise ValueError('Start calibration first')
   self.status['end_a' if a=='mark_min' else 'end_b']=self.hw.read(7,56)
  elif a=='save_calibration':
   if self.status['mode']!='calibrating':raise ValueError('Start calibration first')
   if self.status['end_a'] is None or self.status['end_b'] is None:raise ValueError('Capture both endpoints first')
   limits=safe_limits(self.status['end_a'],self.status['end_b'])
   CAL.parent.mkdir(exist_ok=True);tmp=CAL.with_suffix('.tmp');tmp.write_text(json.dumps(limits,indent=2)+'\n');tmp.replace(CAL)
   self.status['calibration']=limits;self.stop()
  elif a=='start':
   if self.status['mode']!='idle':raise ValueError('Stop the current session first')
   limits=self.load_cal()
   if not limits:raise ValueError('Calibrate the tire tool first')
   self.hw=Hardware();self.target=self.hw.connect_teleop(limits);self.limits=limits
   self.last_heartbeat=time.monotonic();self.was_tire=False;self.last_socket=0;self.last_tick=time.monotonic()
   self.status.update(mode='teleop',error=None)
  else:raise ValueError('Unknown action')
  return self.snapshot()
 def run(self):
  while True:
   tick=time.monotonic()
   try:
    try:
     a,f=self.q.get_nowait()
    except queue.Empty:a=f=None
    if a:
     try:f.set_result(self.command(a))
     except ValueError as e:self.status['error']=str(e);f.set_exception(e)
     except Exception as e:self.status['error']=str(e);self.stop();f.set_exception(e)
    if self.status['mode']=='calibrating':
     position=self.hw.read(7,56);self.status['position']=position
     if not 0<=position<=4095:raise RuntimeError('Tire encoder crossed zero. Stop and review before calibration.')
     with self.lock:held=dict(self.held);last=self.last_heartbeat
     direction=held.get('jog',0) if tick-last<.35 else 0
     dt=tick-self.jog_tick;self.jog_tick=tick
     if direction and not self.jog_blocked:
      if not self.jog_active:
       self.hw.write(7,40,0,1);self.hw.write(7,55,0,1);self.hw.write(7,33,0,1)
       self.hw.write(7,41,254,1);self.hw.write(7,48,200);self.hw.write(7,46,0)
       self.hw.write(7,42,position);self.hw.write(7,55,1,1);self.hw.write(7,40,1,1)
       self.jog_goal=float(position);self.jog_started=tick;self.jog_sample=tick;self.jog_position=position
       self.jog_active=True;self.status['error']=None
      self.jog_goal=jog_target(position,self.jog_goal,direction,dt)
      self.hw.write(7,42,round(self.jog_goal))
      stopped=False
      if tick-self.jog_sample>.6:
       stopped=abs(position-self.jog_position)<2
       self.jog_sample=tick;self.jog_position=position
      if stopped or tick-self.jog_started>5 or self.jog_goal in (5,4090):
       self.hw.write(7,42,position);self.hw.write(7,40,0,1)
       self.jog_active=False;self.jog_blocked=True
       self.status['error']='Jog paused: release the button. Check the mechanism before continuing or mark this endpoint.'
     else:
      if self.jog_active:
       self.hw.write(7,42,position);self.hw.write(7,40,0,1);self.jog_active=False
      if not direction:self.jog_blocked=False
    elif self.status['mode']=='teleop':
     with self.lock:held=dict(self.held);last=self.last_heartbeat
     if tick-last>1.0:raise RuntimeError('Control page disconnected; teleoperation stopped')
     self.hw.track()
     dt=tick-self.last_tick;self.last_tick=tick
     position=self.hw.read(7,56);self.status['position']=position
     if not self.limits['min']-10<=position<=self.limits['max']+10:raise RuntimeError('Tire tool exceeded its calibrated range')
     target,socket=tool_targets(held,tick,last,self.target,self.limits,dt)
     active=held.get('tire',0)!=0 and tick-last<.35
     if active:
      # Limit command lead to prevent accumulated position error if the mechanism stalls.
      target=max(position-35,min(position+35,target));target=max(self.limits['min'],min(self.limits['max'],target))
      self.hw.write(7,42,round(target));self.target=target
     elif self.was_tire:
      self.target=max(self.limits['min'],min(self.limits['max'],position));self.hw.write(7,42,round(self.target))
     self.was_tire=active
     speed=socket*held.get('socket_speed',600)
     if speed!=self.last_socket:
      self.hw.write(8,46,abs(speed)|(32768 if speed<0 else 0));self.last_socket=speed
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
 print('Mayter Teleop: http://127.0.0.1:8010 — motors idle until Start or Calibration is pressed',flush=True)
 try:server.serve_forever()
 finally:controller.request('stop');server.server_close()
