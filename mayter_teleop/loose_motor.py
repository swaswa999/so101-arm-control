"""Standalone ID 1 motor panel. Never starts arm tracking."""
import atexit,json,threading,time
from pathlib import Path
from http.server import ThreadingHTTPServer
import app

VERSION='loose-motor-1-20261022'
class Motor(app.Hardware):
 def connect(self):
  import scservo_sdk as sdk
  self.port=sdk.PortHandler(app.FOLLOWER);self.packet=sdk.PacketHandler(0);self.raw=True
  if not self.port.openPort():raise RuntimeError('Connect the loose motor USB controller')
  if not self.port.setBaudRate(1000000):raise RuntimeError('Cannot set USB baudrate')
  model,comm,error=self.packet.ping(self.port,1)
  if comm or error or model!=777:raise RuntimeError('No healthy STS3215 motor found at ID 1')
  for mid in range(2,9):
   _,comm,_=self.packet.ping(self.port,mid)
   if comm==0:raise RuntimeError('Another motor is connected. Connect only the loose motor.')
  self.tool_ids_checked.add(1)
  self.write(1,40,0,1)
  self.write(1,55,0,1)
  try:
   self.write(1,33,1,1)
   if self.read(1,33,1)!=1:raise RuntimeError('Speed mode did not verify')
  finally:self.write(1,55,1,1)
  self.write(1,41,100,1);self.write(1,48,1000);self.write(1,46,0)
  self.write(1,40,1,1)

class Controller:
 def __init__(self):
  self.lock=threading.RLock();self.hw=None;self.last=0
  self.status={'controller_version':VERSION,'mode':'idle','motor_id':1,'speed':0,'error':None}
  threading.Thread(target=self.watchdog,daemon=True).start()
 def snapshot(self):
  with self.lock:return dict(self.status)
 def stop(self):
  with self.lock:
   errors=self.hw.close() if self.hw else []
   self.hw=None;self.status.update(mode='idle',speed=0)
   if errors:self.status['error']='; '.join(errors)
 def request(self,action):
  if action!='stop':raise ValueError('Use the loose motor direction buttons')
  self.stop();return self.snapshot()
 def controls(self,data):
  direction=data.get('direction',0)
  if type(direction) is not int or direction not in (-1,0,1):raise ValueError('Invalid direction')
  with self.lock:
   if direction==0:self.stop();return
   try:
    if self.hw is None:self.hw=Motor();self.hw.connect()
    speed=direction*2000
    if self.status['speed'] and self.status['speed']*speed<0:
     self.hw.write(1,46,0);time.sleep(.15)
    self.hw.write(1,46,app.encode_velocity(speed))
    self.last=time.monotonic();self.status.update(mode='running',speed=speed,error=None)
   except Exception as e:
    self.status['error']=str(e);self.stop();raise
 def watchdog(self):
  while True:
   with self.lock:
    if self.hw and time.monotonic()-self.last>=.35:self.stop()
   time.sleep(.03)

class Handler(app.Handler):
 def do_GET(self):
  if self.path=='/':self.send(Path(__file__).with_name('loose_motor.html').read_text(),kind='text/html; charset=utf-8')
  elif self.path=='/status':self.send(app.controller.snapshot())
  else:self.send({'error':'Not found'},404)

if __name__=='__main__':
 app.controller=Controller();atexit.register(app.controller.stop)
 server=ThreadingHTTPServer(('127.0.0.1',8010),Handler)
 print('Loose motor ID 1: http://127.0.0.1:8010 — idle until a direction is clicked',flush=True)
 try:server.serve_forever()
 finally:app.controller.stop();server.server_close()
