import threading,time,unittest
from unittest.mock import patch
import loose_motor as m
class Tests(unittest.TestCase):
 def controller(self):
  with patch.object(threading.Thread,'start'):return m.Controller()
 def test_directions_stop_and_no_startup_motion(self):
  c=self.controller();self.assertIsNone(c.hw)
  with patch.object(m,'Motor') as cls:
   h=cls.return_value;h.close.return_value=[]
   c.controls({'direction':1});h.write.assert_called_with(1,46,2000)
   c.controls({'direction':-1});h.write.assert_called_with(1,46,32768+2000)
   self.assertIn(((1,46,0),),h.write.call_args_list)
   c.controls({'direction':0});h.close.assert_called_once();self.assertIsNone(c.hw)
 def test_failed_connection_is_closed(self):
  c=self.controller()
  with patch.object(m,'Motor') as cls:
   cls.return_value.connect.side_effect=RuntimeError('Missing motor');cls.return_value.close.return_value=[]
   with self.assertRaises(RuntimeError):c.controls({'direction':1})
   cls.return_value.close.assert_called_once();self.assertEqual(c.status['speed'],0)
 def test_watchdog_stops_stale_command(self):
  c=self.controller()
  with patch.object(m,'Motor') as cls:
   cls.return_value.close.return_value=[]
   c.controls({'direction':1});c.last=time.monotonic()-1
   with patch.object(m.time,'sleep',side_effect=InterruptedError):
    with self.assertRaises(InterruptedError):c.watchdog()
   self.assertIsNone(c.hw);self.assertEqual(c.status['speed'],0)
 def test_invalid_direction_never_connects(self):
  c=self.controller()
  with self.assertRaises(ValueError):c.controls({'direction':8})
  self.assertIsNone(c.hw)
if __name__=='__main__':unittest.main()
