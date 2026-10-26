import threading,unittest
from concurrent.futures import Future
from unittest.mock import patch
from app import Controller,arm_support

class Startup(unittest.TestCase):
 def test_loading_start_never_opens_hardware_and_tools_command_can_follow(self):
  with patch.object(threading.Thread,'start'):c=Controller()
  with patch('app.arm_support',side_effect=ValueError('Arm software is loading')),patch('app.Hardware') as hardware:
   with self.assertRaisesRegex(ValueError,'loading'):c.command('start')
   hardware.assert_not_called()
   c.command('tools')
   hardware.return_value.connect_tools.assert_called_once()
   hardware.return_value.connect_teleop.assert_not_called()
 def test_cancelled_queued_action_never_runs(self):
  with patch.object(threading.Thread,'start'):c=Controller()
  f=Future();f.cancel();c.q.put(('start',f))
  with patch.object(c,'command') as command,patch('app.time.sleep',side_effect=InterruptedError):
   with self.assertRaises(InterruptedError):c.run()
   command.assert_not_called()
 def test_dependency_loading_happens_on_separate_thread(self):
  with patch('app._arm_support',None),patch('app._arm_error',None),patch('app._arm_loading',False),patch('app.threading.Thread') as thread:
   with self.assertRaisesRegex(ValueError,'loading'):arm_support()
   with self.assertRaisesRegex(ValueError,'loading'):arm_support()
   thread.return_value.start.assert_called_once()
if __name__=='__main__':unittest.main()
