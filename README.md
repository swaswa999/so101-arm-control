# SO101 Arm Control

SO-101 leader/follower arm teleoperation with a local browser interface, a
click-to-run socket tool, jaw calibration, and a customized LeLab interface.

The current controller is `mayter_teleop/app.py`. It tracks the leader arm and
controls one STS3215 socket motor at ID **8**, using a **1,000,000 baud** bus.
The tire motor at ID 7 is disconnected and is not addressed by this controller.

## Setup

Use Python 3.12. From the project folder:

```sh
python3.12 -m venv .lelab-venv
.lelab-venv/bin/python -m pip install -r requirements.txt
```

Set `FOLLOWER` and `LEADER` near the top of `mayter_teleop/app.py` to your serial
ports. The checked-in defaults describe the original Mac setup.

Generate calibration for your own arms with LeRobot, using the IDs and output
folders expected by the controller:

```sh
.lelab-venv/bin/lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/YOUR_FOLLOWER_PORT \
  --robot.id=hiwonder_follower_fresh \
  --robot.calibration_dir=calibration/follower

.lelab-venv/bin/lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/YOUR_LEADER_PORT \
  --teleop.id=hiwonder_leader_fresh \
  --teleop.calibration_dir=calibration/leader
```

Calibration files and diagnostic logs are specific to the physical setup and are
not published. They remain available in the original local project.

## Run

On macOS, double-click **Mayter Teleop.command**, or run:

```sh
.lelab-venv/bin/python mayter_teleop/app.py
```

Open <http://127.0.0.1:8010>. Launching the server leaves the motors idle; use
**Check motor connections** before starting teleoperation. The replacement socket
motor must respond as STS3215 ID 8 before startup can succeed. The controller
seeds hold positions before enabling torque, stops socket movement on stale
controls, and reports failures to verify torque-off.

The controller uses direct leader tracking without a relative target limit.
Keep the arm's travel clear and make motor power accessible when operating it.
Only one controller should own the serial connections at a time.

## Optional LeLab interface

The `lelab/` directory includes the existing customized LeLab source. To install
it into the same environment:

```sh
.lelab-venv/bin/python -m pip install -e './lelab[test]'
.lelab-venv/bin/lelab --rebuild
```

The frontend build requires Node.js 22.12 or newer. Then use
**LeLab Terminal.command** or `./launch-lelab.sh` to open the interface on port
8000. The shell launchers resolve their project directory automatically.
`LeLab-launcher.applescript` targets `~/Desktop/Projects/so101-arm-control`.

See `lelab/README.md` for the upstream interface's documentation.

## Other files

- `mayter_teleop/index.html`: browser controls for the current socket-only setup.
- `mayter_teleop/test_controls.py`: automated tests using mocked hardware.
- `mayter_teleop/app_with_endpoint_calibration.py`: earlier two-tool controller,
  retained for reference; it does not describe the current hardware setup.
- `slow_teleop.py`: separate 30-second tracking diagnostic with slew limits.
- `check_gripper.py`: standalone gripper diagnostic.

The standalone diagnostics access hardware and are not part of the automated
test command.

## Tests

```sh
.lelab-venv/bin/python -m unittest discover -s mayter_teleop -p 'test_*.py' -v
```

These tests mock serial operations and do not move the arm.

## Attribution

LeLab and LeRobot are Hugging Face projects. The included LeLab code retains its
Apache 2.0 license in `lelab/LICENSE`; see `THIRD_PARTY.md` for its source revision
and the local modifications.

Socket controls: click **Forward · Max speed** or **Reverse · Max speed** to run
at the configured controller maximum of 2000 velocity units. **Stop socket**
stops just the tool. Page blur, hiding the page, and a stale heartbeat also stop
the tool. Physical speed depends on the motor, voltage, and load.
