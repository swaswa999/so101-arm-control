"""Short LeRobot tracking session with target slew limits and feedback logging."""
import json
import logging
import time
from pathlib import Path
import scservo_sdk as sdk
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

ROOT = Path(__file__).resolve().parent
FOLLOWER_PORT = '/dev/cu.usbmodem5C821107831'

def verify_torque_off():
    port = sdk.PortHandler(FOLLOWER_PORT)
    handler = sdk.PacketHandler(0)
    if not port.openPort():
        raise RuntimeError('Cannot verify follower torque; disconnect motor power')
    try:
        port.setBaudRate(1000000)
        failures = []
        for motor in range(1, 7):
            handler.write1ByteTxRx(port, motor, 40, 0)
            value, comm, error = handler.read1ByteTxRx(port, motor, 40)
            # A fault status may accompany a successful torque-off write.
            if comm or value != 0:
                failures.append(motor)
            if error:
                print(f'Motor {motor} reports status {error}; no further motion tests.', flush=True)
        if failures:
            raise RuntimeError(f'Torque-off not confirmed for {failures}; disconnect motor power')
        print('All six follower motors confirmed torque off.', flush=True)
    finally:
        port.closePort()

def prepare_hold():
    """Seed goals from actual positions before LeRobot enables holding torque."""
    port = sdk.PortHandler(FOLLOWER_PORT)
    handler = sdk.PacketHandler(0)
    if not port.openPort():
        raise RuntimeError('Cannot open follower')
    try:
        port.setBaudRate(1000000)
        positions = {}
        for motor in range(1, 7):
            for address, low, high in [(40, 0, 0), (33, 0, 0), (62, 90, 126), (63, 0, 64)]:
                value, comm, error = handler.read1ByteTxRx(port, motor, address)
                if comm or error or not low <= value <= high:
                    raise RuntimeError(f'Follower {motor}: preflight failed at register {address}')
            pos, comm, error = handler.read2ByteTxRx(port, motor, 56)
            if comm or error:
                raise RuntimeError('Position read failed')
            positions[motor] = pos
        for motor, pos in positions.items():
            comm, error = handler.write2ByteTxRx(port, motor, 42, pos)
            if comm or error:
                raise RuntimeError('Initial hold target failed')
    finally:
        port.closePort()

def main():
    leader = SO101Leader(SO101LeaderConfig(
        port='/dev/cu.usbmodem5B3E0915611', id='hiwonder_leader_fresh',
        calibration_dir=ROOT/'calibration/leader', use_degrees=True))
    follower = SO101Follower(SO101FollowerConfig(
        port=FOLLOWER_PORT, id='hiwonder_follower_fresh',
        calibration_dir=ROOT/'calibration/follower', use_degrees=True,
        max_relative_target=8.0, cameras={}))
    samples = []
    try:
        prepare_hold()
        leader.connect()
        follower.connect()
        initial = follower.get_observation()
        commanded = {key: value for key, value in initial.items() if key.endswith('.pos')}
        started = previous = time.monotonic()
        print('Tracking active for 30 seconds. Move the leader slowly.', flush=True)
        while time.monotonic() - started < 30:
            tick = time.monotonic()
            dt = min(tick-previous, 0.1)
            previous = tick
            desired = leader.get_action()
            # Empty-gripper commissioning test: avoid both mechanical endpoints.
            desired['gripper.pos'] = max(10.0, min(90.0, desired['gripper.pos']))
            target = {}
            for key, value in commanded.items():
                # Body: degrees/second. Gripper: percentage points/second.
                limit = 10.0 * dt
                target[key] = value + max(-limit, min(limit, desired[key]-value))
            sent = follower.send_action(target)
            actual = follower.get_observation()
            commanded = sent
            samples.append({'time_s':tick-started,'leader':desired,'sent':sent,'actual':actual})
            if len(samples) % 20 == 0:
                follower.bus.read('Present_Temperature', 'gripper', normalize=False)
            time.sleep(max(0, 0.05-(time.monotonic()-tick)))
    finally:
        try:
            if follower.is_connected:
                try:
                    follower.disconnect()
                except Exception as exc:
                    print(f'Normal disconnect reported: {exc}', flush=True)
                    follower.bus.disconnect(False)
        finally:
            try:
                if leader.is_connected:
                    leader.disconnect()
            finally:
                (ROOT/'slow_teleop_feedback.json').write_text(json.dumps(samples,indent=2)+'\n')
                verify_torque_off()
        print('Tracking stopped.',flush=True)
    if samples:
        travel = {}
        for key in samples[0]['sent']:
            values = [sample['actual'][key] for sample in samples]
            travel[key] = round(max(values)-min(values),2)
        print('Measured joint travel:',json.dumps(travel),flush=True)

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    main()
