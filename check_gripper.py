"""Bounded follower gripper diagnostic; never enables the other joints."""
import json
import argparse
import time
from pathlib import Path
import scservo_sdk as sdk

ROOT = Path(__file__).resolve().parent
PORT = '/dev/cu.usbmodem5C821107831'
MOTOR = 6
port = sdk.PortHandler(PORT)
packet = sdk.PacketHandler(0)

def read(address, size=2):
    fn = packet.read1ByteTxRx if size == 1 else packet.read2ByteTxRx
    value, comm, error = fn(port, MOTOR, address)
    if comm or error:
        raise RuntimeError(f'Read failed: communication={comm}, motor status={error}')
    return value

def write(address, value, size=2):
    fn = packet.write1ByteTxRx if size == 1 else packet.write2ByteTxRx
    comm, error = fn(port, MOTOR, address, value)
    if comm or error:
        raise RuntimeError(f'Write failed: communication={comm}, motor status={error}')

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--travel', type=int, choices=[100, 300], default=100)
    args = parser.parse_args()
    if not port.openPort():
        raise RuntimeError('Cannot open follower connection')
    samples = []
    armed = False
    try:
        port.setBaudRate(1000000)
        if read(40, 1) != 0 or read(33, 1) != 0:
            raise RuntimeError('Expected gripper torque off and position mode')
        if not 90 <= read(62, 1) <= 126 or read(63, 1) >= 65:
            raise RuntimeError('Voltage or temperature outside diagnostic bounds')
        cal = json.loads((ROOT/'calibration/follower/hiwonder_follower_fresh.json').read_text())['gripper']
        start = read(56)
        low, high = cal['range_min'] + 15, cal['range_max'] - 15
        target = start + args.travel if start < (low + high)/2 else start - args.travel
        target = max(low, min(high, target))
        if abs(target-start) < 30:
            raise RuntimeError('Not enough calibrated travel for diagnostic')
        write(42, start)
        armed = True
        write(40, 1, 1)
        print('Gripper torque enabled:', read(40, 1), flush=True)
        steps = 50 if args.travel == 300 else 30
        for step in range(1, steps + 1):
            goal = round(start + (target-start)*step/steps)
            write(42, goal)
            time.sleep(0.1)
            actual = read(56)
            samples.append({'goal':goal,'actual':actual,'torque':read(40,1)})
            if abs(actual-start) > args.travel + 40:
                raise RuntimeError('Unexpected travel; stopping')
        time.sleep(0.5)
        final = read(56)
        print(json.dumps({'start':start,'target':target,'final':final,'travel_counts':final-start}),flush=True)
    finally:
        try:
            if armed:
                write(40, 0, 1)
                print('Gripper torque disabled',flush=True)
        finally:
            port.closePort()
            (ROOT/'gripper_diagnostic.json').write_text(json.dumps(samples,indent=2)+'\n')
