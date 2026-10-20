# Third-party source

## LeLab

- Upstream: https://github.com/huggingface/leLab
- Imported revision: `9b5afaee33baa1d773e499937564f3831d25de22`
- Location: `lelab/`
- License: Apache 2.0, retained at `lelab/LICENSE`

This repository includes the local customized working copy. Changes from the
upstream revision cover:

- Camera preview resolution and frame-rate constraints.
- Passing camera settings through preview and recording components.
- AVFoundation capture at 30 fps for the original Mac USB cameras when the
  requested dataset rate is 15 fps.
- A 30 Hz teleoperation loop with read retries, cached joint positions, and
  reporting the last teleoperation error.
- Tests for the local recording and teleoperation behavior.

Modified upstream files are marked with a short notice. Original copyright and
license notices are retained.

## LeRobot and Feetech SDK

The controller depends on LeRobot and the Feetech servo SDK through
`requirements.txt`. Their installed environments are not included in this
repository.
