# flexiv_calibration

Syncs the actual kinematic parameters of a connected Flexiv robot into the kinematics YAML file
that `flexiv_description` loads, so that the generated URDF describes that specific robot instead
of the nominal model.

## calibration_correction

By default this updates `flexiv_description/config/[robot_type]/default_kinematics.yaml` in place,
which is the file every launch file already reads. Nothing else has to change afterwards:

```bash
ros2 launch flexiv_calibration calibration_correction.launch.py robot_sn:=[robot_sn]
```

Launch arguments:

- `robot_sn` (*required*) - serial number of the robot to connect to. Remove any space, for example: Enlight-L-123456
- `robot_type` (default: *empty*) - type of the Flexiv robot, which selects the kinematics file to update. Defaults to the product model reported by the robot.
- `target_filename` (default: *empty*) - write the synced parameters here instead of updating `flexiv_description`. Use this to keep several robots of the same type side by side, and pass the file back with the driver's `kinematics_params_file` argument. This is a destination only: it is never read, and any existing content is replaced.
- `template_filename` (default: *empty*) - template kinematics YAML file to sync. It supplies the joint list, so only the joints it names get measured values. Defaults to `flexiv_description/config/[robot_type]/default_kinematics.yaml`.

Notes:

- The sync fills values into the keys the template already lists; it never adds joints. A template that is empty, or that does not name this robot's joints, syncs nothing — the run fails and no file is written.
- Reading kinematic parameters requires an RDK professional license.
- Run this once per robot, and again whenever the robot is re-calibrated or repaired. A dual-arm robot is one RDK connection and one serial number, so one run covers it.
- Updating `flexiv_description` in place shows up as a local change in that repository, and a workspace built with `--symlink-install` updates the source checkout. Commit it, or use `target_filename` to keep the repository pristine.
- One file describes the whole robot. A single-arm model's file lists the joints directly; a dual-arm model's carries an `ARM_1` and an `ARM_2` section, plus `EXT_AXIS` for the MICO-Plus and MICO-Ultra torso. A single run therefore calibrates both arms, and `target_filename` and the driver's `kinematics_params_file` argument work for every robot type.
- A `calibration_metadata` block recording the robot serial number is appended to the file. It is rewritten rather than repeated when the same file is synced again.

See the [repository README](../README.md#robot-calibration) for the driver-side arguments.
