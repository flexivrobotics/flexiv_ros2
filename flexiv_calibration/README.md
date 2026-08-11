# flexiv_calibration

Extracts the actual kinematic parameters of a connected Flexiv robot into a kinematics YAML
file that `flexiv_description` can load, so that the generated URDF describes that specific
robot instead of the nominal model.

## calibration_correction

Copies a template kinematics YAML, then syncs the connected robot's measured parameters into
the copy and appends a `calibration_metadata` block recording which robot it came from.

```bash
ros2 launch flexiv_calibration calibration_correction.launch.py robot_sn:=[robot_sn] target_filename:=$HOME/flexiv_calib/[robot_sn]_kinematics.yaml
```

Launch arguments:

- `robot_sn` (*required*) - serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
- `target_filename` (*required*) - path of the kinematics YAML file to write. Must be outside `flexiv_description`.
- `robot_type` (default: *empty*) - type of the Flexiv robot, used to pick the template. Defaults to the model name reported by the robot.
- `template_filename` (default: *empty*) - template kinematics YAML file to copy. Defaults to `flexiv_description/config/[robot_type]/default_kinematics.yaml`.
- `overwrite` (default: *false*) - replace the target file if it already exists.

Notes:

- Reading kinematic parameters requires an RDK professional license.
- The sync rewrites its input file in place, so the node always works on a copy. It refuses to
  write anywhere inside `flexiv_description`, including the source checkout that a
  `--symlink-install` workspace points back at.
- Run the extraction once per robot, and again whenever the robot is re-calibrated or repaired.
  For a dual robot setup, run it once per serial number.

Pass the resulting file to the driver with the `kinematics_params_file` launch argument
(`kinematics_params_file_left` / `_right` for dual robot setups). See the
[repository README](../README.md#robot-calibration).
