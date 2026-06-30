# so101_description

This folder is a clean starter bundle extracted from `SO-ARM100` for building your own SO-101 URDF and later a ROS 2 description package / MoveIt config.

## What is included

- `urdf/`
  - `so101_new_calib.urdf`
  - `so101_old_calib.urdf`
  - `so101_new_calib.xml`
  - `so101_old_calib.xml`
  - `joints_properties.xml`
  - `scene.xml`
- `assets/`
  - the mesh assets referenced by the URDF
- `step/`
  - CAD STEP sources for the SO-101 parts
- `stl/`
  - printable STL exports from the SO-ARM100 repo

## Source

Copied from:

- `SO-ARM100/Simulation/SO101`
- `SO-ARM100/STEP/SO101`
- `SO-ARM100/STL/SO101`

## Suggested next step

When you are ready, turn this into a ROS 2 package like:

- `so101_custom_description`

Then add:

- `urdf/*.xacro`
- `meshes/`
- `launch/display.launch.py`
- `package.xml`
- `CMakeLists.txt`

The URDF in `urdf/so101_new_calib.urdf` already uses relative mesh paths like `assets/*.stl`, so it is a good base for a custom description package.
