# Piper planning and control workspace

ROS 2 Humble workspace for the Piper elevator-button pressing project. The
current engineering scope is planning and control from an explicit world-frame
target: approach, align, press, retreat, and return.

## Repository layout

- `src/Piper_ros`: Piper driver, robot description, Gazebo scene, MoveIt
  configuration, and project packages.
- `src/third_party/moveit_task_constructor`: pinned MoveIt Task Constructor
  dependency for ROS 2 Humble.
- `src/third_party/py_binding_tools`: pinned build dependency.
- `docker`: reproducible Ubuntu 22.04 / ROS 2 Humble development environment.
- `examples`: small client examples.

The source directories above keep their upstream Git histories and are pinned
by this workspace repository. Local build products, generated data, trained
weights, and machine-specific configuration are intentionally ignored.

The Piper and MoveIt Task Constructor submodules currently point to local
project commits on `piper-press-dev` and `piper-humble-build`. Before sharing a
fresh clone, push those branches to project-owned forks and update the matching
URLs in `.gitmodules`.

## Perception status

Perception is not part of the current planning/control baseline. The
`piper_button_perception` neural-network prototype was removed on 2026-09-28
together with its training environment, its generated datasets, and its trained
weights.

The next perception effort targets a **calibration board** rather than a learned
detector. Whatever the detector is, it only has to produce a 6-DoF pose in the
planning frame; the manipulation stack keeps consuming an explicit world-frame
target and does not change when the detector does.

The target contract is documented in
[`docs/perception_interface.md`](src/Piper_ros/src/piper_elevator_manipulation/docs/perception_interface.md):
publish a `geometry_msgs/msg/PoseStamped` on `/press/target_pose` whose
orientation's +Z axis is the press direction.

## Build

See [`docker/README.md`](docker/README.md) for container setup and simulation
commands.
