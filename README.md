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
by this workspace repository. Local build products, generated perception data,
trained weights, and machine-specific configuration are intentionally ignored.

The Piper and MoveIt Task Constructor submodules currently point to local
project commits on `piper-press-dev` and `piper-humble-build`. Before sharing a
fresh clone, push those branches to project-owned forks and update the matching
URLs in `.gitmodules`.

## Perception status

The `piper_button_perception` prototype is retained but paused. It is optional,
is not started by the default planning/control launch path, and must not be a
dependency of the manipulation stack. The current target is supplied directly
in the world/planning frame.

## Build

See [`docker/README.md`](docker/README.md) for container setup and simulation
commands.
