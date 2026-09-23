---
name: piper-press-planning-control
description: Guides architecture, implementation, review, and testing of the Piper five-axis elevator-button planning and control system. Use whenever changing Piper URDF/SRDF, MoveIt, MTC, BehaviorTree.CPP, ros2_control, press-task interfaces, contact handling, simulation, or related documentation.
---

# Piper Press Planning and Control

## Required reading

Before changing relevant code, read [ARCHITECTURE.md](ARCHITECTURE.md) completely. Treat it as the project architecture contract. If a request conflicts with it, identify the conflict before editing instead of silently changing the technical route.

## Non-negotiable requirements

Keep these user requirements verbatim:

> 理想按压过程分三段：**自由段**用预录路点（受阻时回退采样式规划）；**接近段**沿面板法向做笛卡尔直线； **按压段**用接触检测停止。

> 代码的编写要遵循可读性强，模块化，边界清晰的原则，初学者也要能看得懂。

Also preserve these system invariants:

- J6 is mechanically unavailable because the press assembly is fixed to the J6 motor stator. Model it as fixed; never restore a fictitious actuated J6 to make IK easier.
- The task target is five-dimensional: world-frame TCP position plus panel-normal direction. Rotation around the symmetric press rod is free.
- Keep URDF/Xacro, SRDF, MoveIt groups, ros2_control, Gazebo controllers, joint limits, and task code consistent with active joints J1–J5 and TCP `press_tcp`.
- Perception is paused. Keep its code for reference, disabled by default and outside planning/control dependencies. Do not delete it or couple the press task to a specific detector.
- Hardware execution must default to non-executing/dry-run behavior until the user explicitly enables motion.

## Change workflow

1. Classify the change as task, planning, execution, robot model, simulation, or interface work.
2. State the layer boundary and input/output contract before implementing a cross-layer change.
3. Check whether the change preserves the three motion segments.
4. Keep policy out of lower layers:
   - Behavior Tree decides what happens next, retry, cancel, and recovery.
   - MoveIt/MTC decides geometric motion and produces validated trajectories.
   - ros2_control executes commands and reports controller/contact state.
5. Put tunable values in parameters or YAML, not magic constants embedded in stage logic.
6. Build and test the smallest affected package, then run the complete dry-run sequence.
7. Only run simulated execution after dry-run succeeds. Do not infer permission to move hardware.
8. Update architecture documentation when a public interface, phase boundary, frame, controller, or recovery rule changes.

## Implementation rules

- Prefer small types with one responsibility: request validation, target transforms, free-motion planning, Cartesian approach, contact monitoring, recovery, and orchestration should be separable.
- Use explicit names such as `panel_normal`, `pre_contact_pose`, `contact_threshold`, and `return_state`; avoid ambiguous names such as `pose1`, `step`, or `data`.
- Keep ROS callbacks thin. Put reusable algorithms in ordinary C++ classes that can be unit-tested without launching ROS.
- Explain safety assumptions, frame conventions, and non-obvious mathematical choices. Avoid comments that merely repeat the code.
- Validate frames, finite values, normalized normals, reachability, joint limits, collision state, timeouts, and stale inputs at boundaries.
- Return structured failure causes. Do not collapse IK failure, collision, contact timeout, controller failure, and cancellation into one generic error.
- Prefer deterministic tests and seeded sampling.
- Preserve the existing upstream six-axis/gripper configuration; add press-specific behavior in press packages/configuration.

## Forbidden shortcuts

- Do not use one monolithic node to own Behavior Tree policy, planning, controller execution, and contact detection.
- Do not use joint interpolation as the final implementation of the panel-normal Cartesian approach.
- Do not declare press success after a fixed travel distance on hardware. Distance/time limits are safety caps; contact detection is the success condition.
- Do not call OMPL first during normal free motion when a valid prerecorded route exists. OMPL is the obstruction fallback.
- Do not let the execution layer select targets, retries, or task order.
- Do not require an arbitrary full 6D TCP orientation from a five-axis mechanism.
- Do not hide model mismatches with fake joints, disabled validation, or broad collision exemptions.

## Review checklist

- [ ] Change belongs to the intended layer.
- [ ] Three-segment motion semantics remain intact.
- [ ] J1–J5 and `press_tcp` remain consistent across model/config/controller code.
- [ ] World-frame point and panel normal are transformed and validated explicitly.
- [ ] Normal path uses prerecorded free-space waypoints before sampling fallback.
- [ ] Approach is a collision-checked Cartesian line along the panel normal.
- [ ] Press termination comes from a contact-monitor abstraction with timeout/travel safety caps.
- [ ] Cancel and failure paths stop motion and enter a defined retreat/recovery path.
- [ ] Defaults are safe, parameterized, and understandable to a new developer.
- [ ] Build, dry-run, and relevant automated tests pass.

## Transitional code rule

The current `piper_elevator_manipulation` demo is a verified integration baseline, not the finished controller. Its short contact motions still use joint interpolation and press completion still uses configured depth. Preserve it while replacing these pieces behind clear interfaces; do not present those transitional choices as the target architecture.
