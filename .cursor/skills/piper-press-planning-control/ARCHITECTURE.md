# Piper elevator-button planning and control architecture

## 1. System objective

Given a target in the world coordinate system, command the five-axis Piper arm to approach the elevator panel, align the symmetric press rod with the panel normal, press until contact is detected, retreat safely, and return to its recorded start state.

The planning/control stack receives a validated geometric target. Button recognition is outside the current scope.

## 2. Canonical technical route

- MoveIt 2 + MoveIt Task Constructor (MTC) perform robot modeling, collision checking, five-dimensional IK integration, motion planning, and manipulation-stage construction.
- BehaviorTree.CPP + BehaviorTree.ROS2 perform task orchestration, retry, cancellation, timeout handling, and recovery.
- A thin elevator press task framework owns domain data and stable interfaces without reimplementing MoveIt or ros2_control.
- ros2_control and its trajectory controllers execute trajectories. A contact-monitor interface reports contact independently of task policy.
- Gazebo provides the simulation plant, scene, sensors, controllers, and contact-test adapters. RViz provides robot, planning-scene, trajectory, TF, and state visualization.

## 3. Three planning/control layers

### Task layer — decide “what to do”

Technology: BehaviorTree.CPP + BehaviorTree.ROS2.

Responsibilities:

- validate the request and system readiness;
- record the return state;
- call free-motion, approach, and press capabilities in order;
- select retry and recovery paths;
- propagate cancellation and timeouts;
- report phase-level feedback and structured results.

It must not calculate trajectories or command motors directly.

Recommended Behavior Tree:

```text
ValidateTarget
  -> CheckSystemReady
  -> CaptureReturnState
  -> FreeMotion
       prerecorded route
       fallback: sampling planner if route is obstructed
  -> CartesianApproach
  -> PressUntilContact
  -> Retreat
  -> ReturnToCapturedState
  -> ReportSuccess
```

Recovery behavior:

- prerecorded route obstructed: invoke sampling-based fallback;
- planning failure before contact: remain stopped, then return or report unreachable;
- approach failure: retreat along the verified incoming path when possible;
- contact timeout or maximum travel exceeded: stop, retreat, report `CONTACT_NOT_DETECTED`;
- controller failure: cancel active goal, hold, then execute only a validated recovery;
- user cancellation: propagate cancel immediately and report the final safe state.

### Planning layer — decide “how to move”

Technology: MoveIt 2 + MTC.

Responsibilities:

- transform the target into the current robot planning frame;
- perform collision checking and five-dimensional IK;
- construct motion stages and trajectories;
- enforce joint, velocity, acceleration, workspace, and collision limits;
- return plans and structured planning failures to the task layer.

The ideal motion has three segments:

1. **Free segment**
   - Normal route: select and replay a validated prerecorded waypoint route.
   - Validate every route against the current planning scene before execution.
   - If blocked or invalid, fall back to a sampling planner such as OMPL/RRTConnect.
   - Do not silently skip collision validation because the route was prerecorded.

2. **Approach segment**
   - Begin from a collision-free pre-approach/pre-contact state.
   - Move the TCP along the panel normal using a Cartesian straight-line planner.
   - Maintain TCP press-axis alignment while leaving roll around the rod unconstrained.
   - Require a minimum Cartesian completion fraction and reject excessive joint jumps.

3. **Press segment**
   - Advance at a separately limited low speed.
   - Terminate successfully when the contact-monitor interface reports stable contact.
   - Treat maximum travel, force/effort limit, and timeout as safety stops, not contact success.
   - Retreat along the panel normal after contact or a recoverable stop.

MTC owns the geometric sequence. The Behavior Tree owns macro-level orchestration and recovery; do not duplicate the same state machine in both.

### Execution layer — decide “how to control”

Technology: ros2_control, `joint_trajectory_controller`, and contact-monitor adapters.

Responsibilities:

- accept only validated trajectories or bounded press commands;
- send J1–J5 commands and publish joint/controller state;
- enforce controller limits and report execution status;
- expose contact state without deciding task policy;
- support clean cancellation and stopping.

Possible contact sources are force/torque sensing, motor current/effort, a mechanical switch, or a simulator contact/button-joint signal. Hide the source behind one interface so planning and Behavior Tree code do not change when hardware sensing changes.

## 4. Stable data contracts

### Press target

Required semantic fields:

- timestamp;
- source frame;
- TCP target point;
- unit panel normal / press direction;
- optional target identifier for logging only.

The target is 5D: three position dimensions plus two independent direction dimensions. Roll around `press_tcp` local Z is free.

Reject stale targets, unknown frames, non-finite values, zero-length normals, unreachable targets, and targets outside configured workspace limits.

### Press task action

Recommended action semantics:

- Goal: press target and optional policy/profile name.
- Feedback: current phase, retry count, path source (`prerecorded` or `sampling`), distance to target, contact state.
- Result: stable error code, message, whether contact occurred, final phase, and whether return/recovery completed.

Suggested result categories:

```text
SUCCESS
INVALID_TARGET
TF_UNAVAILABLE
UNREACHABLE
FREE_PATH_BLOCKED
APPROACH_FAILED
CONTACT_NOT_DETECTED
CONTACT_LIMIT_EXCEEDED
CONTROLLER_FAILED
CANCELLED
RECOVERY_FAILED
```

## 5. Five-axis robot-model contract

- Active motion joints: `joint1`–`joint5`.
- J6 rotor geometry may remain for mass/collision fidelity, but its joint is fixed.
- The stator-mounted press structure branches from the J6 stator frame.
- Task TCP: `press_tcp`; local +Z is the press direction.
- MoveIt group: `arm`, chain `base_link` to `press_tcp`.
- Controller: `arm_controller`, J1–J5 only.
- No gripper joints or gripper commands exist in the press configuration.

Any change to mounting transform, link dimensions, active joints, limits, or TCP requires synchronized review of:

- `piper_description` Xacro/URDF and collision geometry;
- `piper_press_moveit` SRDF, kinematics, joint limits, and controller mapping;
- Gazebo ros2_control configuration;
- task/planning code and tests;
- hardware driver/controller configuration.

## 6. Package boundaries

Current packages:

- `piper_description`: robot and press-tool geometry only.
- `piper_gazebo`: simulation plant, scene, simulated sensors/controllers.
- `piper_press_moveit`: five-axis MoveIt configuration only.
- `piper_elevator_manipulation`: current planning algorithm and MTC integration baseline.

Target evolution:

- Keep reusable 5D IK and geometric planning classes independent of ROS callbacks.
- Add a planning capability/action server around MTC rather than growing the demo main function.
- Put Behavior Tree orchestration in a task package or clearly isolated task module.
- Put contact-source adapters in execution/simulation modules behind a common interface.
- Put shared action/message definitions in a small interface package if cross-package APIs are introduced.

Dependency direction must remain:

```text
task orchestration
      -> planning capability
      -> stable interfaces

planning capability -> MoveIt/MTC + robot model
execution adapters  -> ros2_control + stable interfaces
simulation adapters -> Gazebo + stable interfaces
```

The planning package must not depend on perception implementations. The execution layer must not depend on Behavior Tree policy.

## 7. Readability and maintenance standard

- One class/module should have one clear reason to change.
- Prefer request/result types and explicit interfaces over shared mutable state.
- Keep launch files declarative; keep algorithms out of launch code.
- Put thresholds, route names, speed profiles, and safety limits in validated parameters/YAML.
- Use units in names or documentation and state every coordinate frame explicitly.
- Keep functions short enough that a new developer can understand their inputs, decisions, and outputs without following hidden side effects.
- Add focused unit tests for math and state transitions, integration tests for planning, and simulation tests for execution/recovery.

## 8. Current transition gaps

The current integrated demo proves that the five-axis model, custom 5D IK, MTC, MoveIt controller mapping, and Gazebo execution are connected. It is not yet the final architecture:

- free motion currently calls sampling planning directly instead of first using prerecorded routes;
- contact approach currently uses short joint-interpolated waypoint motions instead of a true panel-normal Cartesian planner;
- press completion currently uses configured travel depth instead of contact detection;
- orchestration is still a sequential MTC demo rather than a BehaviorTree.CPP action server;
- the simulator button/contact signal is not yet exposed through the future contact-monitor interface.

Address these gaps in that order unless a safety or hardware-interface dependency requires a different sequence.
