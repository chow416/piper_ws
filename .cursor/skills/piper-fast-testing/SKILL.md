---
name: piper-fast-testing
description: Use for Docker/ROS 2 builds, simulation checks, and user-authorized physical Piper arm tests in this repository; keep tests targeted and fast while preserving real-motion boundaries.
---

# Piper fast testing

Work decisively: identify the changed boundary, run the shortest test that can
answer the current question, report the result, and move to the next useful
step. Do not turn every code change into a long hardware commissioning exercise.

## Environment

- Host workspace: `/home/zlf/piper_ws`; the Compose service `piper` mounts it
  as `/ws` in container `piper_dev`. Use the container's ROS Humble/Python 3.10
  environment for ROS builds and runs. The host Python/ROS pairing may differ.
- The host user is already listed in the `docker` group, but a Codex sandbox
  process may not inherit that group. If Docker socket access is denied, use
  the tool's approved elevated execution path; `sg docker -c '...'` works
  outside that sandbox. Do not change socket permissions, add users to groups,
  or make Docker accessible system-wide merely to work around the sandbox.
- Inspect container state before `docker exec`. A stopped container is not an
  error in a code-review task; start it only when the requested test needs it.
- `piper_press_hardware` was retired. The live hardware path is
  `piper_hardware` → `arm_controller` → `piper_elevator_manipulation`.
  Read `src/Piper_ros/src/piper_elevator_manipulation/docs/run_press_task.md`
  when executing or diagnosing the current press workflow.

## Choose the smallest meaningful test

1. For edits or diagnostics, inspect the relevant files and run focused
   static/unit checks first. Build only affected packages in the container.
2. Reuse recent, valid evidence. Do not repeat USB enumeration, CAN soak tests,
   joint-sign checks, or lengthy runbooks unless the connection, hardware,
   software boundary, or observed symptoms changed. A multi-minute/soak test
   needs a specific reliability question; it is not a default prerequisite.
3. For a simulation change, do one dry-run/plan check and the smallest relevant
   simulated execution. Expand only when a failure points to another layer.
4. For a real-arm motion request, use the current controller and documented
   enable/execute gates. Check only live conditions that can invalidate this
   run: the intended target and limits, fresh feedback, one active command
   writer/controller, and an available independent stop/clear workspace.
   Avoid re-asking settled setup questions if nothing has changed.
5. On contradictory controller state, stale CAN feedback, unexpected motion,
   or a failed stop, halt new commands and diagnose that fault before retrying.

The user has previously reported a fixed base, independent stop, and support
against arm drop. Treat that as useful prior context, **not** as a permanent
claim that the environment is absolutely safe. This skill does not authorize
powering or moving the arm from a request to inspect code, build, or plan.
Real motion requires authorization in the current task and the project's
non-executing defaults remain in force. Follow
[`piper-press-planning-control`](../piper-press-planning-control/SKILL.md)
for planning and contact semantics;
do not label fixed-travel completion as confirmed physical contact.
