#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoveIt2 「规划 -> 执行」演示脚本（配合 Gazebo 联合仿真）

作用：
    1. 调用 MoveIt2 的 /plan_kinematic_path 服务，让 OMPL 规划器规划一条
       从当前关节角到目标关节角的轨迹（自动做碰撞检测）；
    2. 把规划出的轨迹发给 arm_controller 执行；
    3. 打印执行前后的关节角，验证仿真中的机械臂确实动了。

运行前提（两个终端）：
    终端 A:  ros2 launch piper_gazebo piper_gazebo.launch.py
    终端 B:  ros2 launch piper_with_gripper_moveit piper_moveit.launch.py
    终端 C:  本脚本

用法：
    source /opt/ros/humble/setup.bash
    source ~/piper_ws/install/setup.bash
    python3 ~/piper_ws/examples/moveit_plan_demo.py
    # 自定义目标关节角（单位：弧度）
    python3 ~/piper_ws/examples/moveit_plan_demo.py --target 0.5 0.6 -0.7 0.2 0.4 0.3

注意：
    关节活动范围（来自 URDF）
        joint1: -2.6878 ~ 2.6878      joint4: -1.7802 ~ 1.7802
        joint2:  0.0     ~ 3.4034      joint5: -1.3090 ~ 1.3090
        joint3: -3.0543  ~ 0.0         joint6: -2.0944 ~ 2.0944
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from moveit_msgs.srv import GetMotionPlan

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
DEFAULT_TARGET = [0.5, 0.6, -0.7, 0.2, 0.4, 0.3]
JOINT_LIMITS = {           # (min, max) 弧度
    'joint1': (-2.6878, 2.6878),
    'joint2': (0.0, 3.4034),
    'joint3': (-3.0543, 0.0),
    'joint4': (-1.7802, 1.7802),
    'joint5': (-1.3090, 1.3090),
    'joint6': (-2.0944, 2.0944),
}


class PlanAndExecuteDemo(Node):
    def __init__(self):
        super().__init__('moveit_plan_demo')
        self.latest_joints = None
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.traj_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

    def _on_joint_state(self, msg: JointState):
        self.latest_joints = dict(zip(msg.name, msg.position))

    def wait_for_joint_states(self, timeout=10.0):
        deadline = time.time() + timeout
        while self.latest_joints is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_joints is not None

    def show_angles(self, title):
        if not self.latest_joints:
            print(f'  {title}: (未收到 /joint_states)')
            return
        vals = '  '.join(f'{j}={self.latest_joints.get(j, float("nan")):+.4f}'
                         for j in ARM_JOINTS)
        print(f'  {title}: {vals}')

    # ---------------------------------------------------------------- 规划
    def plan(self, target):
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            print('[X] /plan_kinematic_path 服务不可用，move_group 没启动？')
            return None

        req = GetMotionPlan.Request()
        mpr: MotionPlanRequest = req.motion_plan_request
        mpr.group_name = 'arm'                    # SRDF 里的规划组
        mpr.num_planning_attempts = 10
        mpr.allowed_planning_time = 5.0
        mpr.max_velocity_scaling_factor = 0.3     # 速度 30%
        mpr.max_acceleration_scaling_factor = 0.3

        cons = Constraints()
        for name, angle in zip(ARM_JOINTS, target):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(angle)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            cons.joint_constraints.append(jc)
        mpr.goal_constraints.append(cons)

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if not future.done() or future.result() is None:
            print('[X] 规划服务调用失败/超时')
            return None

        resp = future.result().motion_plan_response
        code = resp.error_code.val
        print(f'  规划结果 error_code = {code}  ({resp.error_code.val} == 1 表示成功)')
        if code != 1:
            print('[X] 规划失败，可能是目标不可达 / 超出关节限位 / 自碰撞')
            return None

        traj = resp.trajectory.joint_trajectory
        pts = len(traj.points)
        dur = 0.0
        if pts:
            last = traj.points[-1].time_from_start
            dur = last.sec + last.nanosec * 1e-9
        print(f'  规划成功：{pts} 个轨迹点，总时长 {dur:.2f} s，规划耗时 {resp.planning_time:.3f} s')
        return traj

    # ---------------------------------------------------------------- 执行
    def execute(self, traj):
        # 只保留机械臂的 6 个关节，并按 /arm_controller 期望的顺序发送
        idx = [traj.joint_names.index(j) for j in ARM_JOINTS]
        out = JointTrajectory()
        out.joint_names = list(ARM_JOINTS)
        for p in traj.points:
            from trajectory_msgs.msg import JointTrajectoryPoint
            np_ = JointTrajectoryPoint()
            np_.positions = [p.positions[i] for i in idx]
            np_.velocities = [p.velocities[i] for i in idx] if p.velocities else []
            np_.accelerations = [p.accelerations[i] for i in idx] if p.accelerations else []
            np_.time_from_start = p.time_from_start
            out.points.append(np_)

        # 等订阅者上线（Gazebo 的 controller 订阅该话题）
        deadline = time.time() + 5.0
        while self.traj_pub.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        print(f'  轨迹订阅者数量: {self.traj_pub.get_subscription_count()}')

        self.traj_pub.publish(out)
        print('  轨迹已下发，等待执行完成 ...')
        end = time.time() + 5.0 + (out.points[-1].time_from_start.sec if out.points else 0)
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)


def check_limits(target):
    for name, val in zip(ARM_JOINTS, target):
        lo, hi = JOINT_LIMITS[name]
        if not (lo <= val <= hi):
            print(f'[X] {name} = {val} 超出限位 [{lo}, {hi}]')
            return False
    return True


def main():
    ap = argparse.ArgumentParser(description='MoveIt2 规划 + 执行演示')
    ap.add_argument('--target', nargs=6, type=float, default=DEFAULT_TARGET,
                    help='目标关节角，6 个弧度值，顺序 joint1~joint6')
    args = ap.parse_args()

    if not check_limits(args.target):
        sys.exit(1)

    rclpy.init()
    node = PlanAndExecuteDemo()
    try:
        print('\n=== 1. 读取当前状态 ===')
        if not node.wait_for_joint_states():
            print('[X] 收不到 /joint_states，检查 Gazebo 是否在运行')
            return
        node.show_angles('当前关节角')

        print('\n=== 2. MoveIt2 规划 ===')
        print(f'  目标关节角: {args.target}')
        traj = node.plan(args.target)
        if traj is None:
            return

        print('\n=== 3. 执行轨迹 ===')
        node.execute(traj)

        print('\n=== 4. 结果 ===')
        node.show_angles('执行后关节角')
        errs = [abs(node.latest_joints.get(j, 0.0) - t)
                for j, t in zip(ARM_JOINTS, args.target)]
        worst = max(errs)
        print(f'  最大跟踪误差: {worst:.4f} rad')
        print('  ' + ('✓ 端到端验证通过：MoveIt2 规划 -> Gazebo 执行 成功'
                      if worst < 0.05 else '! 误差偏大，请检查控制器配置'))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
