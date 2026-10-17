#!/usr/bin/env python3

import rclpy

from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class ArmTucker(Node):

    def __init__(self):
        super().__init__('arm_tucker')

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.torso_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/torso_controller/follow_joint_trajectory'
        )

    def wait_for_controllers(self):

        self.get_logger().info(
            'Waiting for arm and torso trajectory controllers...'
        )

        while rclpy.ok():

            arm_ready = self.arm_client.wait_for_server(
                timeout_sec=1.0
            )

            torso_ready = self.torso_client.wait_for_server(
                timeout_sec=1.0
            )

            if arm_ready and torso_ready:
                self.get_logger().info(
                    'Arm and torso controllers are ready.'
                )
                return True

            self.get_logger().info(
                'Controllers not ready yet; waiting...'
            )

        return False

    @staticmethod
    def make_point(positions, seconds):

        point = JointTrajectoryPoint()
        point.positions = positions

        point.time_from_start = Duration(
            seconds=seconds
        ).to_msg()

        return point

    def build_arm_goal(self):

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            'arm_1_joint',
            'arm_2_joint',
            'arm_3_joint',
            'arm_4_joint',
            'arm_5_joint',
            'arm_6_joint',
            'arm_7_joint',
        ]

        # Official PAL "home" arm trajectory.
        goal.trajectory.points = [

            self.make_point(
                [
                    0.20,
                    0.35,
                    -0.20,
                    1.94,
                    -1.57,
                    1.37,
                    -1.58,
                ],
                0.5
            ),

            self.make_point(
                [
                    0.50,
                    -1.34,
                    -0.48,
                    1.94,
                    -1.49,
                    1.37,
                    -1.58,
                ],
                4.0
            ),

            self.make_point(
                [
                    0.50,
                    -1.34,
                    -0.48,
                    1.94,
                    -1.49,
                    1.37,
                    0.00,
                ],
                7.0
            ),
        ]

        return goal

    def build_torso_goal(self):

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            'torso_lift_joint'
        ]

        # Torso component of the same PAL "home" motion.
        goal.trajectory.points = [

            self.make_point(
                [0.25],
                0.5
            ),

            self.make_point(
                [0.18],
                4.0
            ),

            self.make_point(
                [0.15],
                7.0
            ),
        ]

        return goal

    def execute_home(self):

        if not self.wait_for_controllers():
            return False

        self.get_logger().info(
            'Tucking TIAGo arm to PAL home configuration...'
        )

        arm_future = self.arm_client.send_goal_async(
            self.build_arm_goal()
        )

        torso_future = self.torso_client.send_goal_async(
            self.build_torso_goal()
        )

        rclpy.spin_until_future_complete(
            self,
            arm_future
        )

        rclpy.spin_until_future_complete(
            self,
            torso_future
        )

        arm_handle = arm_future.result()
        torso_handle = torso_future.result()

        if arm_handle is None or not arm_handle.accepted:
            self.get_logger().error(
                'Arm trajectory goal was rejected.'
            )
            return False

        if torso_handle is None or not torso_handle.accepted:
            self.get_logger().error(
                'Torso trajectory goal was rejected.'
            )
            return False

        self.get_logger().info(
            'Arm and torso trajectory goals accepted.'
        )

        arm_result_future = (
            arm_handle.get_result_async()
        )

        torso_result_future = (
            torso_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            arm_result_future
        )

        rclpy.spin_until_future_complete(
            self,
            torso_result_future
        )

        arm_result = (
            arm_result_future.result().result
        )

        torso_result = (
            torso_result_future.result().result
        )

        arm_ok = (
            arm_result.error_code == 0
        )

        torso_ok = (
            torso_result.error_code == 0
        )

        if not arm_ok:
            self.get_logger().error(
                'Arm home trajectory failed: '
                f'code={arm_result.error_code}, '
                f'error="{arm_result.error_string}"'
            )

        if not torso_ok:
            self.get_logger().error(
                'Torso home trajectory failed: '
                f'code={torso_result.error_code}, '
                f'error="{torso_result.error_string}"'
            )

        if arm_ok and torso_ok:
            self.get_logger().info(
                'ARM TUCK SUCCESS: '
                'TIAGo is in PAL home configuration.'
            )
            return True

        return False


def main(args=None):

    rclpy.init(args=args)

    node = ArmTucker()

    try:
        success = node.execute_home()

        if not success:
            node.get_logger().error(
                'ARM TUCK FAILED.'
            )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
