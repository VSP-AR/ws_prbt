
import time
import yaml
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from ament_index_python.packages import get_package_share_directory

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class PRBTJointLoop(Node):
    def __init__(self):
        super().__init__("prbt_joint_loop")

     
        self.declare_parameter("poses_yaml", "config/joint_poses.yaml")
        self.declare_parameter("action_name", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("move_time", 4.0)
        self.declare_parameter("hold_time", 1.0)
        self.declare_parameter("ping_pong", False)

        poses_yaml_rel = self.get_parameter("poses_yaml").value
        self.action_name = self.get_parameter("action_name").value
        self.move_time = float(self.get_parameter("move_time").value)
        self.hold_time = float(self.get_parameter("hold_time").value)
        self.ping_pong = bool(self.get_parameter("ping_pong").value)

        pkg_share = get_package_share_directory("prbt_cell_bringup")
        poses_yaml_path = f"{pkg_share}/{poses_yaml_rel}"

        with open(poses_yaml_path, "r") as f:
            data = yaml.safe_load(f)

        self.joint_names = data["joint_names"]
        self.poses = data["poses"]

        self.client = ActionClient(self, FollowJointTrajectory, self.action_name)

        self.get_logger().info(f"Waiting for action server: {self.action_name}")
        if not self.client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("No FollowJointTrajectory action server found. Check controller name.")
            raise RuntimeError("Trajectory action server not available")

        self.get_logger().info(f"Loaded {len(self.poses)} poses from {poses_yaml_path}")

        self.timer = self.create_timer(1.0, self._start_once)
        self.started = False

    def _start_once(self):
        if self.started:
            return
        self.started = True
        self.timer.cancel()

       
        threading.Thread(target=self.run_loop, daemon=True).start()


    def _duration(self, seconds: float) -> Duration:
        d = Duration()
        d.sec = int(seconds)
        d.nanosec = int((seconds - int(seconds)) * 1e9)
        return d

    def send_pose(self, positions):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start = self._duration(self.move_time)
        goal.trajectory.points = [pt]

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("Goal finished.")

        return True

    def run_loop(self):
        order = list(range(len(self.poses)))

        while rclpy.ok():
            seq = order
            if self.ping_pong and len(order) > 2:
                seq = order + order[-2:0:-1]

            for idx in seq:
                pose = self.poses[idx]
                self.get_logger().info(f"Going to: {pose['name']}")
                ok = self.send_pose(pose["positions"])
                if not ok:
                    self.get_logger().warn("Failed to send pose. Retrying in 1s...")
                    time.sleep(1.0)
                    continue
                time.sleep(self.hold_time)


def main():
    rclpy.init()
    node = PRBTJointLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
