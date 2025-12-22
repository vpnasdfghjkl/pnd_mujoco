from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from adam_u.msg import LowCmd, HandCmd

from vr_jointstate_utils import VrToCmdConfig, extract_adam_u_targets, name_position_to_map


# Reuse the same PD defaults as ros2_open_arm.py (you can tune later).
KP_CONFIG: list[float] = [
    160.0,  # waistRoll (0)
    160.0,  # waistPitch (1)
    160.0,  # waistYaw (2)
    19.0,   # neckYaw (3)
    19.0,   # neckPitch (4)
    118.0,  # shoulderPitch_Left (5)
    19.0,   # shoulderRoll_Left (6)
    19.0,   # shoulderYaw_Left (7)
    19.0,   # elbow_Left (8)
    19.0,   # wristYaw_Left (9)
    19.0,   # wristPitch_Left (10)
    19.0,   # wristRoll_Left (11)
    118.0,  # shoulderPitch_Right (12)
    19.0,   # shoulderRoll_Right (13)
    19.0,   # shoulderYaw_Right (14)
    19.0,   # elbow_Right (15)
    19.0,   # wristYaw_Right (16)
    19.0,   # wristPitch_Right (17)
    19.0    # wristRoll_Right (18)
]

KD_CONFIG: list[float] = [
    1.0,   # waistRoll (0)
    1.0,   # waistPitch (1)
    1.0,   # waistYaw (2)
    0.9,   # neckYaw (3)
    0.9,   # neckPitch (4)
    0.9,   # shoulderPitch_Left (5)
    0.9,   # shoulderRoll_Left (6)
    0.9,   # shoulderYaw_Left (7)
    0.9,   # elbow_Left (8)
    0.9,   # wristYaw_Left (9)
    0.9,   # wristPitch_Left (10)
    0.9,   # wristRoll_Left (11)
    0.9,   # shoulderPitch_Right (12)
    0.9,   # shoulderRoll_Right (13)
    0.9,   # shoulderYaw_Right (14)
    0.9,   # elbow_Right (15)
    0.9,   # wristYaw_Right (16)
    0.9,   # wristPitch_Right (17)
    0.9    # wristRoll_Right (18)
]


class VrJointStateBridge(Node):
    """Subscribe VR /joint_states and publish adam_u lowcmd/handcmd.

    Design:
      - conversion is centralized in vr_jointstate_utils.py (reusable on VR sender)
      - publish-on-receive (simple, low latency)

    Expected JointState names include (subset OK):
      - dof_pos/waistRoll, ..., dof_pos/wristRoll_Right, dof_pos/neckYaw, dof_pos/neckPitch
      - dof_pos/hand_* (12 entries) for hands
    """

    def __init__(self):
        super().__init__("vr_jointstate_to_cmd")

        self._mutex = threading.Lock()
        self._last_name_to_pos: Optional[dict[str, float]] = None

        self._cfg = VrToCmdConfig(
            # If your VR hand values are reversed, swap these two.
            hand_open_angle=0.0,
            hand_close_angle=1.0,
            hand_open_value=1000,
            hand_close_value=0,
        )

        self._pub_lowcmd = self.create_publisher(LowCmd, "lowcmd", 10)
        self._pub_handcmd = self.create_publisher(HandCmd, "handcmd", 10)
        self._sub_js = self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        self.get_logger().info("Subscribed to /joint_states; publishing lowcmd & handcmd")

    def _on_js(self, msg: JointState) -> None:
        name_to_pos = name_position_to_map(msg.name, msg.position)
        with self._mutex:
            self._last_name_to_pos = name_to_pos

        q_targets, hand_targets = extract_adam_u_targets(name_to_pos, cfg=self._cfg)

        lowcmd = LowCmd()
        for i in range(19):
            lowcmd.motor_cmd[i].q = float(q_targets[i])
            lowcmd.motor_cmd[i].dq = 0.0
            lowcmd.motor_cmd[i].kp = float(KP_CONFIG[i])
            lowcmd.motor_cmd[i].kd = float(KD_CONFIG[i])
            lowcmd.motor_cmd[i].tau = 0.0
        self._pub_lowcmd.publish(lowcmd)

        if hand_targets is not None:
            handcmd = HandCmd()
            for i in range(12):
                handcmd.position[i] = int(hand_targets[i])
            self._pub_handcmd.publish(handcmd)


def main() -> None:
    rclpy.init(args=None)
    node = VrJointStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
