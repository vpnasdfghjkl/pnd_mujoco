import math
import rclpy
import os
import time
import threading
from rclpy.node import Node
# from pndbotics_sdk_py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from adam_u.msg import LowCmd, LowState, MotorCmd, MotorState, HandCmd, HandState

# Kp 配置数组（对应19个关节）
KP_CONFIG = [
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

# Kd 配置数组（对应19个关节）
KD_CONFIG = [
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

class DemonController(Node):
    def __init__(self):
        super().__init__('demon_controller')
        self.lowcmd_pub_ = self.create_publisher(LowCmd, 'lowcmd', 10)
        self.handcmd_pub_ = self.create_publisher(HandCmd, 'handcmd', 10)
        
        self.dt = 0.0025 
        self.timer = self.create_timer(self.dt, self.Control)
        self.mutex = threading.Lock()
        
        self.motor_state = None

        self.lowstate_sub_ = self.create_subscription(
            LowState,
            "lowstate",
            self.getLowState,
            10)
            
        self.joint_num = 19  # 根据实际机器人配置修改
        self.open_arm_pos = [0, 0, 1.6,
                           0.0, -0.0,
            -1.6, 2.06, -1.65, -1.77,
                          0.32, 0, 0,
           - 1.6, -2.06, 1.65, -1.77,
                          0.32, 0, 0]

        self.close_arm_pos = [0, 0, 0,
                                 0, 0,
                           0, 0, 0, 0,
                              0, 0, 0,
                           0, 0, 0, 0,
                              0, 0, 0]
        
        self.open_hand = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
        self.close_hand = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.target_positions = [0.0] * self.joint_num
        self.runing_time = 0.0

    def Control(self):
        step_start = time.perf_counter()
 
        self.runing_time += self.dt
        cmd = LowCmd()
        handcmd = HandCmd()
        if (self.runing_time < 3600.0):
            # Stand up in first 3 second
            
            # Total time for standing up or standing down is about 1.2s
            phase = math.tanh(self.runing_time / 1.2)
            for i in range(19):
                cmd.motor_cmd[i].q = phase * self.open_arm_pos[i] + (
                    1 - phase) * self.close_arm_pos[i]
                # 使用配置的 Kp 和 Kd 值
                cmd.motor_cmd[i].kp = phase * KP_CONFIG[i] + (1 - phase) * 20.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = KD_CONFIG[i]
                cmd.motor_cmd[i].tau = 0.0
            for i in range(12):
                handcmd.position[i] = int(phase * self.close_hand[i] + (
                    1 - phase) * self.open_hand[i])
        else:
            # Then stand down
            phase = math.tanh((self.runing_time - 3.0) / 1.2)
            for i in range(19):
                cmd.motor_cmd[i].q = phase * self.close_arm_pos[i] + (
                    1 - phase) * self.open_arm_pos[i]
                # 使用配置的 Kp 和 Kd 值
                cmd.motor_cmd[i].kp = KP_CONFIG[i]
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = KD_CONFIG[i]
                cmd.motor_cmd[i].tau = 0.0
            for i in range(12):
                handcmd.position[i] = int(phase * self.open_hand[i] + (
                    1 - phase) * self.close_hand[i])
        self.lowcmd_pub_.publish(cmd)
        self.handcmd_pub_.publish(handcmd)

        time_until_next_step = self.dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

    def getLowState(self, msg):
        with self.mutex:
            self.motor_state = msg
            # print("received lowstate")
            
def main(args=None):
    # os.environ["ROS_DOMAIN_ID"] = "2"
    # os.environ["ROS_LOCALHOST"] = "127.0.0.1"
    rclpy.init(args=args)
    demon_controller = DemonController()
    rclpy.spin(demon_controller)
    demon_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    input("Press enter to start")
    main()
