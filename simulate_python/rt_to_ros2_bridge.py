from __future__ import annotations

import argparse
import threading
import time
from typing import Any, Optional

from pndbotics_sdk_py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
)

# Vendor IDL (DDS) types
from pndbotics_sdk_py.idl.adam_u.msg.dds_ import LowCmd_ as IdlLowCmd
from pndbotics_sdk_py.idl.adam_u.msg.dds_ import LowState_ as IdlLowState
from pndbotics_sdk_py.idl.adam_u.msg.dds_ import HandCmd_ as IdlHandCmd


TOPIC_RT_LOWCMD = "rt/lowcmd"
TOPIC_RT_LOWSTATE = "rt/lowstate"
TOPIC_RT_HANDCMD = "rt/handcmd"


def _import_ros2_msgs():
    import importlib

    candidates = ("adam_u.msg", "adam_u_msgs.msg")
    last_exc: Optional[BaseException] = None
    for mod_name in candidates:
        try:
            mod = importlib.import_module(mod_name)
            return (
                mod.__name__,
                getattr(mod, "LowCmd"),
                getattr(mod, "LowState"),
                getattr(mod, "HandCmd", None),
            )
        except Exception as exc:
            last_exc = exc

    raise ImportError(
        "Cannot import ROS2 message module. Tried: "
        + ", ".join(candidates)
        + (f". Last error: {last_exc}" if last_exc is not None else "")
        + "\n\nHints:\n"
        + "- Ensure ROS2 is sourced in this shell (e.g. `source /opt/ros/humble/setup.bash`).\n"
        + "- Ensure your custom msg package overlay is sourced (e.g. `source ros2/install/setup.bash`).\n"
        + "- If you are inside a virtualenv, make sure it can see the ROS2 site-packages and your msg install."
    )


def _copy_motor_cmd(dst: Any, src: Any) -> None:
    # Fields differ slightly across msg definitions; copy what exists.
    for name in ("mode", "q", "dq", "tau", "kp", "kd", "reserve"):
        if hasattr(dst, name) and hasattr(src, name):
            setattr(dst, name, getattr(src, name))


def _copy_motor_state(dst: Any, src: Any) -> None:
    for name in ("mode", "q", "dq", "tau_est", "motorstate", "reserve"):
        if hasattr(dst, name) and hasattr(src, name):
            setattr(dst, name, getattr(src, name))


def _copy_hand_cmd(dst: Any, src: Any) -> None:
    # Most definitions use `position` (+ optional `reserve`).
    if hasattr(dst, "position") and hasattr(src, "position"):
        try:
            dst.position = list(src.position)
        except Exception:
            pass

    if hasattr(dst, "reserve") and hasattr(src, "reserve"):
        dst.reserve = src.reserve


class RtToRos2Bridge:
    """Forward vendor rt topics (DDS/IDL) to ROS2 topics for data collection.

    - Subscribes (vendor): rt/lowcmd, rt/lowstate
    - Publishes (ROS2):   rt/lowcmd, rt/lowstate  (as adam_u/LowCmd, adam_u/LowState)

    This is intended for rosbag/analysis. It does NOT control the robot.
    """

    def __init__(
        self,
        *,
        num_motor: int,
        ros2_topic_lowcmd: str,
        ros2_topic_lowstate: str,
        ros2_topic_handcmd: str,
        stats_period_s: float,
    ) -> None:
        self._num_motor = int(num_motor)
        self._ros2_topic_lowcmd = ros2_topic_lowcmd
        self._ros2_topic_lowstate = ros2_topic_lowstate
        self._ros2_topic_handcmd = ros2_topic_handcmd

        mod_name, RosLowCmd, RosLowState, RosHandCmd = _import_ros2_msgs()
        self._RosLowCmd = RosLowCmd
        self._RosLowState = RosLowState
        self._RosHandCmd = RosHandCmd

        import rclpy
        from rclpy.node import Node

        if not rclpy.ok():
            rclpy.init(args=None)

        self._rclpy = rclpy
        self._node = Node("rt_to_ros2_bridge")
        self._pub_lowcmd = self._node.create_publisher(RosLowCmd, ros2_topic_lowcmd, 10)
        self._pub_lowstate = self._node.create_publisher(RosLowState, ros2_topic_lowstate, 10)

        self._pub_handcmd = None
        if RosHandCmd is not None:
            self._pub_handcmd = self._node.create_publisher(RosHandCmd, ros2_topic_handcmd, 10)
        else:
            self._node.get_logger().warn(
                f"ROS2 msg module {mod_name} has no HandCmd; rt/handcmd bridging disabled"
            )

        self._lock = threading.Lock()

        self._closing = False

        self._rx_lowcmd = 0
        self._rx_lowstate = 0
        self._rx_handcmd = 0
        self._last_lowcmd_t = 0.0
        self._last_lowstate_t = 0.0
        self._last_handcmd_t = 0.0
        self._t0 = time.monotonic()

        self._node.get_logger().info(
            "Using msg module: "
            f"{mod_name}; publishing {ros2_topic_lowcmd}, {ros2_topic_lowstate}"
            + (f", {ros2_topic_handcmd}" if self._pub_handcmd is not None else "")
        )

        if stats_period_s and stats_period_s > 0:
            self._node.create_timer(float(stats_period_s), self._log_stats)

        # Vendor DDS/IDL subscriptions
        self._sub_lowcmd = ChannelSubscriber(TOPIC_RT_LOWCMD, IdlLowCmd)
        self._sub_lowstate = ChannelSubscriber(TOPIC_RT_LOWSTATE, IdlLowState)
        self._sub_handcmd = ChannelSubscriber(TOPIC_RT_HANDCMD, IdlHandCmd)
        self._sub_lowcmd.Init(handler=self._on_lowcmd, queueLen=50)
        self._sub_lowstate.Init(handler=self._on_lowstate, queueLen=50)
        self._sub_handcmd.Init(handler=self._on_handcmd, queueLen=50)

    def _log_stats(self) -> None:
        now = time.monotonic()
        with self._lock:
            rx_lowcmd = self._rx_lowcmd
            rx_lowstate = self._rx_lowstate
            rx_handcmd = self._rx_handcmd
            last_lowcmd_t = self._last_lowcmd_t
            last_lowstate_t = self._last_lowstate_t
            last_handcmd_t = self._last_handcmd_t

        def age_str(last_t: float) -> str:
            if last_t <= 0:
                return "never"
            return f"{now - last_t:.3f}s ago"

        self._node.get_logger().info(
            "RX stats: "
            f"lowcmd={rx_lowcmd} (last {age_str(last_lowcmd_t)}), "
            f"lowstate={rx_lowstate} (last {age_str(last_lowstate_t)}), "
            f"handcmd={rx_handcmd} (last {age_str(last_handcmd_t)})"
        )

    def _on_lowcmd(self, msg: IdlLowCmd) -> None:
        if self._closing:
            return
        ros_msg = self._RosLowCmd()
        src_arr = getattr(msg, "motor_cmd", None)
        dst_arr = getattr(ros_msg, "motor_cmd", None)
        if src_arr is not None and dst_arr is not None:
            n = min(self._num_motor, len(src_arr), len(dst_arr))
            for i in range(n):
                _copy_motor_cmd(dst_arr[i], src_arr[i])

        if hasattr(ros_msg, "reserve") and hasattr(msg, "reserve"):
            ros_msg.reserve = msg.reserve

        with self._lock:
            self._rx_lowcmd += 1
            self._last_lowcmd_t = time.monotonic()

        if not self._closing:
            self._pub_lowcmd.publish(ros_msg)
            print("published lowcmd", self._rx_lowcmd)

    def _on_lowstate(self, msg: IdlLowState) -> None:
        if self._closing:
            return
        ros_msg = self._RosLowState()
        src_arr = getattr(msg, "motor_state", None)
        dst_arr = getattr(ros_msg, "motor_state", None)
        if src_arr is not None and dst_arr is not None:
            n = min(self._num_motor, len(src_arr), len(dst_arr))
            for i in range(n):
                _copy_motor_state(dst_arr[i], src_arr[i])

        if hasattr(ros_msg, "reserve") and hasattr(msg, "reserve"):
            ros_msg.reserve = msg.reserve

        with self._lock:
            self._rx_lowstate += 1
            self._last_lowstate_t = time.monotonic()

        if not self._closing:
            self._pub_lowstate.publish(ros_msg)

    def _on_handcmd(self, msg: IdlHandCmd) -> None:
        if self._closing:
            return

        if self._pub_handcmd is None or self._RosHandCmd is None:
            return

        ros_msg = self._RosHandCmd()
        _copy_hand_cmd(ros_msg, msg)

        with self._lock:
            self._rx_handcmd += 1
            self._last_handcmd_t = time.monotonic()

        if not self._closing:
            self._pub_handcmd.publish(ros_msg)

    def spin(self) -> None:
        try:
            self._rclpy.spin(self._node)
        except KeyboardInterrupt:
            pass
        finally:
            # Stop DDS callbacks before tearing down ROS2.
            self._closing = True
            try:
                self._sub_lowcmd.Close()
            except Exception:
                pass
            try:
                self._sub_lowstate.Close()
            except Exception:
                pass
            try:
                self._sub_handcmd.Close()
            except Exception:
                pass

            try:
                self._node.destroy_node()
            finally:
                try:
                    self._rclpy.shutdown()
                except Exception:
                    pass


def main() -> None:
    parser = argparse.ArgumentParser(description="Forward rt/lowcmd & rt/lowstate to ROS2 topics")
    parser.add_argument("--domain-id", type=int, default=2, help="DDS domain id")
    parser.add_argument("--iface", type=str, default="lo", help="DDS interface, e.g. lo/eth0")
    parser.add_argument("--num-motor", type=int, default=19, help="motor count (adam_u=19)")
    parser.add_argument(
        "--stats-period",
        type=float,
        default=2.0,
        help="Print RX stats every N seconds (0 to disable)",
    )
    parser.add_argument(
        "--ros2-lowcmd",
        type=str,
        default="rt/lowcmd",
        help="ROS2 topic name for LowCmd",
    )
    parser.add_argument(
        "--ros2-lowstate",
        type=str,
        default="rt/lowstate",
        help="ROS2 topic name for LowState",
    )
    parser.add_argument(
        "--ros2-handcmd",
        type=str,
        default="rt/handcmd",
        help="ROS2 topic name for HandCmd",
    )
    args = parser.parse_args()

    # Initialize vendor channel stack
    ChannelFactoryInitialize(args.domain_id, args.iface)

    bridge = RtToRos2Bridge(
        num_motor=args.num_motor,
        ros2_topic_lowcmd=args.ros2_lowcmd,
        ros2_topic_lowstate=args.ros2_lowstate,
        ros2_topic_handcmd=args.ros2_handcmd,
        stats_period_s=args.stats_period,
    )
    bridge.spin()


if __name__ == "__main__":
    main()
