from __future__ import annotations

from typing import Any, Optional
import os
import sys
import time
import threading
import traceback

import config

import mujoco
import numpy as np

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
except Exception:  # pragma: no cover
    rclpy = None
    SingleThreadedExecutor = None  # type: ignore
    Node = object  # type: ignore


TOPIC_ROS2_LOWCMD = "lowcmd"
TOPIC_ROS2_HANDCMD = "handcmd"
TOPIC_ROS2_LOWSTATE = "lowstate"

NUM_MOTOR_ADAM_U = 19


def _import_ros2_msgs(candidates: tuple[str, ...]):
    """Import ROS2 message classes at runtime.

    We avoid import-time hard dependency so this file can be imported even when the
    ROS2 workspace isn't sourced.
    """
    import importlib

    last_exc: Optional[BaseException] = None
    for mod_name in candidates:
        try:
            msg_module = importlib.import_module(mod_name)
            LowCmd = getattr(msg_module, "LowCmd")
            HandCmd = getattr(msg_module, "HandCmd")
            LowState = getattr(msg_module, "LowState")
            return msg_module.__name__, LowCmd, HandCmd, LowState
        except Exception as exc:  # pragma: no cover
            last_exc = exc

    raise ImportError(
        "Cannot import ROS2 message module. Tried: "
        + ", ".join(candidates)
        + (f". Last error: {last_exc}" if last_exc is not None else "")
    )


def _get_sim_lock():
    """Return the global simulation lock from the running sim script, if any."""
    try:
        import sys

        main_mod = sys.modules.get("__main__")
        if main_mod is not None and hasattr(main_mod, "locker"):
            return getattr(main_mod, "locker")
    except Exception:
        pass
    return None


class RealRosBridge:
    """MuJoCo <-> ROS2 bridge using rclpy topics.

    - Subscribes: `lowcmd`, `handcmd`
    - Publishes: `lowstate`

    This is a "real ROS" bridge: it does not intercept vendor DDS IDL types.
    """

    def __init__(self, mj_model: mujoco.MjModel, mj_data: mujoco.MjData):
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.dt = self.mj_model.opt.timestep

        self.num_motor = NUM_MOTOR_ADAM_U

        self._mutex = threading.Lock()
        self._latest_lowcmd: Optional[Any] = None
        self._latest_handcmd: Optional[Any] = None

        # Use Any here to avoid static-type issues when rclpy isn't available.
        self._ros2_node: Optional[Any] = None
        self._ros2_executor: Optional[Any] = None
        self._ros2_spin_thread: Optional[threading.Thread] = None

        self._msg_module_name: Optional[str] = None
        self._RosLowCmd = None
        self._RosHandCmd = None
        self._RosLowState = None

        self._init_ros2()

    def _init_ros2(self) -> None:
        if rclpy is None:
            print("[RealRosBridge] rclpy not available in this Python environment")
            return

        candidates = ("adam_u.msg", "adam_u_msgs.msg")
        try:
            self._msg_module_name, self._RosLowCmd, self._RosHandCmd, self._RosLowState = (
                _import_ros2_msgs(candidates)
            )
        except Exception as exc:
            self._print_import_diagnostics(candidates, exc)
            return

        if not rclpy.ok():
            rclpy.init(args=None)

        outer = self

        class _BridgeNode(Node):
            def __init__(self):
                super().__init__("pnd_mujoco_real_ros_bridge")

                self._sub_lowcmd = self.create_subscription(
                    outer._RosLowCmd, TOPIC_ROS2_LOWCMD, self._on_lowcmd, 10
                )
                self._sub_handcmd = self.create_subscription(
                    outer._RosHandCmd, TOPIC_ROS2_HANDCMD, self._on_handcmd, 10
                )
                self._pub_lowstate = self.create_publisher(
                    outer._RosLowState, TOPIC_ROS2_LOWSTATE, 10
                )

            def _on_lowcmd(self, msg: Any) -> None:
                with outer._mutex:
                    outer._latest_lowcmd = msg

            def _on_handcmd(self, msg: Any) -> None:
                with outer._mutex:
                    outer._latest_handcmd = msg

            def publish_lowstate(self, msg: Any) -> None:
                self._pub_lowstate.publish(msg)

        self._ros2_node = _BridgeNode()
        self._ros2_executor = SingleThreadedExecutor()
        self._ros2_executor.add_node(self._ros2_node)

        def _spin() -> None:
            try:
                self._ros2_executor.spin()  # type: ignore[union-attr]
            except Exception:
                traceback.print_exc()

        self._ros2_spin_thread = threading.Thread(target=_spin, daemon=True)
        self._ros2_spin_thread.start()

        print(
            "[RealRosBridge] Using msg module: "
            + str(self._msg_module_name)
            + "; subscribing topics: lowcmd, handcmd"
        )

    def _print_import_diagnostics(self, candidates: tuple[str, ...], exc: BaseException) -> None:
        print("[RealRosBridge] ROS2 message types not importable")
        print("  Tried modules:", ", ".join(candidates))
        print("  Exception:", repr(exc))
        print("  Python:", sys.executable)
        print("  CWD:", os.getcwd())
        print("  ROS_DISTRO:", os.environ.get("ROS_DISTRO"))
        print("  ROS_DOMAIN_ID:", os.environ.get("ROS_DOMAIN_ID"))
        print("  AMENT_PREFIX_PATH:", os.environ.get("AMENT_PREFIX_PATH"))
        print("  PYTHONPATH:", os.environ.get("PYTHONPATH"))
        print("  sys.path[0:5]:")
        for p in sys.path[:5]:
            print("   -", p)
        print(
            "  Hint: run *in the same shell* before starting the sim:\n"
            "    . /home/hanxiao/camille/code/pnd_mujoco/example/ros2/install/setup.zsh"
        )

    def ApplyLatestCommands(self, assume_sim_locked: bool = False) -> None:
        with self._mutex:
            lowcmd = self._latest_lowcmd
            handcmd = self._latest_handcmd

        if self.mj_data is None:
            return

        locker = None
        if not assume_sim_locked:
            locker = _get_sim_lock()
            if locker is not None:
                locker.acquire()
        try:
            if lowcmd is not None:
                motor_cmd = getattr(lowcmd, "motor_cmd", None)
                if motor_cmd is not None:
                    apply_len = min(self.num_motor, len(motor_cmd))
                    for i in range(apply_len):
                        m = motor_cmd[i]
                        self.mj_data.ctrl[i] = (
                            float(getattr(m, "tau", 0.0))
                            + float(getattr(m, "kp", 0.0))
                            * (float(getattr(m, "q", 0.0)) - self.mj_data.sensordata[i])
                            + float(getattr(m, "kd", 0.0))
                            * (
                                float(getattr(m, "dq", 0.0))
                                - self.mj_data.sensordata[i + self.num_motor]
                            )
                        )

            if handcmd is not None:
                position = getattr(handcmd, "position", None)
                if position is not None:
                    fingers_pos = list(position)[0:12]
                    fingers = [finger for finger in fingers_pos for _ in range(2)]

                    if config.HANDPOSE_SRC == 0:
                        for i in range(self.num_motor, self.num_motor + 24):
                            self.mj_data.ctrl[i] = fingers[i - self.num_motor]
                    else:
                        fingers[10] = fingers[8] * 0.5
                        fingers[8] = 2 * fingers[10]
                        fingers[9] = 2 * fingers[10]
                        fingers[22] = fingers[20] * 0.5
                        fingers[20] = 2 * fingers[22]
                        fingers[21] = 2 * fingers[22]

                        for i in range(self.num_motor, self.num_motor + 24):
                            self.mj_data.ctrl[i] = 1.6 - fingers[i - self.num_motor] * 0.0016

                            if i == self.num_motor + 10:
                                self.mj_data.ctrl[i] = 0.5 - fingers[i - self.num_motor] * 0.001

                            if i in (
                                self.num_motor + 11,
                                self.num_motor + 9,
                                self.num_motor + 8,
                            ):
                                self.mj_data.ctrl[i] = 1.0 - fingers[i - self.num_motor] * 0.001

                            if i == self.num_motor + 22:
                                self.mj_data.ctrl[i] = 0.5 - fingers[i - self.num_motor] * 0.001

                            if i in (
                                self.num_motor + 23,
                                self.num_motor + 21,
                                self.num_motor + 20,
                            ):
                                self.mj_data.ctrl[i] = 1.0 - fingers[i - self.num_motor] * 0.001
        finally:
            if locker is not None:
                locker.release()

    def PublishLowState(self, assume_sim_locked: bool = False) -> None:
        if self._ros2_node is None:
            return

        if self._RosLowState is None:
            return

        if self.mj_data is None:
            return

        locker = None
        if not assume_sim_locked:
            locker = _get_sim_lock()
            if locker is not None:
                locker.acquire()
        try:
            msg = self._RosLowState()
            motor_state = getattr(msg, "motor_state", None)
            if motor_state is not None and len(motor_state) >= self.num_motor:
                for i in range(self.num_motor):
                    motor_state[i].q = float(self.mj_data.sensordata[i])
                    motor_state[i].dq = float(self.mj_data.sensordata[i + self.num_motor])
                    motor_state[i].tau_est = float(
                        self.mj_data.sensordata[i + 2 * self.num_motor]
                    )

            self._ros2_node.publish_lowstate(msg)
        finally:
            if locker is not None:
                locker.release()


class ElasticBand:
    def __init__(self):
        self.stiffness = 200
        self.damping = 100
        self.point = np.array([0, 0, 3])
        self.length = 0
        self.enable = True

    def Advance(self, x, dx):
        """Virtual spring band.

        Args:
          δx: desired position - current position
          dx: current velocity
        """
        δx = self.point - x
        distance = np.linalg.norm(δx)
        direction = δx / distance
        v = np.dot(dx, direction)
        f = (self.stiffness * (distance - self.length) - self.damping * v) * direction
        return f


if __name__ == "__main__":
    # test lowcmd and handcmd
    # subscribe lowcmd and handcmd
    # publish lowstate
    # _sub_handcmd = Node.create_subscription(
    #                 RosHandCmd, TOPIC_ROS2_HANDCMD, self._on_handcmd, 10
    #             )
    print("This module is intended to be imported by the simulator.")
