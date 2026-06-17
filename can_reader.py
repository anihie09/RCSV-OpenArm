"""
OpenArm 2.0 - CAN data reader (Task 2)

This includes joint position, velocity and torqur from the arm over CAN FD 
using the OpenArm CAN library using the pattern at docs.openarm.dev/api-reference/can

HARDWARE NOTES :
I do not have physical access to OpenArm 2.0 hardware or a CAN FD-capable
interface, so live SocketCAN frames cannot be captured for this submission.
This module has LiveCANReader and MockCANReader.

Both readers share the JointState dataclass and CANReader so synch and storage doesn't know
data comes from the hardware or simulation. If the hardware is there, switching MockCANReader for 
LiveCANReader in main.py is the only change needed.

"""

from __future__ import annotations

import math
import time
import threading
from dataclasses import dataclass, field
from typing import Callable, Optional

# OpenArm 2.0: 7 DOF per arm
NUM_JOINTS = 7 


@dataclass
class JointState:
    """A single timestamped snapshot of joint feedback."""
    timestamp: float                  # monotonic seconds, host clock
    position: list[float] = field(default_factory=list)   # rad, len=NUM_JOINTS
    velocity: list[float] = field(default_factory=list)   # rad/s
    torque: list[float] = field(default_factory=list)     # N*m
    arm_id: str = "can0"               # which bus / arm this came from


class CANReader:
    """Common interface implemented by both Live and Mock readers."""

    def start(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def read_joint_state(self) -> Optional[JointState]:
        """Return the most recent JointState, or None if not yet available."""
        raise NotImplementedError

    def register_callback(self, cb: Callable[[JointState], None]) -> None:
        """Register a callback invoked on every new JointState (push model)."""
        raise NotImplementedError


class LiveCANReader(CANReader):
    """
    Real CAN FD reader using the OpenArm CAN library's python bindings.

    Written against the documented API surface:
      - openarm.can.socket.OpenArm("can0", True)   # True = CAN FD
      - openarm.init_arm_motors(motor_types, send_ids, recv_ids)
      - openarm.set_callback_mode_all(CallbackMode.STATE)
      - openarm.refresh_all(); openarm.recv_all(timeout_us)
      - motor.get_position() / get_velocity() / get_torque()

    This is used for next steps but not run because there is no hardware or package installed.
    
    """

    def __init__(self, interface: str = "can0", poll_hz: float = 1000.0):
        self.interface = interface
        self.poll_period = 1.0 / poll_hz
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._latest: Optional[JointState] = None
        self._callbacks: list[Callable[[JointState], None]] = []
        self._openarm = None  # would hold openarm_can.OpenArm instance

    def start(self) -> None:
        try:
            from openarm_can import (  # type: ignore
                OpenArm, MotorType, CallbackMode,
            )
        except ImportError as e:
            raise RuntimeError(
                "openarm_can python bindings not available in this "
                "environment. Install per docs.openarm.dev/tutorial/setup, "
                "or use MockCANReader for development/testing."
            ) from e

        self._openarm = OpenArm(self.interface, True)  # CAN FD = True
        motor_types = [MotorType.DM4310] * NUM_JOINTS
        send_ids = list(range(0x01, 0x01 + NUM_JOINTS))
        recv_ids = list(range(0x11, 0x11 + NUM_JOINTS))
        self._openarm.init_arm_motors(motor_types, send_ids, recv_ids)
        self._openarm.set_callback_mode_all(CallbackMode.STATE)
        self._openarm.enable_all()
        self._openarm.recv_all(2000)

        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._openarm:
            self._openarm.disable_all()
            self._openarm.recv_all(1000)

    def _poll_loop(self) -> None:
        while self._running:
            self._openarm.refresh_all()
            self._openarm.recv_all(300)  # 300us, per docs guidance for fast ops

            motors = self._openarm.get_arm().get_motors()
            state = JointState(
                timestamp=time.monotonic(),
                position=[m.get_position() for m in motors],
                velocity=[m.get_velocity() for m in motors],
                torque=[m.get_torque() for m in motors],
                arm_id=self.interface,
            )
            self._latest = state
            for cb in self._callbacks:
                cb(state)

            time.sleep(self.poll_period)

    def read_joint_state(self) -> Optional[JointState]:
        return self._latest

    def register_callback(self, cb: Callable[[JointState], None]) -> None:
        self._callbacks.append(cb)


class MockCANReader(CANReader):
    """
    Software simulator standing in for `LiveCANReader`.

    """

    def __init__(self, arm_id: str = "can0", poll_hz: float = 1000.0, seed: int = 0):
        self.arm_id = arm_id
        self.poll_period = 1.0 / poll_hz
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._latest: Optional[JointState] = None
        self._callbacks: list[Callable[[JointState], None]] = []
        self._t0 = time.monotonic()
        # Per-joint amplitude/frequency/phase so each joint moves differently
        self._amp = [0.3 + 0.05 * i for i in range(NUM_JOINTS)]
        self._freq = [0.15 + 0.03 * i for i in range(NUM_JOINTS)]
        self._phase = [0.2 * i + seed for i in range(NUM_JOINTS)]

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _poll_loop(self) -> None:
        while self._running:
            t = time.monotonic() - self._t0
            pos = [
                self._amp[i] * math.sin(2 * math.pi * self._freq[i] * t + self._phase[i])
                for i in range(NUM_JOINTS)
            ]
            vel = [
                self._amp[i] * 2 * math.pi * self._freq[i] *
                math.cos(2 * math.pi * self._freq[i] * t + self._phase[i])
                for i in range(NUM_JOINTS)
            ]
            # Rough torque model: proportional to position (gravity-like load) + small damping term
            torque = [0.8 * pos[i] + 0.05 * vel[i] for i in range(NUM_JOINTS)]

            state = JointState(
                timestamp=time.monotonic(),
                position=pos,
                velocity=vel,
                torque=torque,
                arm_id=self.arm_id,
            )
            self._latest = state
            for cb in self._callbacks:
                cb(state)

            time.sleep(self.poll_period)

    def read_joint_state(self) -> Optional[JointState]:
        return self._latest

    def register_callback(self, cb: Callable[[JointState], None]) -> None:
        self._callbacks.append(cb)


if __name__ == "__main__":
    # Quick smoke test: print joint states for ~2 seconds
    reader = MockCANReader(arm_id="can0", poll_hz=100.0)
    reader.start()
    try:
        for _ in range(20):
            time.sleep(0.1)
            s = reader.read_joint_state()
            if s:
                print(
                    f"t={s.timestamp:.3f} "
                    f"pos[0]={s.position[0]:+.3f} "
                    f"vel[0]={s.velocity[0]:+.3f} "
                    f"tau[0]={s.torque[0]:+.3f}"
                )
    finally:
        reader.stop()