"""Kinematic-holonomic drive rig control + odometry for the Ridgeback.

The imported robot USD (tools/isaac/import_ridgeback_urdf.py) anchors
base_link to the world through three velocity-driven virtual joints:
px (prismatic X), py (prismatic Y), rz (revolute Z). Body twists from
cmd_vel become world-frame joint velocity targets here; PhysX integrates
them and still resolves base collisions against world geometry.

Odometry is read back from the same joints (exact) and then degraded by
a configurable drift model into what gets published as wheel odometry —
the EKF (P4) is meant to earn its keep, and `ground_truth/pose` stays
exact alongside. `--odom-noise 0` disables degradation for debugging.

Uses the 6.0 experimental core API (isaacsim.core.experimental.prims
.Articulation): it self-attaches to the physics tensor backend on play —
the deprecated isaacsim.core.prims variant needs SimulationContext (which
segfaults headless on 6.0.1) to create its backend and cannot be used
here. The articulation is FIXED-BASE with ArticulationRootAPI on the
world-anchored px joint, so px/py/rz are ordinary dofs.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

RIG_JOINTS = ["px", "py", "rz"]
CMD_TIMEOUT_S = 0.5          # matches the platform's cmd_vel timeout
ACCEL_LIMIT_XY = 1.0         # m/s^2  — platform control.yaml limit
ACCEL_LIMIT_YAW = 2.0        # rad/s^2


@dataclass
class OdomState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class RidgebackRig:
    def __init__(self, robot_prim_path: str, odom_noise: float = 1.0,
                 seed: int | None = 0):
        """odom_noise scales the default drift model (0 = perfect odom)."""
        from isaacsim.core.experimental.prims import Articulation

        self._art = Articulation(robot_prim_path)
        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_stamp = -math.inf
        self._applied = [0.0, 0.0, 0.0]      # accel-limited world-frame targets
        self._noise = odom_noise
        self._rng = random.Random(seed)
        # noise densities, scaled by odom_noise: σ per sqrt(meter) translation,
        # per sqrt(rad) rotation — standard diff-drive-style drift model
        self._sigma_trans = 0.01
        self._sigma_yaw = math.radians(0.4)
        self._odom = OdomState()
        self._last_joint_pos = None
        self._joint_indices = None

    def ready(self) -> bool:
        return bool(self._art.is_physics_tensor_entity_initialized())

    def initialize(self):
        """Call once ready() is true (physics playing, backend attached)."""
        names = list(self._art.dof_names)
        try:
            self._joint_indices = [names.index(j) for j in RIG_JOINTS]
        except ValueError as e:
            raise RuntimeError(f"rig joints missing from articulation "
                               f"(dofs: {names})") from e

    # ---- command side ----------------------------------------------------

    def set_cmd(self, vx: float, vy: float, wz: float, now: float):
        self._cmd = (vx, vy, wz)
        self._cmd_stamp = now

    def step(self, dt: float, now: float):
        """Accel-limit toward the commanded body twist, write joint targets."""
        import numpy as np

        vx, vy, wz = self._cmd
        if now - self._cmd_stamp > CMD_TIMEOUT_S:
            vx, vy, wz = 0.0, 0.0, 0.0

        yaw = self._rig_positions()[2]
        target_world = (
            vx * math.cos(yaw) - vy * math.sin(yaw),
            vx * math.sin(yaw) + vy * math.cos(yaw),
            wz,
        )
        limits = (ACCEL_LIMIT_XY * dt, ACCEL_LIMIT_XY * dt, ACCEL_LIMIT_YAW * dt)
        for i in range(3):
            delta = target_world[i] - self._applied[i]
            delta = max(-limits[i], min(limits[i], delta))
            self._applied[i] += delta

        self._art.set_dof_velocity_targets(
            np.array([self._applied], dtype=np.float32),
            dof_indices=self._joint_indices,
        )

    # ---- odometry side ---------------------------------------------------

    def _rig_positions(self):
        row = self._art.get_dof_positions().numpy()[0]
        return [float(row[i]) for i in self._joint_indices]

    def _rig_velocities(self):
        row = self._art.get_dof_velocities().numpy()[0]
        return [float(row[i]) for i in self._joint_indices]

    def set_planar_pose(self, x: float, y: float, yaw: float):
        import numpy as np
        # 6.0.1 quirk: the experimental setters silently no-op when handed
        # the full dof array without dof_indices — always use the subset
        # form (probed; the full-array form left positions unchanged).
        self._art.set_dof_positions(
            np.array([[x, y, yaw]], dtype=np.float32),
            dof_indices=self._joint_indices)
        # kill any settle motion too: dof velocities, drive targets, and
        # the accel-limiter state all reset to rest
        zeros = np.zeros((1, 3), dtype=np.float32)
        self._art.set_dof_velocities(zeros, dof_indices=self._joint_indices)
        self._applied = [0.0, 0.0, 0.0]
        self._art.set_dof_velocity_targets(
            zeros, dof_indices=self._joint_indices)
        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_stamp = -math.inf
        self._last_joint_pos = None          # re-seed odom at the new pose

    def ground_truth(self):
        """Exact planar pose (x, y, yaw) from the rig joints."""
        x, y, yaw = self._rig_positions()
        return x, y, yaw

    def update_odom(self):
        """Integrate noisy odometry from true pose increments; return
        (OdomState, body twist (vx, vy, wz))."""
        x, y, yaw = self.ground_truth()
        if self._last_joint_pos is None:
            self._last_joint_pos = (x, y, yaw)
            self._odom = OdomState(x, y, yaw)

        px, py, pyaw = self._last_joint_pos
        dx_w, dy_w = x - px, y - py
        dyaw = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
        self._last_joint_pos = (x, y, yaw)

        # world increment -> body frame of the previous true pose
        c, s = math.cos(pyaw), math.sin(pyaw)
        dx_b = c * dx_w + s * dy_w
        dy_b = -s * dx_w + c * dy_w

        if self._noise > 0.0:
            dist = math.hypot(dx_b, dy_b)
            if dist > 0.0:
                sd = self._noise * self._sigma_trans * math.sqrt(dist)
                dx_b += self._rng.gauss(0.0, sd)
                dy_b += self._rng.gauss(0.0, sd)
                dyaw += self._rng.gauss(
                    0.0, self._noise * self._sigma_yaw * math.sqrt(dist))
            if abs(dyaw) > 0.0:
                dyaw += self._rng.gauss(
                    0.0, self._noise * self._sigma_yaw
                    * math.sqrt(abs(dyaw)) * 0.1)

        o = self._odom
        co, so = math.cos(o.yaw), math.sin(o.yaw)
        o.x += co * dx_b - so * dy_b
        o.y += so * dx_b + co * dy_b
        o.yaw = math.atan2(math.sin(o.yaw + dyaw), math.cos(o.yaw + dyaw))

        vx_w, vy_w, wz = self._rig_velocities()
        c, s = math.cos(yaw), math.sin(yaw)
        body_twist = (c * vx_w + s * vy_w, -s * vx_w + c * vy_w, wz)
        return o, body_twist
