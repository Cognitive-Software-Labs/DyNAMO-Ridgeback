# Chassis Attitude and the Floor Crop — Simulator Measurements

**Scope:** the two constants behind `HeightCrop`
(`perception/target_localization/core/isolation_3d.py`) —
`BASE_ABOVE_FLOOR_M_DEFAULT = 0.026` and `FLOOR_MARGIN_M_DEFAULT = 0.05`. This
document records what the simulator can and cannot say about them, measured
2026-09-09.

Two questions were asked of the simulator:

1. Does the chassis actually rest where `BASE_ABOVE_FLOOR_M_DEFAULT` claims?
2. Does the chassis tilt — statically or under acceleration — by enough for the
   floor margin to be doing work?

The first is answered and the constant is now grounded. The second is answered
in a way that forecloses the simulator as evidence for the margin at all.

## Why the margin depends on attitude

`HeightCrop` keeps points whose height above the floor exceeds
`floor_margin_m`. Height above the floor is derived from the camera extrinsic
plus the fixed chassis offset; the floor plane itself is never observed. The
crop therefore inherits any error in the assumed *orientation* of the robot,
and that error projects onto height in proportion to range: a chassis tilted by
θ mispredicts the floor by `R·sin θ` at horizontal distance `R`.

For the shipped scene set, whose farthest target is 10.5 m, the tilt that would
consume the entire margin is

```
asin(0.05 / 10.5) = 0.2728 degrees
```

Below that, floor points stay below the crop plane at every scene distance.
Above it, floor survives the crop at the far end of the set.

Nothing in the stack observes chassis attitude. The EKF runs with
`two_d_mode: True`, and its `imu0_config` enables only yaw rate (index 11) and
x acceleration (index 12) — roll and pitch (indices 3 and 4) are `False`
(`~/clearpath/platform/config/localization.yaml`). A Madgwick filter does run
and does publish a full orientation on `sensors/imu_0/data`, but the EKF
consumes two channels of it and discards the attitude. TF therefore reports
`base_link` as exactly level at all times, by configuration.

## Method

Minimal headless simulation, no RViz, no detector, no perception stack:

```
ros2 launch install/ridgeback_autonomy/share/ridgeback_autonomy/launch/includes/simulation.launch.py \
  world:=target_distance_calibration clearpath_rviz:=false gz_gui:=false
```

Truth comes from Gazebo directly, not from TF or the IMU:
`/world/target_distance_calibration/dynamic_pose/info`. `base_link`'s pose
relative to the model is exactly identity, so the model pose is `base_link`'s
pose. Roll and pitch are taken from that quaternion.

The driving case publishes `geometry_msgs/msg/TwistStamped` on
`r100_0001/cmd_vel` in three phases — 3 s at rest, 5 s at 0.8 m/s, 5 s back at
rest — while the pose stream is captured continuously.

## Result 1 — the chassis offset is confirmed

The floor is not at `z = 0`. `calibration_floor` is an `18 × 10 × 0.1` box
posed at `3 0 0`, so its **top surface is at z = +0.05**
(`sim/worlds/target_distance_calibration.sdf`).

| Quantity | Value |
|---|---|
| `base_link` world z, at rest | 0.075899584 m |
| floor top surface | 0.050000000 m |
| **`base_link` above floor** | **0.025899584 m** |
| shipped `BASE_ABOVE_FLOOR_M_DEFAULT` | 0.026 m |
| predicted from URDF (`0.0759 − 0.0500`) | 0.0259 m |
| contact penetration | 0.42 µm |

The constant is correct to 0.1 mm, and this is now a measurement of where the
physics engine rests the robot rather than a re-reading of the description that
generated it. The wheel-radius-minus-axle-offset derivation is confirmed
independently.

This grounds the constant **for the simulator only**. The robot's configured
`serial_number: r100-0001` is, per the Ridgeback manual, the 296 mm platform;
heights were raised 15 mm at serial R100-0111 and the description models the
taller variant (`deck_height` 0.280 puts the deck at ~306 mm above the floor).
That discrepancy is untouched by anything here and remains open for hardware.

## Result 2 — the simulated chassis cannot tilt

| | at rest | driving 0 → 0.8 m/s → 0 |
|---|---|---|
| peak \|roll\| | 8.04e-09 ° | 8.04e-09 ° |
| peak \|pitch\| | 2.24e-08 ° | 4.87e-07 ° |
| z excursion | — | 3.9e-07 m |
| samples | 3 | 1684 over 3.69 m travelled |

Peak observed tilt is roughly 560,000× below the 0.2728° threshold. The
implied floor-height error at 10.5 m is 8.9e-08 m.

The cause is structural, not incidental. The rockers carry revolute limits of
±0.08726 rad but are instantiated with `joint_type="fixed"`
(`clearpath_platform_description/urdf/r100/drivetrain/wheels.urdf.xacro`), and
contact penetration is sub-micron. The model has no suspension degree of
freedom, so chassis tilt has nowhere to express itself. This is not a flat
floor happening to produce a level robot; it is a robot that cannot tilt.

## What this forecloses

- **The floor margin cannot be validated or falsified in simulation.** With
  attitude error identically zero and depth rendered without a noise model or a
  stereo baseline (stated in `intel_realsense.urdf.xacro`), every margin from
  0 upward classifies floor points identically. A benchmark result showing no
  sensitivity to `floor_margin_m` is a property of the model, not evidence
  about the value.
- **Simulation cannot estimate the hardware figure either.** Driving the model
  was expected to yield an informed estimate of chassis pitch for this mass and
  centre of gravity. It cannot: the compliance that would produce pitch is not
  modelled at all. The simulator yields no information about hardware attitude
  error — not a small value, no value.

## What remains measurable here

The margin's *cost* is simulator-visible even though its benefit is not. The
crop discards everything below `floor_margin_m`, so an object shorter than 5 cm
loses every point after a successful detection, and the miss surfaces as
`ISOLATION_EMPTY` rather than as a height-related reason. At 640 px wide and
f ≈ 443 px, a 5 cm object still spans ~9.5 px at 2 m, so the sensor is not the
limit — the crop is. That regression reproduces without any of the error
sources the margin exists to absorb.

## Consequences

- `BASE_ABOVE_FLOOR_M_DEFAULT`'s comment may now honestly say measured, citing
  the method above. Hardware remains unverified.
- `FLOOR_MARGIN_M_DEFAULT` stays ungrounded and cannot be grounded from this
  repository. Its value is an error budget dominated by an attitude term that
  only hardware can supply.
- Any range-scaled replacement for a fixed margin must be argued from geometry
  and deployment conditions. The benchmark holds the robot stationary on a
  perfectly flat floor and will report no difference either way.
