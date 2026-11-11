# Gazebo odometry heading during leader following

Recorded dates: 2026-11-11
Tested revisions: `63fd602d98b2e36a73df20d46adea52f48a6614e`
Provenance: partial

The checkout was dirty: it carried the Gazebo `--drive` EKF override this
record tested. The follow controller and Nav2 configuration were as committed.

## Symptom

In the explainer office with the faster follow configuration, Gazebo lost the
leader lock for about 12.6 of 24 seconds while Isaac did not. The
[recorder](assets/2026-11-11-f4-gazebo-odometry/recorder.py) logged
`platform/odom/filtered` alongside simulator truth. Filtered odometry heading
was within 0.1° for the first 5 seconds. It then diverged once the robot turned
and strafed, reaching −55° and a 3 m position error by the end. With that
heading, tracked people moved as the robot turned. Tracks lost continuity,
produced duplicate tracks 0.06–0.10 m apart and dropped below the confidence
floor.

## Isolation

A [probe](assets/2026-11-11-f4-gazebo-odometry/spin_probe.py) held the crowd
static and commanded fixed velocities, recording raw wheel odometry, filtered
odometry and integrated IMU rate against truth:

| Test | True yaw change | Wheel odometry | Filtered | IMU |
|---|---:|---:|---:|---:|
| Spin 0.5 rad/s, then drive and turn | 263.2° | 271.9° | 271.9° | 263.3° |
| Strafe, then drive + strafe + turn | 52.4° | 102.3° | 102.3° | 52.3° |

Pure strafing kept yaw within 0.1°. Combined strafe and turn roughly doubled
the wheel-odometry yaw change, and Clearpath's EKF follows absolute wheel yaw
([generated configuration](assets/2026-11-11-f4-gazebo-odometry/gz-strafe/localization.yaml)).
The IMU rate integrated within 0.3°.

## Change and result

Gazebo `--drive` runs write a run-local `platform.extras.ros_parameters.ekf_node`
override that fuses only wheel `vx`/`vy` and takes yaw from the IMU rate. The
repository and hardware declarations are unchanged. With the override, the
same strafe test gave a filtered yaw of 52.4° against 52.4° true
([configuration](assets/2026-11-11-f4-gazebo-odometry/gz-strafe2/localization.yaml)).
Position still drifted 2.8 m, because the wheels report strafing that the
simulated base does not achieve.

The [analysis](assets/2026-11-11-f4-gazebo-odometry/analyse.py) now maps each
odometry-frame lock target into the world through the recorded odometry and
truth poses. Earlier results assumed odometry equals the world frame.

| Explainer office, Gazebo | Heading error at end | Lock off | On leader | Wrong / unmatched | Final distance |
|---|---:|---:|---:|---:|---:|
| Before the override | −55° | 12.6 s | 11.3 s | 0 / 0 s | 12.3 m |
| With the override | < 1° | 3.3 s | 20.6 s | 0 / 0 s | 5.7 m |

Both rows use the corrected analysis. The earlier "wrong" and "unmatched"
Gazebo labels came from odometry error, not from a lock on another person.
Isaac's comparable run had 20.6 s on the leader. In the override run the
look-alike passed within 0.33 m of the robot centre while the robot averaged
0.02 m/s. [Results](assets/2026-11-11-f4-gazebo-odometry/results.txt), truth
and event logs are kept.

## Limits

Single runs at half real time. Odometry position drift of about 1 m remains in
the explainer scene. The physical Ridgeback's wheel odometry was not
evaluated, and this override applies only to simulated Gazebo following.
