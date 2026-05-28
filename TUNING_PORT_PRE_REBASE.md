# Pre-rebase tuning snapshot — port to `find-g1`

Captured at commit `e0814c8` (tag `pre-rebase-custom_exploration`).

Robot drives noticeably faster + smoother on `office` here than on `find-g1`
post-rebase. This file lists every param that differs between the two
checkouts so they can be re-applied on `find-g1` and benchmarked.

For each file the table shows: param, value on `find-g1` (current branch we
want to fix), value on `e0814c8` (this working snapshot), and a one-line
note. Default goal of the port is the `e0814c8` value unless the note says
otherwise.

---

## `src/ridgeback_autonomy/config/nav2_params.yaml`

### bt_navigator

| param                    | find-g1 | e0814c8 (port) | note |
|--------------------------|---------|----------------|------|
| `enable_stamped_cmd_vel` | absent  | `true`         | needed if the controller chain runs TwistStamped — keep |

### controller_server / general_goal_checker

| param                | find-g1 | e0814c8 (port) | note |
|----------------------|---------|----------------|------|
| `xy_goal_tolerance`  | 0.55    | **0.40**       | tighter goal acceptance → robot drives closer to frontier center |
| `yaw_goal_tolerance` | 0.55    | **0.40**       | matching |

### controller_server / FollowPath (MPPI)

| param                | find-g1 | e0814c8 (port) | note |
|----------------------|---------|----------------|------|
| `time_steps`         | 35      | **56**         | longer rollout horizon (2.8 s) |
| `model_dt`           | 0.05    | 0.05           | same |
| `batch_size`         | 2000    | 2000           | same |
| `ax_max`             | 1.0     | **3.0**        | KEY: high accel → reach vmax in ~0.17 s instead of 0.9 s |
| `ax_min`             | -1.0    | **-3.0**       | matching |
| `ay_max`             | 1.0     | **3.0**        | matching |
| `az_max`             | 2.0     | **3.5**        | faster yaw response |
| `vx_std`             | 0.5     | **0.2**        | smaller forward sample noise — less jitter |
| `vy_std`             | 0.1     | **0.2**        | KEEP find-g1 0.1 — pre-rebase had lateral leak; ours is tighter. Skip this port |
| `wz_std`             | 0.65    | **0.4**        | smaller yaw noise |
| `vx_max`             | 0.9     | **0.5**        | LOWER cap but combined with high `ax_max` robot reaches it instantly |
| `vx_min`             | -0.5    | **-0.35**      | matching scaled down |
| `vy_max`             | 0.75    | **0.5**        | matching |
| `wz_max`             | 2.6     | **1.9**        | calmer yaw |
| `iteration_count`    | 2       | **1**          | single pass — saves CPU, less jitter actually |
| `prune_distance`     | 6.0     | **1.7**        | KEEP find-g1 6.0 — but only if PathFollow is rebalanced (see critics) |
| `temperature`        | 0.5     | **0.3**        | narrower softmin — elite samples dominate, less averaging blur |
| `regenerate_noises`  | false   | **true**       | fresh noise per cycle — pairs with `iteration_count: 1` |
| `TrajectoryVisualizer.trajectory_step` | 2 | **5** | fewer rendered trajectories — lighter rviz load |

### controller_server / FollowPath critics

| critic.param                              | find-g1 | e0814c8 (port) | note |
|-------------------------------------------|---------|----------------|------|
| `GoalCritic.threshold_to_consider`        | 0.7     | **1.4**        | activates farther out — earlier precision mode |
| `PathAlignCritic.max_path_occupancy_ratio`| 0.08    | **0.05**       | tighter path tracking allowed because robot is already slow precision |
| `PathFollowCritic.cost_weight`            | 14.0    | **5.0**        | KEY: lower because high `ax_max` already produces speed; weight 14 caused corner cuts |
| `PathFollowCritic.threshold_to_consider`  | 0.7     | **1.4**        | matches GoalCritic threshold |

### local_costmap / global_costmap (inflation)

| param                | find-g1 | e0814c8 (port) | note |
|----------------------|---------|----------------|------|
| `cost_scaling_factor`| 3.0     | **5.0**        | both costmaps — sharper falloff from walls |
| `inflation_radius`   | 0.65    | **0.75**       | both costmaps — larger keep-out zone |

### global_costmap (frequency)

| param                 | find-g1 | e0814c8 (port) | note |
|-----------------------|---------|----------------|------|
| `update_frequency`    | 1.0     | **2.0**        | faster global costmap refresh |
| `publish_frequency`   | 1.0     | **2.0**        | matching |

### planner_server / GridBased (NavFn)

| param        | find-g1 | e0814c8 (port) | note |
|--------------|---------|----------------|------|
| `tolerance`  | 1.0     | **0.75**       | planner more strict about reaching goal exactly |

### behavior_server (rotational limits for recovery behaviors)

| param                | find-g1 | e0814c8 (port) | note |
|----------------------|---------|----------------|------|
| `max_rotational_vel` | 1.8     | **1.0**        | recovery spins are calmer |
| `min_rotational_vel` | 0.25    | **0.2**        | matching |
| `rotational_acc_lim` | 1.2     | **0.5**        | gentler ramp |

### velocity_smoother

| param           | find-g1                     | e0814c8 (port)                  | note |
|-----------------|-----------------------------|---------------------------------|------|
| `max_velocity`  | `[1.0, 0.85, 2.8]`          | **`[1.3, 1.3, 4.0]`**           | wider headroom — wheel-controller cap (1.3) is the ceiling |
| `min_velocity`  | `[-0.6, -0.85, -2.8]`       | **`[-1.3, -1.3, -4.0]`**        | symmetric |

### collision_monitor / FootprintApproach

| param                   | find-g1 | e0814c8 (port) | note |
|-------------------------|---------|----------------|------|
| `time_before_collision` | 0.8     | **1.2**        | looks further ahead |
| `min_points`            | 10      | **6**          | triggers on fewer lidar points |
| `enabled`               | false   | **true**       | RE-ENABLE — was disabled while debugging |

---

## `src/ridgeback_autonomy/config/frontier_explorer_params.yaml`

| param                | find-g1 | e0814c8 (port) | note |
|----------------------|---------|----------------|------|
| `goal_advance_cells` | 3       | **10**         | push goal 10 cells past frontier centroid (~0.6 m) — better inside-room landing |

---

## `src/ridgeback_autonomy/config/slam_toolbox_params.yaml`

| param                     | find-g1 | e0814c8 (port) | note |
|---------------------------|---------|----------------|------|
| `minimum_travel_distance` | 0.0     | **0.05**       | throttle scan additions — less DB churn |
| `minimum_travel_heading`  | 0.0     | **0.05**       | matching |

NOTE: find-g1 set these to 0 deliberately during benchmarking to capture every
scan. If we port back to 0.05 we lose dense scan logging — only do so if SLAM
performance / map quality benefits in practice.

---

## Hypothesis on why robot is faster here

It's NOT `vx_max` — that's actually lower at 0.5 vs find-g1's 0.9. The combination that matters:

1. **`ax_max: 3.0`** — robot accelerates to vmax in ~0.17 s. find-g1 at `ax_max: 1.0` needs 0.9 s to ramp, so over typical short MPPI horizons it never reaches vmax.
2. **`temperature: 0.3` + `iteration_count: 1` + `regenerate_noises: true`** — narrow softmin picks elite sample each tick; fresh noise prevents prior from getting trapped at 0.
3. **`velocity_smoother.max_velocity [1.3,1.3,4.0]`** — smoother doesn't re-cap below MPPI's already-low 0.5 vmax. find-g1 caps smoother at 1.0 / 0.85.
4. **`PathFollowCritic.cost_weight: 5.0` + `max_path_occupancy_ratio: 0.05`** — moderate path-follow weight + tight occupancy keeps the robot ON the path without overshooting; find-g1's weight 14 makes MPPI cut corners.

---

## Migration checklist (apply on `find-g1`)

When porting back to `find-g1`:

- [ ] Apply all rows marked **port** in tables above, EXCEPT:
  - keep `vy_std: 0.1` (ours is better)
  - keep `prune_distance: 6.0` (only if PathFollow weight is also reduced)
  - re-evaluate `slam.minimum_travel_*` — only port if benchmarking continues
- [ ] Restart stack: `bash cleanup.sh && bash start_exploration.sh mock_hospital custom`
- [ ] Verify HUD `actual` row reaches ~0.4 m/s in open hallway (vs ~0.06 m/s currently)
- [ ] Re-enable `collision_monitor.FootprintApproach.enabled: true` and test that robot doesn't stall against walls
- [ ] If trajectories smooth and tracking is good, commit as `tune(mppi): port pre-rebase tuning back into find-g1`
