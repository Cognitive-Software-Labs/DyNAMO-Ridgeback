# Matching the explainer pedestrian choreography

Recorded dates: 2026-11-05
Tested revisions: unknown
Provenance: partial

## Reference and population

The earlier modern-office crowd used nine people and 20-second routes. The
HTML illustration instead uses eleven people over 24 seconds. Smoothing cached
poses and correcting initial facing did not resolve that population mismatch.
The separate `explainer` scenario copies the pedestrian choreography, leaving
the six benchmark families available unchanged by this addition.

The [reference animation block](assets/2026-11-05-explainer-pedestrian-motion/illustration-sim-block.txt)
preserves its leader, ten bystander routes and times. World coordinates use
`x = svg_x - 2.2`, `y = 4.6 - svg_y`, without rescaling distances or time.
Three group members remain stationary. The look-alike repeats the leader's
model and scale. The observer robot stays at the origin: the illustration's
robot following, lock-state controller and avoidance are not implemented by
this playback scenario. No recognition inference is run.

The illustration-specific office expands its floor and shifts side partitions
and furniture outside the scripted walking area. It does not clip crossing
paths at the original room boundary. Pedestrians follow their prescribed
paths rather than reacting to one another; close approaches from the drawing
are not resolved by an avoidance controller.

## Verification and provenance

The regression check compares every person against the HTML at all 721
30 Hz display times, including stationary positions. Separate checks cover
static obstacle clearance, look-alike assignment, delayed initial facing and
interpolated cached motion. The [test output](assets/2026-11-05-explainer-pedestrian-motion/tests.log)
and [build output](assets/2026-11-05-explainer-pedestrian-motion/build.log)
record the automated checks. Full playback checks use simulator-observed poses,
sensor coverage, calibration and startup lidar visibility.

Testing used a dirty tree while another process changed the branch history;
a single exact tested commit is not established. The [working-tree source
snapshot](assets/2026-11-05-explainer-pedestrian-motion/sources.tar.gz)
and [patch](assets/2026-11-05-explainer-pedestrian-motion/source.patch)
preserve the implementation, with runtime source snapshots retained separately
for each backend. The environment, drivers and full dependency installation
are not preserved. Raw run output remains under the local artifact root
`artifacts/leader-following/explainer-motion-review/`.

Both backends passed the complete 24-second run with eleven people and 5,280
observed truth samples per backend. Maximum observed-versus-commanded pose
error was 2.3e-16 in Gazebo and 1.4e-7 m/rad in Isaac (rounded upper bounds).
The [playback summary](assets/2026-11-05-explainer-pedestrian-motion/playback-summary.json)
and [runtime source, manifest, sensor and truth snapshots](assets/2026-11-05-explainer-pedestrian-motion/runtime-sources-and-truth.tar.gz)
preserve this limited scene/sensor result. This does not establish recognition,
robot following or reactive pedestrian behaviour.
