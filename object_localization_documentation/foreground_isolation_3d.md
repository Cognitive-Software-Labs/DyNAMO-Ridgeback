# Euclidean reconstruction — Foreground Isolation Methods (3D Point-Domain Route)

**Context.** Companion to `foreground_isolation_2d.md`, which covers the 2D
depth-image route (projective ranging). In euclidean reconstruction the mask has already selected a point set
out of the organized cloud — but with a *rect* mask that set is the whole box
frustum: the G1 **plus** the floor strip under it **plus** whatever background
falls inside the box. Foreground isolation must then separate them **in the
point domain**. When the mask is *tight*, isolation reduces to statistical
outlier cleanup and a centroid.

**Contract (same shape as the 2D one).** Input: the masked point set (organized
where possible, so pixel indices remain available). Output: the **foreground
point subset** — the points belonging to the G1. Both downstream reductions
(centroid for the coordinate, median range for the distance) read this one
subset, so they agree by construction. Every method below is one implementation
of this contract, swappable behind it for the benchmark.

**Relationship to the 2D methods.** The 2D catalogue does not become obsolete
in euclidean reconstruction:

- **Value-based 2D methods transfer verbatim** (nearest-mode histogram, Otsu,
  GMM, mean-shift): depth value *is* the Z coordinate, so running them before
  deprojection or on `points[...].z` after is the same 1-D computation.
- **Image-structure methods** (region growing, GrabCut, CRF, SAM) still apply
  when the cloud is organized — they run on the image-grid side and *select*
  points by pixel index.
- The methods in **this** document are the 3D-native additions: they exploit
  Euclidean geometry that no depth-value distribution can express. The floor is
  the canonical example — inside the box its depth values span a continuous
  ramp (no separable mode), but in 3D it is a literal plane.

The two axes worth keeping in mind when analyzing results:

- **Cue:** geometric prior (1, 3, 4) vs. distance distribution (2) vs.
  spatial connectivity (5, 6) vs. global/graph (7, 8) vs. learned (9)
- **Input needed:** XYZ only (1, 2, 3, 5) vs. XYZ + normals (4, 6, 8) vs.
  XYZ + seed/prior (7) vs. XYZ (+RGB) + model (9)

---

## 1. Extrinsic Ground-Plane Crop (Height Filter)

**Idea.** The camera's pose above the floor is *known*. Transform the masked
points into a gravity-aligned frame and drop everything below `z < ε`. The
floor is removed by prior knowledge, not estimation. In the shipped `HeightCrop`
(`perception/core/isolation_3d.py`) the camera height and pitch are read from
live TF at runtime via `camera_floor_geometry` (the camera-above-base
translation plus a fixed chassis offset, and the pitch from the rotation); the
`CAMERA_HEIGHT_M_DEFAULT = 1.053` constant is only the static/test default. The
legacy `camera_config.json` height of 0.85 m is deliberately left untouched and
unused by the mask stack.

**Pros**
- Essentially free: one rigid transform (already needed anyway) plus one
  comparison per point.
- Deterministic, zero tuning beyond ε; cannot "fit" the wrong thing because it
  fits nothing.
- Uses information every other method ignores: the extrinsics are calibrated
  and the Ridgeback's floor is genuinely flat.

**Cons**
- Trusts calibration: pitch error of a few degrees tilts the virtual floor and
  either shaves the G1's feet or keeps a wedge of real floor.
- Only removes the floor — far background (walls, furniture) passes untouched,
  so it is a partial isolator that needs a companion (e.g. Method 2 or 5).
- Assumes flat, level ground; fails on ramps or uneven terrain.

**Paper access.** Folk method (a fixed half-space crop); no canonical
publication. Document as "extrinsic height crop" in the comparison. The
pass-through/crop-box filters in PCL are the standard implementation:
R. B. Rusu, S. Cousins, *"3D is here: Point Cloud Library (PCL),"* IEEE ICRA,
2011. DOI: `10.1109/ICRA.2011.5980567` (open PDF on pointclouds.org).

---

## 2. Range-Band Selection (Percentile Anchor + Inlier Window)

**Idea.** Sort the masked points by distance, anchor at a low percentile ("the
near surface"), keep points within a fixed window around the anchor, discard the
rest. The shipped `RangeBand` (`perception/core/isolation_3d.py`) anchors on the
**Euclidean camera-frame range** of each point (`np.linalg.norm`), keeping a
25th-percentile anchor with a −0.10 m / +0.35 m inlier window. The legacy
`pointcloud` estimator (`compute_pointcloud_measurement` in `geometry.py`)
anchors on *forward* distance instead; the two are identical at zero lateral
offset and diverge only as the object moves off the optical axis. It is the
point-domain twin of the 2D nearest-mode histogram.

**Pros**
- Already implemented and benchmarked (MAE 0.156 m in the 70-trial sim run of
  2026-07-13; see `benchmark-results/`, figure possibly superseded by a later
  run) — the incumbent every other method must beat.
- A handful of NumPy lines; negligible runtime.
- The percentile anchor is robust to a moderate fraction of nearer-than-object
  noise, which a plain minimum is not.

**Cons**
- Same failure as its 2D twin: clutter *in front of* the G1 captures the
  anchor.
- Window width is a magic number tied to the G1's body depth (~0.35 m); wrong
  for any other object.
- No spatial coherence: floor points that happen to lie in the range band
  survive (the window's asymmetry is precisely a hand-tuned patch for this).

**Paper access.** Folk method; no canonical publication. The percentile-anchor
formulation is this repository's own (`geometry.py`); document as
"range-band / percentile anchor" in the comparison.

---

## 3. RANSAC Plane Removal

**Idea.** Estimate the dominant plane in the masked point set with RANSAC,
remove its inliers, optionally repeat for a second plane (wall). What survives
is the non-planar residue — the G1. The estimated twin of Method 1: no
calibration trust, the floor is found from the data.

**Pros**
- Robust to calibration drift and moderate outlier contamination — RANSAC's
  home turf.
- Also removes a *wall* behind the G1, which Method 1 cannot.
- Standard, battle-tested component (`SACSegmentation` in PCL); the classic
  first stage of the tabletop-segmentation pattern.

**Cons**
- The G1 itself has near-planar patches (torso plates); with few floor points
  in the box, RANSAC can lock onto the robot and delete the subject.
  Constrained variants (fit only near-horizontal planes, use Method 1's prior
  as a gate) mitigate this.
- Iterative and randomized: runtime and result vary frame to frame unless
  seeded/limited.
- Distance threshold (inlier band of the plane) is a tuning knob; too wide
  eats the G1's feet.

**Paper access.**
M. A. Fischler, R. C. Bolles, *"Random Sample Consensus: A Paradigm for Model
Fitting with Applications to Image Analysis and Automated Cartography,"*
Communications of the ACM, vol. 24, no. 6, pp. 381–395, 1981.
- DOI: `10.1145/358669.358692` (ACM Digital Library; PDF widely mirrored)

---

## 4. Normal-Based Filtering

**Idea.** Estimate a surface normal per point (PCA over a local neighborhood),
then drop points whose normal is near-vertical — floor points announce
themselves by pointing up. A soft, per-point version of plane removal that
needs no global model fit.

**Pros**
- Removes *any* horizontal surface (floor at multiple heights, low steps), not
  just the single dominant plane.
- Per-point decision: no global fit to go wrong, trivially parallel.
- The computed normals are reusable by Methods 6 and 8 — shared cost when
  chaining.

**Cons**
- Normal estimation is the expensive part (k-NN search per point) and is noisy
  on sparse or edge regions — exactly where the G1 meets the floor.
- Neighborhood radius is a tuning knob coupling to point density (which varies
  with distance).
- Keeps vertical background (walls) — partial isolator, like Method 1.

**Paper access.** PCA normal estimation traces to:
H. Hoppe, T. DeRose, T. Duchamp, J. McDonald, W. Stuetzle, *"Surface
Reconstruction from Unorganized Points,"* SIGGRAPH 1992, pp. 71–78.
- DOI: `10.1145/133994.134011` (open PDF on hhoppe.com)
The practical treatment for robotic point clouds is R. B. Rusu's dissertation,
*"Semantic 3D Object Maps for Everyday Manipulation in Human Living
Environments,"* TU München, 2009 (open access via TUM library; condensed in KI
– Künstliche Intelligenz 24(4), 2010, DOI: `10.1007/s13218-010-0059-6`).

---

## 5. Euclidean Cluster Extraction / DBSCAN

**Idea.** Connected components in 3D: group points whose neighbor distance is
below a threshold, emit clusters, then pick the G1's cluster — nearest along
the ray, largest, or closest to the mask centroid. The background is discarded
not by its depth *value* but by the empty space separating it from the robot.
DBSCAN is the density-based generalization (adds noise labels, minPts).

**Pros**
- The natural formalization of "the G1 is one contiguous object": exploits the
  air gap between subject and background directly.
- Wins where range bands fail — background at a *similar* distance but
  physically separate (a pillar beside the G1).
- Standard, fast implementations (`EuclideanClusterExtraction` in PCL,
  DBSCAN in every ML library); near-linear with a k-d tree or the organized
  grid's neighborhood structure.

**Cons**
- Cluster tolerance is a critical knob coupled to point density: too small
  fragments the G1 (legs become separate clusters), too large fuses it with
  the floor — which is why plane removal (Method 1/3) usually runs *first*.
- Cluster *selection* needs its own rule; "nearest cluster" re-imports Method
  2's failure mode (clutter in front wins).
- Point density falls with distance squared; a fixed tolerance that works at
  1.5 m over-fragments at 5.5 m.

**Paper access.**
Euclidean cluster extraction: R. B. Rusu dissertation (see Method 4), §6.2 —
the reference PCL cites.
DBSCAN: M. Ester, H.-P. Kriegel, J. Sander, X. Xu, *"A Density-Based Algorithm
for Discovering Clusters in Large Spatial Databases with Noise,"* KDD-96,
pp. 226–231, 1996.
- Open access via AAAI Press (KDD-96 proceedings PDF online).

---

## 6. Region Growing with Smoothness Constraint

**Idea.** The 3D twin of 2D seeded region growing, but the growth criterion is
*surface smoothness*: start from a seed (lowest-curvature point, or the mask
centroid), add neighbors while the angle between normals stays small. Growth
stops where the surface bends sharply — the G1/floor junction is exactly such
a crease, even though the two surfaces *touch* (no air gap for Method 5 to
exploit).

**Pros**
- Separates touching surfaces that Euclidean clustering fuses — the
  feet-on-floor contact is its home case.
- Produces surface-coherent segments; robust to depth speckle.
- Reuses Method 4's normals — natural chain.

**Cons**
- Two coupled knobs (angle threshold, curvature threshold) plus seed choice.
- The G1 is not one smooth surface — joints and plate edges are creases too,
  so the robot fragments into several segments that must be re-merged (e.g.
  by a bounding-volume or connectivity rule).
- Inherits normal-estimation cost and noise (Method 4's cons).

**Paper access.**
T. Rabbani, F. van den Heuvel, G. Vosselman, *"Segmentation of Point Clouds
Using Smoothness Constraint,"* ISPRS Archives XXXVI-5, pp. 248–253, 2006.
- Open access: ISPRS archives (isprs.org); the reference PCL's
  `RegionGrowing` cites.

---

## 7. Min-Cut Point-Cloud Segmentation

**Idea.** The 3D twin of GrabCut: build a graph over the points, set a
foreground seed (mask centroid deprojected, or Method 2's anchor point) and an
expected object radius, weight edges by distance, and run min-cut to split
foreground from background globally. Binary, seed-driven, exact.

**Pros**
- Global optimum for its energy — no greedy growth artifacts, no cluster
  fusing/fragmenting knobs.
- The radius prior matches this use case well: the G1's physical size is fixed
  and known.
- Handles subject-touches-background *and* similar-distance clutter in one
  formulation.

**Cons**
- Heaviest classical method here: graph construction plus max-flow per mask
  per frame.
- Seed and radius are hard priors: a bad seed (detection box off-center)
  poisons the cut.
- Less standard tooling than Methods 3/5 (PCL's `MinCutSegmentation` exists
  but sees far less use).

**Paper access.**
A. Golovinskiy, T. Funkhouser, *"Min-Cut Based Segmentation of Point Clouds,"*
IEEE ICCV Workshops (3DRR), 2009.
- DOI: `10.1109/ICCVW.2009.5457721`; open PDF on the Princeton graphics site.

---

## 8. Supervoxel + Convexity Segmentation (LCCP)

**Idea.** Over-segment the cloud into supervoxels (3D superpixels honoring
boundaries), then merge adjacent supervoxels whose junction is *convex* —
object-internal junctions tend to be convex, object-to-support junctions
concave. The G1 emerges as the union of mutually convex patches sitting on a
concave crease with the floor.

**Pros**
- Object-level reasoning with zero learned models; the convexity cue is
  remarkably general.
- Handles multi-part objects (exactly the joints/plates that fragment
  Method 6) by design.
- Supervoxel stage is reusable structure for any downstream reasoning.

**Cons**
- Two-stage pipeline with several resolution/seed parameters; the heaviest
  tuning burden of the classical methods.
- Runtime well above Methods 1–5; overkill when one robot stands on one
  floor.
- Convexity cue degrades on noisy normals at range (density again).

**Paper access.**
Supervoxels: J. Papon, A. Abramov, M. Schoeler, F. Wörgötter, *"Voxel Cloud
Connectivity Segmentation — Supervoxels for Point Clouds,"* CVPR 2013.
- DOI: `10.1109/CVPR.2013.264` (open access via CVF).
LCCP: S. C. Stein, M. Schoeler, J. Papon, F. Wörgötter, *"Object Partitioning
Using Local Convexity,"* CVPR 2014.
- DOI: `10.1109/CVPR.2014.46` (open access via CVF).

---

## 9. Learned 3D Segmentation — Upper Bound

**Idea.** The SAM slot of the 3D route: feed the (masked or full) cloud to a
learned point-cloud segmentation network (PointNet++ family, or newer
transformer-based successors) and take the points labeled as the
robot/person-like object. Included as the learned upper bound, not as a
deployment candidate.

**Pros**
- Learns shape priors no geometric heuristic encodes (a humanoid silhouette in
  3D).
- Robust to exactly the hard cases above: touching surfaces, similar-range
  clutter, fragmented parts.

**Cons**
- Heavy runtime and a model dependency on a robot whose GPU budget is already
  spent on detection (and possibly Depth-Anything).
- Needs training data or a pre-trained class that matches "G1 humanoid" —
  nothing off-the-shelf guarantees that.
- Overkill: one known rigid-ish object on a flat floor is a geometric problem.

**Paper access.**
C. R. Qi, L. Yi, H. Su, L. J. Guibas, *"PointNet++: Deep Hierarchical Feature
Learning on Point Sets in a Metric Space,"* NeurIPS 2017.
- Open access: arXiv `1706.02413`.

---

## Summary Comparison

| # | Method | Cue | Handles touching FG/BG | Speed | Tuning burden | Extra input | Key risk |
|---|--------|-----|------------------------|-------|---------------|-------------|----------|
| 1 | Extrinsic height crop | Known extrinsics | floor only | ★★★★★ | ε | TF/calib | Calibration drift; non-floor BG passes |
| 2 | Range band (percentile) | Distance distribution | No | ★★★★★ | Window width | — | Clutter in front; **incumbent** |
| 3 | RANSAC plane removal | Fitted plane | floor/wall | ★★★★☆ | Inlier band | — | Locks onto the G1's flat patches |
| 4 | Normal filter | Per-point normals | floor only | ★★★☆☆ | Radius, angle | normals | Noisy normals at range |
| 5 | Euclidean cluster / DBSCAN | Air-gap connectivity | **No** (fuses) | ★★★★☆ | Tolerance vs. density | — | Fragment at range, fuse at floor |
| 6 | Region growing (smoothness) | Normal continuity | **Yes** | ★★★☆☆ | Angle, curvature, seed | normals | G1 self-fragments at joints |
| 7 | Min-cut | Global graph + radius prior | **Yes** | ★★☆☆☆ | Seed, radius | seed | Bad seed poisons the cut |
| 8 | Supervoxel + LCCP | Convexity | **Yes** | ★★☆☆☆ | Resolutions, seeds | normals | Heaviest tuning; overkill |
| 9 | Learned 3D seg | Learned shape | **Yes** | ★☆☆☆☆ | Model choice | model | Runtime; no G1 class off-the-shelf |

**Chaining is the norm, not the exception.** The textbook tabletop recipe is
3 → 5 (plane removal, then clustering, then pick the cluster); the cheapest
credible recipe is 1 → 2 (height crop, then range band) in pure NumPy with no
PCL dependency. Methods 1/3/4 are floor-removers, Methods 2/5 are
background-separators — one of each makes a complete isolator.

## Recommended Evaluation Protocol

1. **Ground truth:** in sim, the spawned G1's pose is exact — both the true
   centroid and the true distance come free from the Gazebo pose topic (unlike
   the 2D route, no hand labeling is needed for the distance metric;
   point-level labels are only needed if per-point IoU is wanted).
2. **Metrics per method:** distance error of the reduced estimate (the number
   that matters), 3D centroid error, foreground point count stability across
   frames, runtime per mask.
3. **Ablations:** each background-separator (2, 5) with and without a
   floor-remover (1, 3, 4) in front; Method 5 with nearest-vs-largest cluster
   selection; Method 2's window width sensitivity.
4. **Cross-route comparison:** run the winning 2D recipe
   (`foreground_isolation_2d.md`) and the winning 3D recipe on identical frames —
   this is the "isolate-in-2D-then-deproject vs. deproject-then-isolate-in-3D"
   swap point of the pipeline's benchmark matrix.
5. **Report:** as in the 2D protocol, highlight cases where a crude foreground
   set still yields a near-perfect distance — for the benchmark's purpose that
   finding is as valuable as a clean segmentation.
