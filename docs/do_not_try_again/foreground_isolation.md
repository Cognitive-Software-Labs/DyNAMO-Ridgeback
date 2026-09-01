# Foreground-isolation candidates

Unimplemented research, not the runtime registry. The method numbers within
each dimensional section retain the original catalogue numbering so its
cross-references remain meaningful. Qualitative speed/quality claims are
hypotheses from the original literature survey, not repository measurements;
citations are retained research leads, not a newly verified literature review.

Implemented 2D baselines live in
[projective ranging](../target_localization/projective_ranging.md#implemented-2d-recipes),
and implemented 3D baselines live in
[euclidean reconstruction](../target_localization/euclidean_reconstruction.md#implemented-3d-recipes).
No candidate here is an automatic backlog commitment.
[Occlusion characterization](../BACKLOG.md#occlusion-characterization)
must establish the failure before a recovery algorithm is selected.

## 2D candidates

Methods 1 and 2 are the implemented nearest-mode and Otsu baselines in the 2D
reference. Methods 3–7 below are not registered recipes.

### 3. GMM / k-Means Clustering on Depth Values

**Idea.** The multi-modal generalization of Otsu. Fit a k-component Gaussian Mixture Model (via EM) or run 1D k-means on the ROI's depth values; take the cluster with the smallest mean depth as foreground. Handles the "wall + table + floor all visible in the box" case that breaks a single threshold.

**Pros**
- Handles arbitrary numbers of background depth layers.
- Still very fast in 1D (the feature space is just depth).
- GMM gives soft assignments, which can feed directly into a CRF unary (Method 7) — a natural stacking experiment.

**Cons**
- **k must be chosen** (or model-selected via BIC/AIC, which adds cost and complexity).
- EM is sensitive to initialization; degenerate fits happen on noisy RealSense depth with many invalid pixels.
- Like Methods 1–2, purely value-based: no spatial reasoning.

**Paper access.**
The GMM-with-iterative-refinement machinery is exactly what GrabCut formalizes for segmentation, so the standard citation is:
C. Rother, V. Kolmogorov, A. Blake, *"'GrabCut': Interactive Foreground Extraction using Iterated Graph Cuts,"* ACM SIGGRAPH / ACM Transactions on Graphics, vol. 23, no. 3, pp. 309–314, 2004.
- DOI: `10.1145/1015706.1015720` (ACM Digital Library)
- Freely available from Microsoft Research: search "GrabCut Microsoft Research 2004 PDF".
For plain k-means, the classical references are S. Lloyd (1982, IEEE Trans. Information Theory, `10.1109/TIT.1982.1056489`) and J. MacQueen (1967, Berkeley Symposium — open access via Project Euclid).

---

### 4. Mean-Shift Mode Seeking on the Depth Distribution

**Idea.** The nonparametric alternative to GMM: rather than fixing k, find the modes of the depth density directly by kernel density estimation and gradient ascent. Take the pixels belonging to the nearest mode's basin of attraction as foreground. This is the *principled* version of Method 1 — the kernel bandwidth replaces bin width as the single tuning knob.

**Pros**
- No k to choose; the number of modes emerges from the data.
- Provably converges to the modes of the underlying density.
- Only one parameter (bandwidth / analysis resolution).
- Robust to non-Gaussian, arbitrarily shaped depth clusters.

**Cons**
- Slower than Otsu/k-means (iterative per seed point), though 1D depth keeps it manageable.
- Bandwidth choice still matters: too small → over-fragmented modes; too large → subject and near background merge.
- Again purely value-based.

**Paper access.**
D. Comaniciu, P. Meer, *"Mean Shift: A Robust Approach Toward Feature Space Analysis,"* IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 24, no. 5, pp. 603–619, 2002.
- DOI: `10.1109/34.1000236` (IEEE Xplore)
- Author copy freely hosted by Rutgers (Peter Meer's page); searching the exact title finds the PDF immediately.

---

### 5. Seeded Region Growing / Depth Flood Fill

**Idea.** Exploit **spatial connectivity** instead of depth value alone. Seed at the ROI center (or at the nearest-mode pixel from Method 1/4), then grow outward, accepting neighboring pixels whose depth is within a tolerance of the region. Growth stops at depth discontinuities — which is exactly the subject/background boundary. OpenCV's `floodFill` with a depth tolerance is a practical implementation.

**Pros**
- The only classical method here that produces a **spatially connected** mask by construction — no depth-coincident speckle from elsewhere in the box.
- Most likely to win when background objects sit at a *similar depth* to the subject but are not physically connected to it.
- Fast, robust, essentially tuning-free apart from the depth tolerance.

**Cons**
- Seed placement matters: if the ROI center lands on background (subject off-center in the box, or a hole in the depth map), the wrong region grows.
- Depth holes (invalid pixels, common on dark/reflective G1 surfaces) can block growth and fragment the mask.
- Result is order-dependent in the original formulation (later variants fix this).

**Paper access.**
R. Adams, L. Bischof, *"Seeded Region Growing,"* IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 16, no. 6, pp. 641–647, 1994.
- DOI: `10.1109/34.295913` (IEEE Xplore)
- PDF widely mirrored; the improved order-independent variant is Mehnert & Jackway, Pattern Recognition Letters 18 (1997) 1065–1071.

---

### 6. GrabCut Seeded from the Rectangle (RGB / depth-as-intensity / RGB-D)

**Idea.** The rectangle mask is literally GrabCut's native input format. The algorithm models foreground and background color distributions as GMMs, builds a Markov random field over pixel labels, and applies iterative graph-cut (min-cut) optimization until the segmentation converges. Three interesting variants to compare against each other:
- **RGB GrabCut** — run on the color frame, then sample the aligned depth under the resulting mask.
- **Depth-as-intensity GrabCut** — render the depth ROI as a grayscale image and run GrabCut on that.
- **RGB-D graph cut** — the depth-aware extension that combines a color-space (Euclidean) distance and a depth-space (geodesic) distance in a single unified graph-cut framework.

**Pros**
- Global optimization with spatial smoothness built in — clean, coherent boundaries.
- Rectangle initialization is exactly the input the pipeline already produces; zero adaptation needed.
- The three input variants form a self-contained mini-ablation on the value of depth vs. color cues.

**Cons**
- Iterative and considerably slower than Methods 1–5 (typically tens of ms to hundreds of ms per ROI, iteration-count dependent).
- Color-based GMMs struggle when subject and background have similar colors (gray robot against gray lab walls — a realistic risk with the G1).
- Can catastrophically eat into the foreground when initialization is poor (known failure mode).

**Paper access.**
C. Rother, V. Kolmogorov, A. Blake, *"'GrabCut': Interactive Foreground Extraction using Iterated Graph Cuts,"* ACM SIGGRAPH 2004 / ACM TOG 23(3), pp. 309–314.
- DOI: `10.1145/1015706.1015720`; free PDF via Microsoft Research.
RGB-D extension: Ge, Zhang, et al., *"Interactive foreground segmentation and shape reconstruction from RGBD images"* (Multimedia Tools and Applications, 2019) — find via the title on SpringerLink or ResearchGate; it integrates color and geodesic depth cues in one Graph Cut energy.
Implementation: `cv2.grabCut(...)` in OpenCV, mode `GC_INIT_WITH_RECT`.

---

### 7. Dense (Fully-Connected) CRF Refinement

**Idea.** Not a standalone isolator but a **refinement layer**: take a coarse unary labeling (the output of Method 2, 3, or 4 — or even Method 1) and run a fully-connected Conditional Random Field over all ROI pixels, with Gaussian pairwise potentials on position + color (and optionally depth). Efficient mean-field inference makes the fully-connected graph tractable: each pixel's label is iteratively updated by aggregating messages from **all** other pixels, snapping the boundary onto true image edges.

**Pros**
- Composable: report every value-based method with/without CRF — a clean extra dimension in the comparison, cheaply obtained.
- Dramatically improves boundary quality of coarse/blocky masks, which matters if boundary pixels contaminate the depth median.
- A candidate implementation is `pydensecrf`; maintenance and compatibility need checking before adoption.

**Cons**
- Adds runtime on top of whatever produced the unary (typically tens of ms for an ROI).
- Several kernel weights/bandwidths to tune (appearance kernel, smoothness kernel).
- If the unary is badly wrong, the CRF confidently refines the wrong answer — it fixes boundaries, not gross errors.

**Paper access.**
P. Krähenbühl, V. Koltun, *"Efficient Inference in Fully Connected CRFs with Gaussian Edge Potentials,"* NeurIPS 2011.
- arXiv: `arXiv:1210.5644` — free PDF at `https://arxiv.org/abs/1210.5644`
- Project page with code and unaries: Stanford graphics group ("densecrf").

---

### Appearance-only control: segmentation, not another depth recipe

Box-prompted SAM is already implemented as a mask producer. Its model choice,
batching, mask selection, and rejection rules belong to
[Segmentation](../target_localization/segmentation.md); do not duplicate them here.
Comparing `box` and `silhouette` gates is an appearance-versus-depth-isolation
experiment, not a missing SAM isolation implementation.

Retained literature reference: A. Kirillov et al., *Segment Anything*, ICCV
2023, pp. 4015–4026, arXiv `2304.02643`.

## 3D candidates

Methods 1, 2, and 2b are the implemented height crop, percentile range band,
and nearest-mode band in the 3D reference. Methods 3–9 below are not registered.
Learned 3D segmentation is a research comparison, not an established upper bound.

### 3. RANSAC Plane Removal

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

### 4. Normal-Based Filtering

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

### 5. Euclidean Cluster Extraction / DBSCAN

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

### 6. Region Growing with Smoothness Constraint

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

### 7. Min-Cut Point-Cloud Segmentation

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

### 8. Supervoxel + Convexity Segmentation (LCCP)

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

### 9. Learned 3D segmentation candidate

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

## Evaluation protocol

1. Fix scenes and input frames across candidates. Include distance, viewpoint,
   clutter, touching surfaces, and partial/full occlusion. Distance ground truth
   comes from benchmark poses; pixel/point IoU needs suitable instance labels.
2. Measure published planar distance/position error and misses/coverage, not
   just a depth median or a visually appealing foreground set. Report foreground
   count stability, runtime per ROI and per frame, memory, and target hardware.
3. Compare separators with/without a floor-remover; test seed/nearest/largest
   selection failures explicitly. Compare value-based methods with/without CRF,
   and RGB, depth, and RGB-D GrabCut variants only if selected for investigation.
4. For placement experiments, hold selection policy constant where possible:
   isolate in image space before deprojection versus isolate points afterward.
   Optical Z and Euclidean camera range are different statistics off-axis.
5. Compare the shipped box/silhouette paths as controls. Report cases where mask
   quality and distance quality disagree, and do not equate missing output with
   accurate output. Record parameters, input provenance, and baseline revision.

The active comparison task is [Isolation validation](../BACKLOG.md#isolation-validation).
Literature candidates require a separate selection decision before implementation.
