# Projective ranging — Foreground Isolation Methods (2D Depth-Image Route)

**Context.** In projective ranging of the pipeline, an aligned depth frame (1:1 with RGB) and a binary mask arrive through the mask interface. When the mask is *tight* (instance segmentation), a robust median over the masked depth pixels is sufficient. When the mask is a *rectangle* (rasterized detection box), the ROI contains both the foreground subject (the static G1) and background pixels, so a **foreground isolation** step must separate them **purely in the 2D depth image** — no back-projection into a point cloud (that is euclidean reconstruction's job).

The problem reduces to separating the depth distribution inside the rectangle: the G1 forms a near mode, the background forms one or more farther modes, plus invalid/zero-depth pixels. The methods below span the design space from trivial histogram tricks to learned foundation models, so they can be benchmarked against each other on:

1. **Mask quality** (IoU against hand-labeled ground truth)
2. **Distance error** (error in the final median depth — the metric that actually matters for projective ranging)
3. **Runtime per frame** (this runs live on the Ridgeback)

They differ along two axes worth keeping in mind when analyzing results:

- **Value-based** (2, 3, 4) vs. **connectivity-based** (5) vs. **joint/global** (6, 7, 8)
- **Depth-only** vs. **RGB-only** vs. **RGB-D** input

**Implemented so far** (`perception/target_localization/core/isolation_2d.py`): Method 1
`nearest_mode_histogram` (the default recipe) and Method 2 `otsu`. The
nearest-mode recipe applies a 5% significance floor — a bin must hold at least
5% of the valid masked pixels to count as the near peak — with a
dispersed-distribution fallback to the nearest non-empty bin (not the global
mode, which at range would lock onto the background wall). The rest of the
catalogue below remains for benchmarking against these two.

---

## 1. Nearest-Mode Histogram Selection (Baseline)

**Idea.** Build a histogram of the depth values inside the rectangle, locate the closest significant peak, and keep all pixels within a fixed band around it. The assumption: the subject is the nearest coherent surface in the box.

**Pros**
- Trivial to implement (a few lines of NumPy); essentially zero runtime cost.
- No training, no models, fully deterministic and interpretable.
- Works surprisingly well when the subject really is the nearest object and the box is reasonably tight.

**Cons**
- Fails when clutter sits *in front of* the subject (e.g., the Ridgeback's own arm or a chair edge entering the box) — the nearest mode is then the wrong one.
- Band width and histogram bin width are hand-tuned magic numbers.
- Ignores spatial structure entirely: any pixel at the right depth is accepted, even disconnected noise.

**Paper access.** This is a folk method with no canonical publication — it serves as the baseline that every other method must justify its complexity against. Document it as "nearest-depth-mode heuristic" in the comparison.

---

## 2. Otsu Thresholding on the ROI Depth Histogram

**Idea.** Treat the depth values inside the rectangle as a bimodal distribution (subject near, background far) and automatically pick the threshold that maximizes between-class variance. Everything below the threshold = foreground.

**Pros**
- Fully automatic and parameter-free — no bin widths or bands to tune.
- Extremely fast (one histogram pass + one 1D search).
- A principled step up from the fixed nearest-depth band; a very strong "cheap" entry in the comparison.

**Cons**
- Assumes **bimodality**. If the background inside the box has multiple depth layers (wall + table + floor), the single threshold lands in the wrong place.
- Purely value-based: no spatial coherence, so speckle at foreground depth is accepted.
- Sensitive to the proportion of foreground vs. background pixels in the box (known weakness for very small or very large objects relative to the ROI).

**Paper access.**
N. Otsu, *"A Threshold Selection Method from Gray-Level Histograms,"* IEEE Transactions on Systems, Man, and Cybernetics, vol. SMC-9, no. 1, pp. 62–66, 1979.
- DOI: `10.1109/TSMC.1979.4310076` (IEEE Xplore; accessible via most university subscriptions)
- Widely mirrored as PDF — searching the DOI or title finds open copies easily.

---

## 3. GMM / k-Means Clustering on Depth Values

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

## 4. Mean-Shift Mode Seeking on the Depth Distribution

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

## 5. Seeded Region Growing / Depth Flood Fill

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

## 6. GrabCut Seeded from the Rectangle (RGB / depth-as-intensity / RGB-D)

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

## 7. Dense (Fully-Connected) CRF Refinement

**Idea.** Not a standalone isolator but a **refinement layer**: take a coarse unary labeling (the output of Method 2, 3, or 4 — or even Method 1) and run a fully-connected Conditional Random Field over all ROI pixels, with Gaussian pairwise potentials on position + color (and optionally depth). Efficient mean-field inference makes the fully-connected graph tractable: each pixel's label is iteratively updated by aggregating messages from **all** other pixels, snapping the boundary onto true image edges.

**Pros**
- Composable: report every value-based method with/without CRF — a clean extra dimension in the comparison, cheaply obtained.
- Dramatically improves boundary quality of coarse/blocky masks, which matters if boundary pixels contaminate the depth median.
- Well-maintained reference implementation exists (`pydensecrf`).

**Cons**
- Adds runtime on top of whatever produced the unary (typically tens of ms for an ROI).
- Several kernel weights/bandwidths to tune (appearance kernel, smoothness kernel).
- If the unary is badly wrong, the CRF confidently refines the wrong answer — it fixes boundaries, not gross errors.

**Paper access.**
P. Krähenbühl, V. Koltun, *"Efficient Inference in Fully Connected CRFs with Gaussian Edge Potentials,"* NeurIPS 2011.
- arXiv: `arXiv:1210.5644` — free PDF at `https://arxiv.org/abs/1210.5644`
- Project page with code and unaries: Stanford graphics group ("densecrf").

---

## 8. Segment Anything (SAM), Box-Prompted — Learned Upper Bound

**Idea.** Prompt single-image SAM with the detection box. SAM's architecture — a heavy image encoder, a light prompt encoder, and a fast mask decoder — returns a valid segmentation mask for any prompt, and because image and prompt embeddings are computed separately, one image embedding can be reused across multiple prompts on the same frame. Depth is then sampled under the returned mask.

**Pros**
- Highest expected mask quality of all methods here; handles thin structures (G1 limbs) that classical methods lose.
- Zero training or tuning; the box prompt is exactly what the detection component already emits.
- Serves as a **control experiment**: SAM uses *no depth at all*, so it quantifies how much of the isolation problem is solvable from appearance alone — which frames the value of the depth-based methods.

**Cons**
- By far the heaviest: the ViT image encoder dominates runtime; even the small variants cost orders of magnitude more than Methods 1–5. Real-time on embedded hardware requires distilled variants (MobileSAM, EfficientSAM, FastSAM) — a legitimate follow-up axis.
- Ignores depth, so it can include background regions that *look* attached to the subject (shadows, similarly-textured surfaces behind the G1).
- Ambiguity: SAM returns multiple candidate masks; selecting the right one (whole robot vs. one limb) needs a heuristic (e.g., highest-score mask, or the mask whose depth distribution is most compact).

**Paper access.**
A. Kirillov, E. Mintun, N. Ravi, H. Mao, C. Rolland, L. Gustafson, T. Xiao, S. Whitehead, A. C. Berg, W.-Y. Lo, P. Dollár, R. Girshick, *"Segment Anything,"* ICCV 2023, pp. 4015–4026.
- arXiv: `arXiv:2304.02643` — free PDF at `https://arxiv.org/abs/2304.02643`
- Open-access CVF version: openaccess.thecvf.com (ICCV 2023).
- Code + weights: `https://github.com/facebookresearch/segment-anything`

---

## Summary Comparison

| # | Method | Cue | Spatial coherence | Speed | Tuning burden | Depth used | Key risk |
|---|--------|-----|-------------------|-------|---------------|-----------|----------|
| 1 | Nearest-mode histogram | Depth value | No | ★★★★★ | Bin + band width | Yes | Clutter in front of subject |
| 2 | Otsu threshold | Depth value | No | ★★★★★ | None | Yes | Multi-layer background |
| 3 | GMM / k-means | Depth value | No | ★★★★☆ | k, init | Yes | Degenerate fits, k choice |
| 4 | Mean-shift modes | Depth value | No | ★★★☆☆ | Bandwidth | Yes | Mode merging/splitting |
| 5 | Seeded region growing | Depth + connectivity | **Yes** | ★★★★☆ | Tolerance, seed | Yes | Bad seed, depth holes |
| 6 | GrabCut (3 variants) | Color / depth / both, global | Yes | ★★☆☆☆ | Iterations | Optional | Similar FG/BG colors |
| 7 | Dense CRF (refiner) | Color + position pairwise | Yes | ★★★☆☆ | Kernel params | Optional | Garbage-in, garbage-out |
| 8 | SAM (box prompt) | Learned appearance | Yes | ★☆☆☆☆ | Mask selection | No | Runtime; depth-blind |

## Recommended Evaluation Protocol

1. **Ground truth:** hand-label tight masks on a sample of frames (varying distance, viewpoint, clutter).
2. **Metrics per method:** mask IoU, absolute error of the median foreground depth vs. ground-truth median, runtime per ROI.
3. **Ablations:** each of Methods 1–5 with and without CRF refinement (Method 7); GrabCut in its three input variants; SAM as the appearance-only control.
4. **Report:** since the downstream output is one distance number, expect and highlight cases where an "ugly" mask still yields a near-perfect median depth — for projective ranging, that finding is as valuable as a pretty mask.