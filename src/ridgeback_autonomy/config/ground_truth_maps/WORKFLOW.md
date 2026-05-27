# Ground-Truth Map Generation Workflow

Quick reference for generating hand-authored ground-truth occupancy maps using Approach 1 (SLAM-derived with manual cleanup).

## Quick Start (per world)

### Phase 1: Data Collection (10–15 min)

**Terminal 1: Start the manual mapping session**
```bash
cd /home/vili/Documents/DyNAMO/DyNAMO-Ridgeback
source install/local_setup.bash
ros2 launch ridgeback_autonomy manual_mapping.launch.py world:=warehouse
```

**Terminal 2: Monitor in RViz**
- Gazebo window will open on the left (3D simulation)
- RViz window will open on the right (SLAM map visualization)
- Keyboard controls in Gazebo: Arrow keys or `w/a/s/d` for movement, `q/e` for rotation

**What to do:**
1. Drive the robot close to all walls (1st pass)
   - Traces wall boundaries
   - Exposes doorways and corridors
   - Builds the map skeleton
2. Drive through all open spaces (2nd pass)
   - Fills interior regions
   - Ensures no missed rooms or corridors
   - Watch the RViz map growing in real-time
3. Total time: 8–12 minutes typically

### Phase 2: Save the Map (1 min)

**Terminal 3: Save SLAM map to home directory**
```bash
ros2 run nav2_map_server map_saver_cli -f ground_truth_warehouse
```

Output goes to: `$HOME/ground_truth_warehouse.pgm` and `$HOME/ground_truth_warehouse.yaml`

### Phase 3: Move & Organize (1 min)

```bash
# From workspace root
cd /home/vili/Documents/DyNAMO/DyNAMO-Ridgeback
mv ~/ground_truth_warehouse.* src/ridgeback_autonomy/config/ground_truth_maps/
```

Or use the helper:
```bash
cd src/ridgeback_autonomy/config/ground_truth_maps
bash capture_ground_truth.sh warehouse --finalize
```

### Phase 4: Manual Cleanup (10–20 min)

Clean up SLAM artifacts in GIMP:

```bash
cd src/ridgeback_autonomy/config/ground_truth_maps
gimp ground_truth_warehouse.pgm &
```

**Cleanup checklist:**
- [ ] Find gray (unknown) pixels inside rooms → fill with white (free)
- [ ] Remove speckle noise on walls → despeckle or paint
- [ ] Erase phantom walls in doorways → fill with white
- [ ] Verify obstacles outside the robot's reach are black or gray
- [ ] Save when done (Ctrl+S in GIMP)

**GIMP Tips:**
- Use **Filters > Enhance > Despeckle** for noise
- Use **paintbrush** (white/black) for manual touch-ups
- Use **selection tools** (fuzzy select) to fill regions quickly
- Export as PNG internally, then **File > Export As** as `.pgm`

### Phase 5: Verify .yaml Metadata (2 min)

Check `src/ridgeback_autonomy/config/ground_truth_maps/ground_truth_warehouse.yaml`:

```bash
cat src/ridgeback_autonomy/config/ground_truth_maps/ground_truth_warehouse.yaml
```

Verify:
- `image:` points to the correct `.pgm` file
- `resolution: 0.05` matches SLAM param  
- `origin:` looks reasonable (should be negative for most worlds, e.g., `[-10.0, -15.0, 0.0]`)

If `origin` seems wrong:
1. Open SLAM-generated map + ground-truth in GIMP side-by-side
2. Estimate pixel offset between obstacles
3. Convert pixel offset to world coords: `delta_pixels * resolution`
4. Adjust `origin` by that offset and retry

### Phase 6: Validate (5–20 min)

Run a short exploration sweep and measure coverage:

```bash
# Terminal: Run exploration
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py \
    world:=warehouse \
    explorer:=frontier_explorer \
    time_limit:=900

# (In another terminal, after exploration finishes)
# Save the SLAM map it generated
ros2 run nav2_map_server map_saver_cli -f test_coverage
```

**Analyze coverage:**
```bash
cd src/ridgeback_autonomy/config/ground_truth_maps
python3 analyze_coverage.py ~/test_coverage.pgm ground_truth_warehouse.pgm \
    --output coverage_visualization.png
```

**Expected result:**
- ✓ Coverage ≥ 75% → Ground-truth is valid
- ✗ Coverage < 75% → Check:
  - Is SLAM resolution correct in `.yaml`?
  - Is `origin` off (walls misaligned)?
  - Are there unreachable areas marked as free?

## World-Specific Notes

### warehouse
- **Complexity**: Low–Medium (open floor with racks)
- **Teleoperation time**: ~8 min
- **Expected cleanup**: Minimal (open spaces, fewer walls)
- **Coverage target**: >85% (open geometry means most space is reachable)

### office
- **Complexity**: Medium (cubicles, hallways, interior walls)
- **Teleoperation time**: ~10 min
- **Expected cleanup**: Moderate (shadows from cubicles, doorways)
- **Coverage target**: >75%

### mock_hospital
- **Complexity**: High (interconnected rooms, corridors, many walls)
- **Teleoperation time**: ~12 min
- **Expected cleanup**: Moderate–High (many doorways, wall artifacts)
- **Coverage target**: >75%

## Troubleshooting

| Issue | Solution |
|-------|----------|
| **Robot stuck in teleoperation** | SLAM loop-closure failure. Back up to known area; map should re-converge. Retry slowly. |
| **Map has huge gaps or discontinuities** | SLAM drift. Start fresh with slower, more careful teleoperation. |
| **Walls misaligned in validation** | Origin value wrong. Overlay SLAM + ground-truth in GIMP; estimate pixel offset and adjust origin. |
| **Too much noise in final map** | Use GIMP Despeckle + Median Blur filters. Manual touch-up with paintbrush. |
| **Coverage consistently < 50%** | Ground-truth map setup issue. Check: resolution matches SLAM, origin correct, unreachable areas are black. |

## File Structure

```
src/ridgeback_autonomy/config/ground_truth_maps/
├── README.md                          (Full documentation)
├── WORKFLOW.md                        (This file)
├── capture_ground_truth.sh            (Helper script for map organization)
├── analyze_coverage.py                (Coverage analyzer tool)
│
├── mock_hospital.pgm                  (Generated by manual teleoperation + GIMP cleanup)
├── mock_hospital.yaml                 (ROS map metadata)
│
├── office.pgm                         (Generated)
├── office.yaml                        (ROS map metadata)
│
├── warehouse.pgm                      (Generated)
└── warehouse.yaml                     (ROS map metadata)
```

## Checklist: All Worlds Done

- [ ] **warehouse** teleoperated, cleaned, validated
- [ ] **office** teleoperated, cleaned, validated
- [ ] **mock_hospital** teleoperated, cleaned, validated
- [ ] All `.pgm` + `.yaml` pairs committed to git
- [ ] Coverage: each world > 75% on validation run
- [ ] Update README.md status for each world

## Next: Integration with Sweeps

Once all ground-truth maps are ready:

1. **Coverage metric**: Compute `coverage(explorer, world) = (discovered cells) / (reachable cells from ground-truth)`
2. **Benchmark runner**: Add ground-truth path to sweep config; load at runtime
3. **Metric reporting**: Include coverage % in sweep results alongside time, distance, exploration patterns

Example integration in benchmark config:
```yaml
metrics:
  - coverage_threshold: 0.75
    ground_truth_map: ground_truth_warehouse.yaml
```

---

For detailed documentation, see [README.md](README.md).
