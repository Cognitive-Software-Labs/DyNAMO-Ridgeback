#!/bin/bash
# Helper script to capture and organize ground-truth maps
# Usage: ./capture_ground_truth.sh <world_name>
# Example: ./capture_ground_truth.sh warehouse

if [ -z "$1" ]; then
    echo "Usage: $0 <world_name>"
    echo "Example: $0 warehouse"
    echo "Supported worlds: warehouse, office, hospital (mock_hospital)"
    exit 1
fi

WORLD=$1
WORKSPACE_ROOT="$(git rev-parse --show-toplevel)"
GROUND_TRUTH_DIR="${WORKSPACE_ROOT}/src/ridgeback_autonomy/sim/ground_truth_maps"

echo "=== Ground-Truth Map Capture Helper ==="
echo "World: $WORLD"
echo "Target directory: $GROUND_TRUTH_DIR"
echo ""
echo "Steps:"
echo "1. In Terminal 1, run:"
echo "   ros2 launch ridgeback_autonomy manual_mapping.launch.py world:=$WORLD"
echo ""
echo "2. Teleoperate the robot through the entire reachable space"
echo "   - Pass 1: Drive close to walls (boundary mapping)"
echo "   - Pass 2: Drive through open areas (interior fill)"
echo "   - Ensure every room/corridor/passage is traversed"
echo ""
echo "3. When done, in Terminal 2, run:"
echo "   ros2 run nav2_map_server map_saver_cli -f ground_truth_${WORLD}"
echo ""
echo "4. Then run this script to move and organize the map:"
echo "   $0 $WORLD --finalize"
echo ""

if [ "$2" = "--finalize" ]; then
    echo "=== Finalizing map capture ==="
    
    # Check if map files exist in home directory
    PGM_FILE="$HOME/ground_truth_${WORLD}.pgm"
    YAML_FILE="$HOME/ground_truth_${WORLD}.yaml"
    
    if [ ! -f "$PGM_FILE" ] || [ ! -f "$YAML_FILE" ]; then
        echo "ERROR: Map files not found in $HOME"
        echo "Expected:"
        echo "  - $PGM_FILE"
        echo "  - $YAML_FILE"
        exit 1
    fi
    
    echo "Found map files:"
    echo "  - $(ls -lh $PGM_FILE | awk '{print $9, $5}')"
    echo "  - $(ls -lh $YAML_FILE | awk '{print $9, $5}')"
    echo ""
    
    # Copy to ground_truth_maps directory, stripping the "ground_truth_" prefix
    # so filenames match the .yaml image: references (e.g. warehouse.pgm, not ground_truth_warehouse.pgm)
    cp "$PGM_FILE" "$GROUND_TRUTH_DIR/${WORLD}.pgm"

    # The pre-existing .yaml already has the correct image: reference and origin estimate.
    # Don't overwrite it with the one produced by map_saver_cli (which has a temp name and
    # possibly a different origin guess).  Just confirm it is present.
    if [ ! -f "$GROUND_TRUTH_DIR/${WORLD}.yaml" ]; then
        cp "$YAML_FILE" "$GROUND_TRUTH_DIR/${WORLD}.yaml"
        echo "✓ Copied new ${WORLD}.yaml (verify origin manually)"
    else
        echo "✓ Kept existing ${WORLD}.yaml (origin already configured)"
    fi

    echo "✓ PGM copied to: $GROUND_TRUTH_DIR/${WORLD}.pgm"
    echo ""
    echo "Next steps:"
    echo "1. Review and cleanup in GIMP:"
    echo "   gimp $GROUND_TRUTH_DIR/${WORLD}.pgm"
    echo ""
    echo "2. Verify .yaml metadata:"
    echo "   cat $GROUND_TRUTH_DIR/${WORLD}.yaml"
    echo ""
    echo "3. Validate with exploration run:"
    echo "   ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=$WORLD"
    echo ""
fi
