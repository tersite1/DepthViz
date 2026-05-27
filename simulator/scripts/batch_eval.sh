#!/bin/bash
# Batch evaluation script for DV-SLAM on ScanNet++
# Usage: ./batch_eval.sh <scannetpp_data_dir> <output_dir>
#
# Example:
#   ./batch_eval.sh /data/scannetpp/data ./results

set -e

SCANNETPP_DIR=${1:?"Usage: $0 <scannetpp_data_dir> <output_dir>"}
OUTPUT_DIR=${2:-"./results"}
DV_SIM=${3:-"./build/dv_sim"}

# Presets to run
PRESETS=("full" "noConf" "noTLS" "sparse100" "dense3000" "imuOnly")

# Find all scenes with iPhone data
SCENES=$(find "$SCANNETPP_DIR" -maxdepth 1 -mindepth 1 -type d | sort | head -20)

echo "╔══════════════════════════════════════════╗"
echo "║   DV-SLAM Batch Evaluation               ║"
echo "╚══════════════════════════════════════════╝"
echo "Data: $SCANNETPP_DIR"
echo "Output: $OUTPUT_DIR"
echo "Presets: ${PRESETS[*]}"
echo ""

for scene_dir in $SCENES; do
    scene_id=$(basename "$scene_dir")

    # Check if iPhone data exists
    if [ ! -f "$scene_dir/iphone/pose_intrinsic_imu.json" ]; then
        echo "⏭  Skipping $scene_id (no iPhone data)"
        continue
    fi

    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Scene: $scene_id"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    for preset in "${PRESETS[@]}"; do
        out="$OUTPUT_DIR/$scene_id/$preset"
        if [ -f "$out/report.txt" ]; then
            echo "  ✅ $preset — already done"
            continue
        fi

        echo "  🔄 Running $preset..."
        $DV_SIM "$scene_dir" --preset "$preset" --output "$OUTPUT_DIR/$scene_id" 2>&1 | tail -5
        echo ""
    done
done

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║   Batch evaluation complete!              ║"
echo "╚══════════════════════════════════════════╝"
echo "Results: $OUTPUT_DIR"
