#!/bin/bash
# Run all baselines on a ScanNet++ scene
# Usage: ./run_baselines.sh <scene_path> <output_dir>

set -e

SCENE=${1:?"Usage: $0 <scene_path> <output_dir>"}
OUTPUT=${2:-"./results/$(basename $SCENE)"}
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SIM_DIR="$(dirname "$SCRIPT_DIR")"

echo "╔══════════════════════════════════════════╗"
echo "║   DV-SLAM Baseline Comparison Suite      ║"
echo "╚══════════════════════════════════════════╝"
echo "Scene: $SCENE"
echo "Output: $OUTPUT"
echo ""

mkdir -p "$OUTPUT"

# ============================================================
# 1. DV-SLAM (ours) — all presets
# ============================================================
echo "━━━ [1/6] DV-SLAM ━━━"
DV_SIM="$SIM_DIR/build/dv_sim"
if [ -x "$DV_SIM" ]; then
    for preset in full noConf noTLS sparse100 dense3000 imuOnly; do
        if [ ! -f "$OUTPUT/dvsim_$preset/report.txt" ]; then
            echo "  Running: $preset"
            $DV_SIM "$SCENE" --preset "$preset" --output "$OUTPUT/dvsim_$preset" 2>&1 | tail -3
        else
            echo "  ✅ $preset done"
        fi
    done

    # Naive ICP (no IMU)
    if [ ! -f "$OUTPUT/dvsim_naiveICP/report.txt" ]; then
        echo "  Running: naiveICP"
        $DV_SIM "$SCENE" --preset "full" --output "$OUTPUT/dvsim_naiveICP" 2>&1 | tail -3
    fi
else
    echo "  ⚠️  dv_sim not built. Run: cd $SIM_DIR && mkdir build && cd build && cmake .. && make -j8"
fi

# ============================================================
# 2. ARKit Baseline
# ============================================================
echo ""
echo "━━━ [2/6] ARKit Baseline ━━━"
if [ ! -f "$OUTPUT/arkit/report.txt" ]; then
    python3 "$SCRIPT_DIR/baseline_arkit.py" "$SCENE" --output "$OUTPUT/arkit" 2>&1 | tail -5
else
    echo "  ✅ done"
fi

# ============================================================
# 3. KISS-ICP
# ============================================================
echo ""
echo "━━━ [3/6] KISS-ICP ━━━"
if command -v kiss_icp_pipeline &> /dev/null; then
    if [ ! -f "$OUTPUT/kiss_icp/report.txt" ]; then
        echo "  Running KISS-ICP..."
        # KISS-ICP can process point clouds directly
        # Need to generate per-frame PLY/PCD from depth first
        python3 "$SCRIPT_DIR/baseline_kiss_icp.py" "$SCENE" --output "$OUTPUT/kiss_icp" 2>&1 | tail -5
    else
        echo "  ✅ done"
    fi
else
    echo "  ⚠️  KISS-ICP not installed. Run: pip install kiss-icp"
fi

# ============================================================
# 4. FAST-LIO
# ============================================================
echo ""
echo "━━━ [4/6] FAST-LIO ━━━"
FAST_LIO_DIR="$SIM_DIR/baselines/FAST_LIO"
if [ -d "$FAST_LIO_DIR" ]; then
    echo "  FAST-LIO found at $FAST_LIO_DIR"
    echo "  ⚠️  Requires ROS + rosbag. Run manually:"
    echo "    1. python3 $SCRIPT_DIR/scannetpp_to_rosbag.py $SCENE --output /tmp/scene.bag"
    echo "    2. roslaunch fast_lio mapping_*.launch"
    echo "    3. rosbag play /tmp/scene.bag"
else
    echo "  ⚠️  Not cloned. Run: cd $SIM_DIR/baselines && git clone https://github.com/hku-mars/FAST_LIO"
fi

# ============================================================
# 5. FAST-LIVO2
# ============================================================
echo ""
echo "━━━ [5/6] FAST-LIVO2 ━━━"
FAST_LIVO2_DIR="$SIM_DIR/baselines/FAST-LIVO2"
if [ -d "$FAST_LIVO2_DIR" ]; then
    echo "  FAST-LIVO2 found at $FAST_LIVO2_DIR"
    echo "  ⚠️  Requires ROS + rosbag. Run manually:"
    echo "    1. python3 $SCRIPT_DIR/scannetpp_to_rosbag.py $SCENE --output /tmp/scene.bag"
    echo "    2. roslaunch fast_livo mapping_*.launch"
    echo "    3. rosbag play /tmp/scene.bag"
else
    echo "  ⚠️  Not cloned. Run: cd $SIM_DIR/baselines && git clone https://github.com/hku-mars/FAST-LIVO2"
fi

# ============================================================
# 6. RTAB-Map
# ============================================================
echo ""
echo "━━━ [6/6] RTAB-Map ━━━"
if command -v rtabmap &> /dev/null; then
    if [ ! -f "$OUTPUT/rtabmap/report.txt" ]; then
        echo "  Running RTAB-Map offline..."
        python3 "$SCRIPT_DIR/baseline_rtabmap.py" "$SCENE" --output "$OUTPUT/rtabmap" 2>&1 | tail -5
    else
        echo "  ✅ done"
    fi
else
    echo "  ⚠️  RTAB-Map not installed."
    echo "    ROS: sudo apt install ros-noetic-rtabmap-ros"
    echo "    Standalone: https://github.com/introlab/rtabmap"
fi

# ============================================================
# Evaluate all
# ============================================================
echo ""
echo "━━━ Evaluation ━━━"
python3 "$SCRIPT_DIR/evaluate.py" "$OUTPUT" 2>&1

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║   All baselines complete!                 ║"
echo "╚══════════════════════════════════════════╝"
