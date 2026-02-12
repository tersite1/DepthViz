#!/usr/bin/env python3
"""
DV-SLAM RA-L Benchmark Runner
==============================

Full evaluation pipeline for comparing DV-SLAM against COLMAP and ARKit
on ARKitScenes sequences. Generates LaTeX tables and trajectory plots
for the RA-L submission.

Usage:
  python3 run_benchmark.py \
      --arkitscenes_dir ~/ARKitScenes/raw/Training \
      --sequences 47331606 47332006 47332308 47332687 47333015 \
      --build_dir ./build \
      --output_dir ./results

Requirements:
  - DV-SLAM built (dv_benchmark binary in build_dir)
  - Python: numpy, matplotlib, Pillow, scipy
  - Optional: COLMAP (for COLMAP comparison)
"""

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("WARNING: matplotlib not found. Plots will be skipped.")

# ---------------------------------------------------------------------------
# TUM trajectory utilities
# ---------------------------------------------------------------------------

def load_tum_trajectory(path):
    """Load TUM-format trajectory → list of (timestamp, 4x4 matrix)."""
    trajectory = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            ts = float(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            T = quat_to_matrix(tx, ty, tz, qx, qy, qz, qw)
            trajectory.append((ts, T))
    return trajectory


def quat_to_matrix(tx, ty, tz, qx, qy, qz, qw):
    """Quaternion + translation → 4x4 matrix."""
    T = np.eye(4)
    # Rotation from quaternion
    n = qx*qx + qy*qy + qz*qz + qw*qw
    if n < 1e-10:
        T[:3, 3] = [tx, ty, tz]
        return T
    s = 2.0 / n
    wx, wy, wz = s*qw*qx, s*qw*qy, s*qw*qz
    xx, xy, xz = s*qx*qx, s*qx*qy, s*qx*qz
    yy, yz, zz = s*qy*qy, s*qy*qz, s*qz*qz

    T[0, 0] = 1 - (yy + zz)
    T[0, 1] = xy - wz
    T[0, 2] = xz + wy
    T[1, 0] = xy + wz
    T[1, 1] = 1 - (xx + zz)
    T[1, 2] = yz - wx
    T[2, 0] = xz - wy
    T[2, 1] = yz + wx
    T[2, 2] = 1 - (xx + yy)
    T[:3, 3] = [tx, ty, tz]
    return T


def align_trajectories(est, gt):
    """Align estimated trajectory to ground truth using Umeyama (SE3).

    Args:
        est: list of (timestamp, 4x4) estimated poses
        gt:  list of (timestamp, 4x4) ground-truth poses

    Returns:
        aligned est, matched gt (same length, time-matched)
    """
    # Match by nearest timestamp
    gt_dict = {}
    for ts, T in gt:
        gt_dict[ts] = T

    gt_times = np.array([ts for ts, _ in gt])
    matched_est = []
    matched_gt = []

    for ts_e, T_e in est:
        idx = np.argmin(np.abs(gt_times - ts_e))
        if abs(gt_times[idx] - ts_e) < 0.05:  # 50ms tolerance
            matched_est.append(T_e)
            matched_gt.append(gt[idx][1])

    if len(matched_est) < 3:
        return matched_est, matched_gt

    # Extract positions
    p_est = np.array([T[:3, 3] for T in matched_est])
    p_gt = np.array([T[:3, 3] for T in matched_gt])

    # Umeyama alignment (translation + rotation, no scale)
    mu_est = np.mean(p_est, axis=0)
    mu_gt = np.mean(p_gt, axis=0)

    est_centered = p_est - mu_est
    gt_centered = p_gt - mu_gt

    H = est_centered.T @ gt_centered
    U, _, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    S = np.eye(3)
    if d < 0:
        S[2, 2] = -1
    R_align = Vt.T @ S @ U.T
    t_align = mu_gt - R_align @ mu_est

    # Apply alignment
    aligned = []
    for T in matched_est:
        T_aligned = np.eye(4)
        T_aligned[:3, :3] = R_align @ T[:3, :3]
        T_aligned[:3, 3] = R_align @ T[:3, 3] + t_align
        aligned.append(T_aligned)

    return aligned, matched_gt


def compute_ate(est_aligned, gt_matched):
    """Compute ATE (RMSE, mean, max) from aligned trajectories."""
    if not est_aligned:
        return {"rmse": float('inf'), "mean": float('inf'), "max": float('inf'), "n": 0}
    errors = []
    for T_e, T_g in zip(est_aligned, gt_matched):
        e = np.linalg.norm(T_e[:3, 3] - T_g[:3, 3])
        errors.append(e)
    errors = np.array(errors)
    return {
        "rmse": float(np.sqrt(np.mean(errors**2))),
        "mean": float(np.mean(errors)),
        "max": float(np.max(errors)),
        "n": len(errors)
    }


def compute_rpe(est_aligned, gt_matched, delta=1):
    """Compute RPE (translation RMSE and rotation RMSE in degrees)."""
    if len(est_aligned) < delta + 1:
        return {"trans_rmse": float('inf'), "rot_rmse": float('inf')}

    trans_errors = []
    rot_errors = []

    for i in range(len(est_aligned) - delta):
        # Relative motion
        dT_est = np.linalg.inv(est_aligned[i]) @ est_aligned[i + delta]
        dT_gt = np.linalg.inv(gt_matched[i]) @ gt_matched[i + delta]

        # Relative error
        dT_err = np.linalg.inv(dT_gt) @ dT_est

        trans_errors.append(np.linalg.norm(dT_err[:3, 3]))
        cos_angle = (np.trace(dT_err[:3, :3]) - 1.0) / 2.0
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        rot_errors.append(np.degrees(np.arccos(cos_angle)))

    return {
        "trans_rmse": float(np.sqrt(np.mean(np.array(trans_errors)**2))),
        "rot_rmse": float(np.sqrt(np.mean(np.array(rot_errors)**2)))
    }


# ---------------------------------------------------------------------------
# COLMAP runner
# ---------------------------------------------------------------------------

def extract_frames_for_colmap(preprocessed_dir, colmap_dir, intrinsics):
    """Extract RGB frames from ARKitScenes for COLMAP.

    ARKitScenes provides lowres_wide/*.png RGB images.
    """
    images_dir = colmap_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    # Look for RGB images in the original ARKitScenes scene dir
    scene_dir = preprocessed_dir  # We'll pass original scene dir separately
    return images_dir


def run_colmap(scene_dir, colmap_work_dir, colmap_binary="colmap"):
    """Run COLMAP reconstruction on a sequence.

    Returns path to exported trajectory or None if COLMAP fails.
    """
    colmap_work_dir = Path(colmap_work_dir)
    images_dir = scene_dir / "lowres_wide"

    if not images_dir.exists():
        print(f"    No RGB images at {images_dir}, skipping COLMAP")
        return None

    # Check COLMAP is available
    if shutil.which(colmap_binary) is None:
        print(f"    COLMAP not found in PATH, skipping")
        return None

    db_path = colmap_work_dir / "database.db"
    sparse_dir = colmap_work_dir / "sparse"
    colmap_work_dir.mkdir(parents=True, exist_ok=True)
    sparse_dir.mkdir(exist_ok=True)

    try:
        # Feature extraction
        print("    COLMAP: feature extraction...")
        subprocess.run([
            colmap_binary, "feature_extractor",
            "--database_path", str(db_path),
            "--image_path", str(images_dir),
            "--ImageReader.single_camera", "1",
            "--ImageReader.camera_model", "PINHOLE",
            "--SiftExtraction.max_image_size", "512",
            "--SiftExtraction.max_num_features", "4096",
        ], check=True, capture_output=True, timeout=600)

        # Exhaustive matching
        print("    COLMAP: matching...")
        subprocess.run([
            colmap_binary, "exhaustive_matcher",
            "--database_path", str(db_path),
            "--SiftMatching.max_num_matches", "4096",
        ], check=True, capture_output=True, timeout=600)

        # Sparse reconstruction
        print("    COLMAP: sparse reconstruction...")
        subprocess.run([
            colmap_binary, "mapper",
            "--database_path", str(db_path),
            "--image_path", str(images_dir),
            "--output_path", str(sparse_dir),
        ], check=True, capture_output=True, timeout=1200)

        # Export poses as TUM
        model_dir = sparse_dir / "0"
        if not model_dir.exists():
            print("    COLMAP: no model produced")
            return None

        tum_path = colmap_work_dir / "colmap_traj.txt"
        subprocess.run([
            colmap_binary, "model_converter",
            "--input_path", str(model_dir),
            "--output_path", str(colmap_work_dir / "model_txt"),
            "--output_type", "TXT",
        ], check=True, capture_output=True, timeout=120)

        # Parse COLMAP images.txt → TUM format
        images_txt = colmap_work_dir / "model_txt" / "images.txt"
        if images_txt.exists():
            traj = parse_colmap_images_txt(images_txt)
            write_tum_trajectory(tum_path, traj)
            print(f"    COLMAP: {len(traj)} poses exported")
            return tum_path

    except subprocess.TimeoutExpired:
        print("    COLMAP: timed out")
    except subprocess.CalledProcessError as e:
        print(f"    COLMAP failed: {e}")
    except Exception as e:
        print(f"    COLMAP error: {e}")

    return None


def parse_colmap_images_txt(path):
    """Parse COLMAP images.txt → list of (timestamp, tx, ty, tz, qx, qy, qz, qw)."""
    traj = []
    with open(path, 'r') as f:
        lines = f.readlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        i += 1
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) < 10:
            i += 1  # Skip the points2D line
            continue
        # IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
        qw, qx, qy, qz = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
        tx, ty, tz = float(parts[5]), float(parts[6]), float(parts[7])
        name = parts[9]
        # Extract timestamp from filename
        try:
            ts = float(Path(name).stem)
        except ValueError:
            ts = float(len(traj))
        # COLMAP stores camera-to-world inverse, convert to world-from-camera
        # Actually COLMAP images.txt stores world-to-camera (R, t such that X_cam = R*X_world + t)
        # We need camera-to-world for trajectory comparison
        R = quat_to_matrix(0, 0, 0, qx, qy, qz, qw)[:3, :3]
        t_vec = np.array([tx, ty, tz])
        # Camera position in world: C = -R^T * t
        C = -R.T @ t_vec
        R_c2w = R.T
        traj.append((ts, C[0], C[1], C[2], qx, qy, qz, qw))
        i += 1  # Skip points2D line
    traj.sort(key=lambda x: x[0])
    return traj


def write_tum_trajectory(path, traj):
    """Write trajectory as TUM format."""
    with open(path, 'w') as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for row in traj:
            f.write(" ".join(f"{v:.6f}" for v in row) + "\n")


# ---------------------------------------------------------------------------
# ARKit baseline (raw poses from .traj)
# ---------------------------------------------------------------------------

def extract_arkit_baseline(scene_dir, output_path):
    """Extract raw ARKit poses as TUM trajectory for baseline comparison."""
    traj_path = scene_dir / "lowres_wide.traj"
    if not traj_path.exists():
        return None

    # Import from preprocess script
    poses = []
    with open(traj_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 7:
                continue
            ts = float(parts[0])
            ax, ay, az = float(parts[1]), float(parts[2]), float(parts[3])
            tx, ty, tz = float(parts[4]), float(parts[5]), float(parts[6])
            # Convert axis-angle to quaternion
            angle = np.sqrt(ax*ax + ay*ay + az*az)
            if angle < 1e-10:
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            else:
                axis = np.array([ax, ay, az]) / angle
                qw = np.cos(angle / 2)
                s = np.sin(angle / 2)
                qx, qy, qz = axis * s
            poses.append((ts, tx, ty, tz, qx, qy, qz, qw))

    write_tum_trajectory(output_path, poses)
    return output_path


# ---------------------------------------------------------------------------
# Result parsing
# ---------------------------------------------------------------------------

def parse_dv_slam_csv(csv_path):
    """Parse DV-SLAM results CSV."""
    results = {}
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            results[row['ablation']] = {
                'ate_rmse': float(row['ate_rmse']),
                'ate_mean': float(row['ate_mean']),
                'ate_max': float(row['ate_max']),
                'rpe_trans': float(row['rpe_trans']),
                'rpe_rot': float(row['rpe_rot']),
                'fps': float(row['fps']),
                'avg_bd_ms': float(row.get('avg_bd_ms', 0)),
                'avg_lio_ms': float(row.get('avg_lio_ms', 0)),
                'reduction_pct': float(row.get('reduction_pct', 0)),
            }
    return results


# ---------------------------------------------------------------------------
# LaTeX table generation
# ---------------------------------------------------------------------------

def generate_main_comparison_table(all_results, output_path):
    """Generate Table I: Main comparison (DV-SLAM vs COLMAP vs ARKit).

    Format: one row per sequence, columns for each method's ATE RMSE.
    """
    lines = []
    lines.append(r"\begin{table}[t]")
    lines.append(r"  \centering")
    lines.append(r"  \caption{Absolute Trajectory Error (ATE RMSE, meters) on ARKitScenes.}")
    lines.append(r"  \label{tab:main_comparison}")
    lines.append(r"  \begin{tabular}{l ccc}")
    lines.append(r"    \toprule")
    lines.append(r"    Sequence & ARKit & COLMAP & \textbf{DV-SLAM (Ours)} \\")
    lines.append(r"    \midrule")

    arkit_vals = []
    colmap_vals = []
    dv_vals = []

    for seq_id, results in sorted(all_results.items()):
        arkit_ate = results.get('arkit', {}).get('ate_rmse', float('inf'))
        colmap_ate = results.get('colmap', {}).get('ate_rmse', float('inf'))
        dv_ate = results.get('dv_slam', {}).get('ate_rmse', float('inf'))

        def fmt(v):
            if v == float('inf') or v > 100:
                return "---"
            return f"{v:.4f}"

        # Bold the best result
        vals = [arkit_ate, colmap_ate, dv_ate]
        best = min(v for v in vals if v < 100)

        arkit_s = fmt(arkit_ate)
        colmap_s = fmt(colmap_ate)
        dv_s = fmt(dv_ate)

        if arkit_ate == best and arkit_ate < 100:
            arkit_s = r"\textbf{" + arkit_s + "}"
        if colmap_ate == best and colmap_ate < 100:
            colmap_s = r"\textbf{" + colmap_s + "}"
        if dv_ate == best and dv_ate < 100:
            dv_s = r"\textbf{" + dv_s + "}"

        lines.append(f"    {seq_id} & {arkit_s} & {colmap_s} & {dv_s} \\\\")

        if arkit_ate < 100: arkit_vals.append(arkit_ate)
        if colmap_ate < 100: colmap_vals.append(colmap_ate)
        if dv_ate < 100: dv_vals.append(dv_ate)

    # Average row
    lines.append(r"    \midrule")
    avg_arkit = np.mean(arkit_vals) if arkit_vals else float('inf')
    avg_colmap = np.mean(colmap_vals) if colmap_vals else float('inf')
    avg_dv = np.mean(dv_vals) if dv_vals else float('inf')

    def fmt(v):
        return f"{v:.4f}" if v < 100 else "---"

    lines.append(f"    \\textit{{Average}} & {fmt(avg_arkit)} & {fmt(avg_colmap)} & {fmt(avg_dv)} \\\\")
    lines.append(r"    \bottomrule")
    lines.append(r"  \end{tabular}")
    lines.append(r"\end{table}")

    with open(output_path, 'w') as f:
        f.write("\n".join(lines) + "\n")
    print(f"  LaTeX table: {output_path}")


def generate_ablation_table(all_results, output_path):
    """Generate Table II: Ablation study.

    Rows: ablation variants. Columns: average ATE RMSE across sequences.
    """
    ablation_names = {
        'full': 'DV-SLAM (Full)',
        'no_bd': 'w/o Bundle \\& Discard',
        'no_conf': 'w/o Confidence Weight',
        'no_tls': 'w/o TLS',
        'no_imu': 'w/o IMU Prediction',
        'arkit_only': 'ARKit Only (no LIO)',
    }

    # Collect per-ablation averages across sequences
    ablation_metrics = {}
    for seq_id, results in all_results.items():
        dv_csv = results.get('dv_slam_ablations', {})
        for abl_name, metrics in dv_csv.items():
            if abl_name not in ablation_metrics:
                ablation_metrics[abl_name] = {
                    'ate_rmse': [], 'rpe_trans': [], 'rpe_rot': [],
                    'fps': [], 'reduction_pct': []
                }
            ablation_metrics[abl_name]['ate_rmse'].append(metrics['ate_rmse'])
            ablation_metrics[abl_name]['rpe_trans'].append(metrics['rpe_trans'])
            ablation_metrics[abl_name]['rpe_rot'].append(metrics['rpe_rot'])
            ablation_metrics[abl_name]['fps'].append(metrics['fps'])
            ablation_metrics[abl_name]['reduction_pct'].append(metrics['reduction_pct'])

    lines = []
    lines.append(r"\begin{table}[t]")
    lines.append(r"  \centering")
    lines.append(r"  \caption{Ablation study on ARKitScenes (averaged across all sequences).}")
    lines.append(r"  \label{tab:ablation}")
    lines.append(r"  \begin{tabular}{l cccc}")
    lines.append(r"    \toprule")
    lines.append(r"    Configuration & ATE$\downarrow$ & RPE$_t\downarrow$ & RPE$_r\downarrow$ & FPS$\uparrow$ \\")
    lines.append(r"    & (m) & (m) & ($^\circ$) & \\")
    lines.append(r"    \midrule")

    order = ['full', 'no_bd', 'no_conf', 'no_tls', 'no_imu', 'arkit_only']
    for abl in order:
        if abl not in ablation_metrics:
            continue
        m = ablation_metrics[abl]
        name = ablation_names.get(abl, abl)
        ate = np.mean(m['ate_rmse'])
        rpe_t = np.mean(m['rpe_trans'])
        rpe_r = np.mean(m['rpe_rot'])
        fps = np.mean(m['fps'])

        if abl == 'full':
            lines.append(f"    \\textbf{{{name}}} & \\textbf{{{ate:.4f}}} & \\textbf{{{rpe_t:.4f}}} & \\textbf{{{rpe_r:.2f}}} & \\textbf{{{fps:.1f}}} \\\\")
        else:
            lines.append(f"    {name} & {ate:.4f} & {rpe_t:.4f} & {rpe_r:.2f} & {fps:.1f} \\\\")

    lines.append(r"    \bottomrule")
    lines.append(r"  \end{tabular}")
    lines.append(r"\end{table}")

    with open(output_path, 'w') as f:
        f.write("\n".join(lines) + "\n")
    print(f"  Ablation table: {output_path}")


def generate_profiling_table(all_results, output_path):
    """Generate Table III: Runtime profiling."""
    ablation_metrics = {}
    for seq_id, results in all_results.items():
        dv_csv = results.get('dv_slam_ablations', {})
        for abl_name, metrics in dv_csv.items():
            if abl_name not in ablation_metrics:
                ablation_metrics[abl_name] = {
                    'avg_bd_ms': [], 'avg_lio_ms': [], 'fps': [], 'reduction_pct': []
                }
            ablation_metrics[abl_name]['avg_bd_ms'].append(metrics.get('avg_bd_ms', 0))
            ablation_metrics[abl_name]['avg_lio_ms'].append(metrics.get('avg_lio_ms', 0))
            ablation_metrics[abl_name]['fps'].append(metrics['fps'])
            ablation_metrics[abl_name]['reduction_pct'].append(metrics.get('reduction_pct', 0))

    lines = []
    lines.append(r"\begin{table}[t]")
    lines.append(r"  \centering")
    lines.append(r"  \caption{Runtime profiling (averaged across all sequences).}")
    lines.append(r"  \label{tab:profiling}")
    lines.append(r"  \begin{tabular}{l cccc}")
    lines.append(r"    \toprule")
    lines.append(r"    Config & B\&D (ms) & LIO (ms) & Reduction (\%) & FPS \\")
    lines.append(r"    \midrule")

    order = ['full', 'no_bd', 'no_conf', 'no_tls', 'no_imu', 'arkit_only']
    for abl in order:
        if abl not in ablation_metrics:
            continue
        m = ablation_metrics[abl]
        bd = np.mean(m['avg_bd_ms'])
        lio = np.mean(m['avg_lio_ms'])
        fps = np.mean(m['fps'])
        red = np.mean(m['reduction_pct'])
        lines.append(f"    {abl} & {bd:.2f} & {lio:.2f} & {red:.1f} & {fps:.1f} \\\\")

    lines.append(r"    \bottomrule")
    lines.append(r"  \end{tabular}")
    lines.append(r"\end{table}")

    with open(output_path, 'w') as f:
        f.write("\n".join(lines) + "\n")
    print(f"  Profiling table: {output_path}")


# ---------------------------------------------------------------------------
# Trajectory plotting
# ---------------------------------------------------------------------------

def plot_trajectories(seq_id, trajectories, output_path):
    """Plot 2D trajectory comparison (top-down XZ view).

    trajectories: dict of name → list of (timestamp, 4x4)
    """
    if not HAS_MATPLOTLIB:
        return

    fig, ax = plt.subplots(1, 1, figsize=(6, 6))

    colors = {
        'Ground Truth': '#333333',
        'ARKit': '#2196F3',
        'COLMAP': '#FF9800',
        'DV-SLAM': '#E91E63',
    }

    for name, traj in trajectories.items():
        if not traj:
            continue
        positions = np.array([T[:3, 3] if isinstance(T, np.ndarray) else T[1][:3, 3] for T in traj])
        if isinstance(traj[0], tuple):
            positions = np.array([T[:3, 3] for _, T in traj])
        color = colors.get(name, '#888888')
        lw = 2.5 if name == 'DV-SLAM' else 1.5
        ls = '-' if name != 'Ground Truth' else '--'
        ax.plot(positions[:, 0], positions[:, 2], color=color, linewidth=lw,
                linestyle=ls, label=name, alpha=0.85)

    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Z (m)', fontsize=12)
    ax.set_title(f'Trajectory — Sequence {seq_id}', fontsize=13)
    ax.legend(fontsize=10, loc='best')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Plot: {output_path}")


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def build_dv_slam(build_dir, source_dir):
    """Build DV-SLAM from CMakeLists.txt."""
    build_dir = Path(build_dir)
    build_dir.mkdir(parents=True, exist_ok=True)

    print("Building DV-SLAM...")
    try:
        subprocess.run(
            ["cmake", str(source_dir), "-DCMAKE_BUILD_TYPE=Release"],
            cwd=str(build_dir), check=True, capture_output=True
        )
        subprocess.run(
            ["cmake", "--build", ".", "--parallel"],
            cwd=str(build_dir), check=True, capture_output=True
        )
        print("  Build successful")
        return True
    except subprocess.CalledProcessError as e:
        print(f"  Build failed: {e}")
        if e.stderr:
            print(e.stderr.decode()[:2000])
        return False


def run_sequence(seq_id, scene_dir, build_dir, output_dir, run_colmap_flag):
    """Run full evaluation pipeline on one sequence."""
    scene_dir = Path(scene_dir)
    build_dir = Path(build_dir)
    output_dir = Path(output_dir) / seq_id
    output_dir.mkdir(parents=True, exist_ok=True)

    results = {}
    print(f"\n{'='*60}")
    print(f"Sequence: {seq_id}")
    print(f"{'='*60}")

    # Step 1: Preprocess
    preprocess_dir = output_dir / "preprocessed"
    preprocess_script = Path(__file__).parent / "preprocess_arkitscenes.py"

    if not (preprocess_dir / "poses.txt").exists():
        print("  Preprocessing ARKitScenes data...")
        try:
            subprocess.run(
                [sys.executable, str(preprocess_script), str(scene_dir), str(preprocess_dir)],
                check=True, capture_output=True
            )
        except subprocess.CalledProcessError as e:
            print(f"  Preprocessing failed: {e}")
            return results
    else:
        print("  Using cached preprocessed data")

    # Load ground truth
    gt_path = preprocess_dir / "poses.txt"
    gt_traj = load_tum_trajectory(gt_path)
    print(f"  Ground truth: {len(gt_traj)} poses")

    # Step 2: Run DV-SLAM benchmark (all ablations)
    dv_output = output_dir / "dv_slam"
    benchmark_bin = build_dir / "dv_benchmark"
    if not benchmark_bin.exists():
        benchmark_bin = build_dir / "bin" / "dv_benchmark"

    if benchmark_bin.exists():
        print("  Running DV-SLAM (all ablations)...")
        try:
            result = subprocess.run(
                [str(benchmark_bin), str(preprocess_dir), str(dv_output), "--ablation", "all"],
                check=True, capture_output=True, timeout=3600
            )
            print(result.stdout.decode()[-500:] if result.stdout else "  (no output)")
        except subprocess.TimeoutExpired:
            print("  DV-SLAM timed out")
        except subprocess.CalledProcessError as e:
            print(f"  DV-SLAM failed: {e}")

        # Parse results
        csv_path = dv_output / "results.csv"
        if csv_path.exists():
            results['dv_slam_ablations'] = parse_dv_slam_csv(csv_path)
            if 'full' in results['dv_slam_ablations']:
                results['dv_slam'] = results['dv_slam_ablations']['full']
    else:
        print(f"  WARNING: dv_benchmark not found at {benchmark_bin}")

    # Step 3: ARKit baseline
    arkit_traj_path = output_dir / "arkit_trajectory.txt"
    extract_arkit_baseline(scene_dir, arkit_traj_path)
    if arkit_traj_path.exists():
        arkit_traj = load_tum_trajectory(arkit_traj_path)
        aligned, gt_matched = align_trajectories(arkit_traj, gt_traj)
        ate = compute_ate(aligned, gt_matched)
        rpe = compute_rpe(aligned, gt_matched)
        results['arkit'] = {
            'ate_rmse': ate['rmse'],
            'ate_mean': ate['mean'],
            'ate_max': ate['max'],
            'rpe_trans': rpe['trans_rmse'],
            'rpe_rot': rpe['rot_rmse'],
        }
        print(f"  ARKit ATE RMSE: {ate['rmse']:.4f} m ({ate['n']} poses)")

    # Step 4: COLMAP (optional)
    if run_colmap_flag:
        colmap_dir = output_dir / "colmap"
        colmap_traj_path = run_colmap(scene_dir, colmap_dir)
        if colmap_traj_path and colmap_traj_path.exists():
            colmap_traj = load_tum_trajectory(colmap_traj_path)
            aligned, gt_matched = align_trajectories(colmap_traj, gt_traj)
            ate = compute_ate(aligned, gt_matched)
            rpe = compute_rpe(aligned, gt_matched)
            results['colmap'] = {
                'ate_rmse': ate['rmse'],
                'ate_mean': ate['mean'],
                'ate_max': ate['max'],
                'rpe_trans': rpe['trans_rmse'],
                'rpe_rot': rpe['rot_rmse'],
            }
            print(f"  COLMAP ATE RMSE: {ate['rmse']:.4f} m ({ate['n']} poses)")
    else:
        print("  Skipping COLMAP (use --colmap to enable)")

    # Step 5: Plot trajectories
    plot_trajs = {}
    plot_trajs['Ground Truth'] = gt_traj
    if arkit_traj_path.exists():
        plot_trajs['ARKit'] = load_tum_trajectory(arkit_traj_path)

    # Load DV-SLAM estimated trajectory
    dv_est_path = dv_output / "full" / "estimated.txt"
    if dv_est_path.exists():
        plot_trajs['DV-SLAM'] = load_tum_trajectory(dv_est_path)

    colmap_est_path = output_dir / "colmap" / "colmap_traj.txt"
    if colmap_est_path.exists():
        plot_trajs['COLMAP'] = load_tum_trajectory(colmap_est_path)

    if len(plot_trajs) > 1:
        plot_trajectories(seq_id, plot_trajs, output_dir / f"trajectory_{seq_id}.pdf")

    return results


def main():
    parser = argparse.ArgumentParser(
        description="DV-SLAM RA-L Benchmark Runner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run on specific sequences:
  python3 run_benchmark.py \\
      --arkitscenes_dir ~/ARKitScenes/raw/Training \\
      --sequences 47331606 47332006 \\
      --build_dir ./build \\
      --output_dir ./results

  # Run with COLMAP comparison:
  python3 run_benchmark.py \\
      --arkitscenes_dir ~/ARKitScenes/raw/Training \\
      --sequences all \\
      --build_dir ./build \\
      --output_dir ./results \\
      --colmap

  # Skip build (use existing binary):
  python3 run_benchmark.py \\
      --arkitscenes_dir ~/ARKitScenes/raw/Training \\
      --sequences 47331606 \\
      --build_dir ./build \\
      --output_dir ./results \\
      --no-build
        """
    )
    parser.add_argument("--arkitscenes_dir", required=True,
                        help="Path to ARKitScenes raw data (e.g. ~/ARKitScenes/raw/Training)")
    parser.add_argument("--sequences", nargs='+', default=["all"],
                        help="Sequence IDs to evaluate (or 'all')")
    parser.add_argument("--build_dir", default="./build",
                        help="CMake build directory")
    parser.add_argument("--output_dir", default="./results",
                        help="Output directory for results")
    parser.add_argument("--colmap", action="store_true",
                        help="Run COLMAP comparison (requires COLMAP installed)")
    parser.add_argument("--no-build", action="store_true",
                        help="Skip building DV-SLAM (use existing binary)")
    parser.add_argument("--source_dir", default=None,
                        help="DV-SLAM source directory (default: auto-detect)")

    args = parser.parse_args()

    arkitscenes_dir = Path(args.arkitscenes_dir).expanduser()
    build_dir = Path(args.build_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Auto-detect source directory
    source_dir = Path(args.source_dir) if args.source_dir else Path(__file__).parent.parent
    if not (source_dir / "CMakeLists.txt").exists():
        source_dir = Path(__file__).parent.parent
    print(f"Source dir: {source_dir}")
    print(f"Build dir:  {build_dir}")
    print(f"Output dir: {output_dir}")

    # Build
    if not args.no_build:
        if not build_dv_slam(build_dir, source_dir):
            print("Build failed. Fix errors and retry, or use --no-build with existing binary.")
            sys.exit(1)

    # Discover sequences
    if "all" in args.sequences:
        if arkitscenes_dir.exists():
            sequences = sorted([d.name for d in arkitscenes_dir.iterdir()
                               if d.is_dir() and d.name.isdigit()])
        else:
            print(f"ERROR: ARKitScenes directory not found: {arkitscenes_dir}")
            sys.exit(1)
    else:
        sequences = args.sequences

    print(f"\nSequences to evaluate: {sequences}")

    # Run evaluation
    all_results = {}
    for seq_id in sequences:
        scene_dir = arkitscenes_dir / seq_id
        if not scene_dir.exists():
            print(f"\nWARNING: Scene directory not found: {scene_dir}")
            continue
        all_results[seq_id] = run_sequence(seq_id, scene_dir, build_dir, output_dir, args.colmap)

    if not all_results:
        print("\nNo results produced. Check your ARKitScenes directory and sequence IDs.")
        sys.exit(1)

    # Generate tables
    print(f"\n{'='*60}")
    print("Generating LaTeX tables")
    print(f"{'='*60}")

    tables_dir = output_dir / "tables"
    tables_dir.mkdir(exist_ok=True)

    generate_main_comparison_table(all_results, tables_dir / "table_main_comparison.tex")
    generate_ablation_table(all_results, tables_dir / "table_ablation.tex")
    generate_profiling_table(all_results, tables_dir / "table_profiling.tex")

    # Save raw results as JSON
    json_results = {}
    for seq_id, res in all_results.items():
        json_results[seq_id] = {}
        for method, metrics in res.items():
            if isinstance(metrics, dict):
                json_results[seq_id][method] = metrics
    with open(output_dir / "all_results.json", 'w') as f:
        json.dump(json_results, f, indent=2)
    print(f"\n  Raw results: {output_dir / 'all_results.json'}")

    # Print summary
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    print(f"{'Sequence':<15} {'ARKit':>10} {'COLMAP':>10} {'DV-SLAM':>10}")
    print(f"{'':<15} {'ATE(m)':>10} {'ATE(m)':>10} {'ATE(m)':>10}")
    print("-" * 50)

    for seq_id in sorted(all_results.keys()):
        res = all_results[seq_id]
        arkit_ate = res.get('arkit', {}).get('ate_rmse', float('inf'))
        colmap_ate = res.get('colmap', {}).get('ate_rmse', float('inf'))
        dv_ate = res.get('dv_slam', {}).get('ate_rmse', float('inf'))

        def fmt(v):
            return f"{v:.4f}" if v < 100 else "---"

        print(f"{seq_id:<15} {fmt(arkit_ate):>10} {fmt(colmap_ate):>10} {fmt(dv_ate):>10}")

    print(f"\nResults saved to: {output_dir}")
    print(f"LaTeX tables:     {tables_dir}")


if __name__ == "__main__":
    main()
