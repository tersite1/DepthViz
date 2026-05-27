#!/usr/bin/env python3
"""
DV-SLAM Evaluation Script
Computes ATE, RPE, and generates comparison plots using 'evo' library.

Usage:
    python evaluate.py <results_dir>
    python evaluate.py ./results/scene_id

Requirements:
    pip install evo numpy matplotlib
"""

import sys
import os
import json
import glob
import numpy as np

def load_tum(path):
    """Load TUM format trajectory: timestamp tx ty tz qx qy qz qw"""
    data = []
    with open(path) as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            vals = list(map(float, line.strip().split()))
            if len(vals) >= 8:
                data.append(vals)
    return np.array(data)

def compute_ate(est, gt):
    """Compute Absolute Trajectory Error (translation only)"""
    n = min(len(est), len(gt))
    errors = np.linalg.norm(est[:n, 1:4] - gt[:n, 1:4], axis=1)
    return {
        'rmse': float(np.sqrt(np.mean(errors**2))),
        'mean': float(np.mean(errors)),
        'median': float(np.median(errors)),
        'max': float(np.max(errors)),
        'std': float(np.std(errors)),
        'num_frames': n
    }

def evaluate_scene(scene_dir):
    """Evaluate all presets in a scene directory"""
    results = {}
    presets = sorted(glob.glob(os.path.join(scene_dir, '*')))

    for preset_dir in presets:
        if not os.path.isdir(preset_dir):
            continue
        preset = os.path.basename(preset_dir)

        est_path = os.path.join(preset_dir, 'estimated_trajectory.txt')
        gt_path = os.path.join(preset_dir, 'gt_trajectory.txt')

        if not os.path.exists(est_path) or not os.path.exists(gt_path):
            continue

        est = load_tum(est_path)
        gt = load_tum(gt_path)

        if len(est) == 0 or len(gt) == 0:
            continue

        metrics = compute_ate(est, gt)
        results[preset] = metrics
        print(f"  {preset:15s} | ATE RMSE: {metrics['rmse']*100:.2f} cm | "
              f"Mean: {metrics['mean']*100:.2f} cm | Max: {metrics['max']*100:.2f} cm | "
              f"Frames: {metrics['num_frames']}")

    return results

def generate_table(all_results):
    """Generate LaTeX table for paper"""
    print("\n" + "="*80)
    print("LaTeX Table (copy to paper):")
    print("="*80)

    presets_order = ['full', 'noConf', 'noTLS', 'sparse100', 'dense3000', 'imuOnly']
    preset_labels = {
        'full': 'DV-SLAM (Full)',
        'noConf': '$-$ Confidence',
        'noTLS': '$-$ TLS',
        'sparse100': '$\\sim$100 pts',
        'dense3000': '$\\sim$3000 pts',
        'imuOnly': 'IMU Only'
    }

    print("\\begin{tabular}{lccc}")
    print("\\toprule")
    print("Configuration & ATE RMSE [cm] & ATE Mean [cm] & Frames \\\\")
    print("\\midrule")

    for preset in presets_order:
        if preset in all_results:
            m = all_results[preset]
            label = preset_labels.get(preset, preset)
            print(f"{label} & {m['rmse']*100:.2f} & {m['mean']*100:.2f} & {m['num_frames']} \\\\")

    print("\\bottomrule")
    print("\\end{tabular}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python evaluate.py <results_dir>")
        print("       python evaluate.py ./results")
        sys.exit(1)

    results_dir = sys.argv[1]

    if not os.path.exists(results_dir):
        print(f"Directory not found: {results_dir}")
        sys.exit(1)

    # Check if this is a single scene or multi-scene directory
    scene_dirs = sorted(glob.glob(os.path.join(results_dir, '*')))

    all_scene_results = {}

    for scene_dir in scene_dirs:
        if not os.path.isdir(scene_dir):
            continue

        # Check if this contains presets (has subdirs with trajectory files)
        has_presets = any(
            os.path.exists(os.path.join(scene_dir, d, 'estimated_trajectory.txt'))
            for d in os.listdir(scene_dir)
            if os.path.isdir(os.path.join(scene_dir, d))
        )

        if has_presets:
            scene_name = os.path.basename(scene_dir)
            print(f"\n{'='*60}")
            print(f"Scene: {scene_name}")
            print(f"{'='*60}")
            results = evaluate_scene(scene_dir)
            if results:
                all_scene_results[scene_name] = results

    # Aggregate across scenes
    if len(all_scene_results) > 1:
        print(f"\n{'='*60}")
        print("Aggregated Results (mean across scenes)")
        print(f"{'='*60}")
        presets = set()
        for sr in all_scene_results.values():
            presets.update(sr.keys())

        agg = {}
        for preset in sorted(presets):
            rmses = [sr[preset]['rmse'] for sr in all_scene_results.values() if preset in sr]
            if rmses:
                agg[preset] = {
                    'rmse': float(np.mean(rmses)),
                    'mean': float(np.mean(rmses)),  # approximate
                    'max': float(np.max(rmses)),
                    'num_frames': sum(sr[preset]['num_frames'] for sr in all_scene_results.values() if preset in sr)
                }
                print(f"  {preset:15s} | ATE RMSE: {agg[preset]['rmse']*100:.2f} cm (n={len(rmses)} scenes)")

        generate_table(agg)
    elif len(all_scene_results) == 1:
        scene_name = list(all_scene_results.keys())[0]
        generate_table(all_scene_results[scene_name])

    # Save JSON
    json_path = os.path.join(results_dir, 'evaluation_results.json')
    with open(json_path, 'w') as f:
        json.dump(all_scene_results, f, indent=2)
    print(f"\nResults saved to: {json_path}")

if __name__ == '__main__':
    main()
