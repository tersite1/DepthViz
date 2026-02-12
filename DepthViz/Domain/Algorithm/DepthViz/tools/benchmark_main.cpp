// DV-SLAM Benchmark CLI
// Usage:
//   dv_benchmark <sequence_dir> <output_dir> [--ablation <name>]
//
// Ablation options:
//   full        — Full DV-SLAM pipeline (default)
//   no_bd       — Disable Bundle & Discard
//   no_conf     — Disable confidence weighting
//   no_tls      — Disable Truncated Least Squares
//   no_imu      — Disable IMU prediction
//   arkit_only  — ARKit poses only (no LIO refinement)
//   all         — Run all ablations sequentially

#include "../include/DV_OfflineEvaluator.h"
#include "../include/DepthVizEngine.hpp"

#include <string>
#include <vector>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>

struct AblationRun {
    std::string name;
    DepthVizEngine::AblationConfig config;
};

static void printUsage(const char* prog) {
    printf("DV-SLAM Benchmark\n");
    printf("Usage: %s <sequence_dir> <output_dir> [--ablation <name>]\n\n", prog);
    printf("Ablation options:\n");
    printf("  full        Full DV-SLAM (default)\n");
    printf("  no_bd       Disable Bundle & Discard\n");
    printf("  no_conf     Disable confidence weighting\n");
    printf("  no_tls      Disable Truncated Least Squares\n");
    printf("  no_imu      Disable IMU prediction\n");
    printf("  arkit_only  ARKit poses only (no LIO)\n");
    printf("  all         Run all ablations\n");
}

static std::vector<AblationRun> getAblations(const std::string& name) {
    std::vector<AblationRun> runs;

    auto makeConfig = [](bool bd, bool conf, bool tls, bool imu, bool lio) {
        DepthVizEngine::AblationConfig c;
        c.enable_bundle_discard = bd;
        c.enable_confidence_weight = conf;
        c.enable_tls = tls;
        c.enable_imu = imu;
        c.enable_lio = lio;
        return c;
    };

    if (name == "full" || name == "all") {
        runs.push_back({"full", makeConfig(true, true, true, true, true)});
    }
    if (name == "no_bd" || name == "all") {
        runs.push_back({"no_bd", makeConfig(false, true, true, true, true)});
    }
    if (name == "no_conf" || name == "all") {
        runs.push_back({"no_conf", makeConfig(true, false, true, true, true)});
    }
    if (name == "no_tls" || name == "all") {
        runs.push_back({"no_tls", makeConfig(true, true, false, true, true)});
    }
    if (name == "no_imu" || name == "all") {
        runs.push_back({"no_imu", makeConfig(true, true, true, false, true)});
    }
    if (name == "arkit_only" || name == "all") {
        runs.push_back({"arkit_only", makeConfig(false, false, false, false, false)});
    }

    return runs;
}

static void mkdirp(const std::string& path) {
    mkdir(path.c_str(), 0755);
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    std::string seq_dir = argv[1];
    std::string out_dir = argv[2];
    std::string ablation_name = "full";

    for (int i = 3; i < argc; i++) {
        if (std::strcmp(argv[i], "--ablation") == 0 && i + 1 < argc) {
            ablation_name = argv[++i];
        }
    }

    auto ablations = getAblations(ablation_name);
    if (ablations.empty()) {
        printf("ERROR: Unknown ablation '%s'\n", ablation_name.c_str());
        printUsage(argv[0]);
        return 1;
    }

    // Load sequence once (shared across ablations)
    DV::DV_OfflineEvaluator evaluator;
    if (!evaluator.loadSequence(seq_dir)) {
        printf("ERROR: Failed to load sequence from %s\n", seq_dir.c_str());
        return 1;
    }

    mkdirp(out_dir);

    // Summary table header
    printf("\n%-15s %10s %10s %10s %10s %10s %10s\n",
           "Ablation", "ATE RMSE", "ATE Mean", "ATE Max", "RPE Trans", "RPE Rot", "FPS");
    printf("%-15s %10s %10s %10s %10s %10s %10s\n",
           "", "(m)", "(m)", "(m)", "(m)", "(deg)", "");
    printf("%s\n", std::string(85, '-').c_str());

    // CSV output for LaTeX table generation
    std::string csv_path = out_dir + "/results.csv";
    FILE* csv = fopen(csv_path.c_str(), "w");
    if (csv) {
        fprintf(csv, "ablation,ate_rmse,ate_mean,ate_max,rpe_trans,rpe_rot,fps,avg_bd_ms,avg_lio_ms,reduction_pct\n");
    }

    for (const auto& run : ablations) {
        printf("\n=== Running: %s ===\n", run.name.c_str());

        // Create fresh evaluator for each run (engine resets)
        DV::DV_OfflineEvaluator eval_run;
        eval_run.loadSequence(seq_dir);
        eval_run.setAblation(run.config);

        auto error = eval_run.evaluate();
        auto profiling = eval_run.getProfilingStats();

        // Print summary row
        printf("%-15s %10.4f %10.4f %10.4f %10.4f %10.2f %10.1f\n",
               run.name.c_str(),
               error.ate_rmse, error.ate_mean, error.ate_max,
               error.rpe_rmse, error.rpe_rot_rmse,
               profiling.fps());

        // Write outputs
        std::string run_dir = out_dir + "/" + run.name;
        mkdirp(run_dir);

        eval_run.writeTUMTrajectory(run_dir + "/estimated.txt", eval_run.getEstimatedTrajectory());
        eval_run.writeTUMTrajectory(run_dir + "/groundtruth.txt", eval_run.getGroundTruthTrajectory());
        eval_run.writeErrorReport(run_dir + "/report.txt", error, &profiling);
        eval_run.writePLY(run_dir + "/reconstruction.ply");
        eval_run.writePLYWithTrajectory(run_dir + "/reconstruction_with_traj.ply");

        // CSV row
        if (csv) {
            fprintf(csv, "%s,%.6f,%.6f,%.6f,%.6f,%.4f,%.1f,%.2f,%.2f,%.1f\n",
                    run.name.c_str(),
                    error.ate_rmse, error.ate_mean, error.ate_max,
                    error.rpe_rmse, error.rpe_rot_rmse,
                    profiling.fps(),
                    profiling.avgBundleDiscardMs(),
                    profiling.avgLioMs(),
                    profiling.avgReductionRatio() * 100.0);
        }
    }

    if (csv) fclose(csv);

    printf("\n%s\n", std::string(85, '-').c_str());
    printf("Results saved to: %s\n", out_dir.c_str());
    printf("CSV results: %s\n", csv_path.c_str());

    return 0;
}
