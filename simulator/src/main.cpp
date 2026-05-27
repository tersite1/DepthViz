// DV-SLAM ScanNet++ Simulator
// Offline evaluation tool for IEEE RA-L submission.
//
// Usage:
//   ./dv_sim <scene_path> [--preset full|noConf|sparse100|dense3000|imuOnly]
//            [--stride 4] [--output ./output]
//
// Example:
//   ./dv_sim /data/scannetpp/data/scene_id --preset full --output ./results/scene_id

#include "ScanNetLoader.h"
#include "OfflineRunner.h"
#include <iostream>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

RunConfig parseArgs(int argc, char* argv[]) {
    RunConfig config;

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " <scene_path> [--preset name] [--stride N] [--output dir]" << std::endl;
        std::cerr << "\nPresets: full, noConf, sparse100, dense3000, imuOnly, noTLS" << std::endl;
        exit(1);
    }

    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--preset" && i + 1 < argc) {
            config.preset_name = argv[++i];
        } else if (arg == "--stride" && i + 1 < argc) {
            config.depth_stride = std::stoi(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            config.output_dir = argv[++i];
        }
    }

    // Apply preset
    if (config.preset_name == "full") {
        config.enable_imu = true;
        config.enable_lio = true;
        config.enable_confidence_weight = true;
        config.enable_tls = true;
        config.depth_stride = 4;
    } else if (config.preset_name == "noConf") {
        config.enable_confidence_weight = false;
    } else if (config.preset_name == "sparse100") {
        config.depth_stride = 12;
    } else if (config.preset_name == "dense3000") {
        config.depth_stride = 1;
    } else if (config.preset_name == "imuOnly") {
        config.enable_lio = false;
    } else if (config.preset_name == "noTLS") {
        config.enable_tls = false;
    }

    return config;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " <scene_path> [--preset name] [--stride N] [--output dir]" << std::endl;
        return 1;
    }

    std::string scene_path = argv[1];
    RunConfig config = parseArgs(argc, argv);

    // Append preset to output dir
    config.output_dir = config.output_dir + "/" + config.preset_name;

    std::cout << "╔══════════════════════════════════════╗" << std::endl;
    std::cout << "║   DV-SLAM ScanNet++ Simulator        ║" << std::endl;
    std::cout << "╚══════════════════════════════════════╝" << std::endl;
    std::cout << "Scene: " << scene_path << std::endl;
    std::cout << "Output: " << config.output_dir << std::endl;

    // Load scene
    ScanNetLoader loader;
    ScanNetScene scene;
    if (!loader.load(scene_path, scene)) {
        std::cerr << "Failed to load scene: " << scene_path << std::endl;
        return 1;
    }

    // Run DV-SLAM
    OfflineRunner runner;
    RunResult result = runner.run(scene, config);

    // Write outputs
    fs::create_directories(config.output_dir);

    runner.writeTUMTrajectory(
        config.output_dir + "/estimated_trajectory.txt",
        result.estimated_trajectory);

    runner.writeTUMTrajectory(
        config.output_dir + "/gt_trajectory.txt",
        result.gt_trajectory);

    runner.writePLY(config.output_dir + "/reconstruction.ply");
    runner.writeReport(config.output_dir + "/report.txt", result);

    std::cout << "\n✅ Output written to: " << config.output_dir << std::endl;
    std::cout << "  estimated_trajectory.txt (TUM format)" << std::endl;
    std::cout << "  gt_trajectory.txt (TUM format)" << std::endl;
    std::cout << "  reconstruction.ply" << std::endl;
    std::cout << "  report.txt" << std::endl;

    return 0;
}
