#include "ScanNetLoader.h"
#include "json.hpp"

#include <fstream>
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <cstring>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

bool ScanNetLoader::load(const std::string& scene_path, ScanNetScene& scene) {
    scene.scene_path = scene_path;
    scene.scene_id = fs::path(scene_path).filename().string();

    // Paths
    std::string iphone_dir = scene_path + "/iphone";
    std::string json_path = iphone_dir + "/pose_intrinsic_imu.json";
    std::string depth_dir = iphone_dir + "/depth";
    std::string mesh_path = scene_path + "/scans/mesh_aligned_0.05.ply";

    if (fs::exists(mesh_path)) {
        scene.mesh_path = mesh_path;
    }

    // Load JSON (poses, intrinsics, IMU)
    if (!loadPoseIntrinsicIMU(json_path, scene)) {
        std::cerr << "[ScanNetLoader] Failed to load " << json_path << std::endl;
        return false;
    }

    // Load depth frames
    if (!loadDepthFrames(depth_dir, scene)) {
        std::cerr << "[ScanNetLoader] Failed to load depth from " << depth_dir << std::endl;
        return false;
    }

    // Compute depth intrinsics from first RGB intrinsic
    if (!scene.poses.empty()) {
        auto& K = scene.poses[0].intrinsic;
        float scale_x = 256.0f / 1920.0f;
        float scale_y = 192.0f / 1440.0f;
        scene.fx_depth = static_cast<float>(K(0, 0)) * scale_x;
        scene.fy_depth = static_cast<float>(K(1, 1)) * scale_y;
        scene.cx_depth = static_cast<float>(K(0, 2)) * scale_x;
        scene.cy_depth = static_cast<float>(K(1, 2)) * scale_y;
    }

    std::cout << "[ScanNetLoader] Scene: " << scene.scene_id << std::endl;
    std::cout << "  Poses: " << scene.poses.size() << std::endl;
    std::cout << "  Depth frames: " << scene.depth_frames.size() << std::endl;
    std::cout << "  IMU samples: " << scene.imu_samples.size() << std::endl;
    std::cout << "  Depth intrinsics: fx=" << scene.fx_depth
              << " fy=" << scene.fy_depth
              << " cx=" << scene.cx_depth
              << " cy=" << scene.cy_depth << std::endl;
    if (!scene.mesh_path.empty()) {
        std::cout << "  GT mesh: " << scene.mesh_path << std::endl;
    }

    return scene.valid();
}

bool ScanNetLoader::loadPoseIntrinsicIMU(const std::string& json_path, ScanNetScene& scene) {
    std::ifstream file(json_path);
    if (!file.is_open()) return false;

    json j;
    try {
        file >> j;
    } catch (const std::exception& e) {
        std::cerr << "[ScanNetLoader] JSON parse error: " << e.what() << std::endl;
        return false;
    }

    // Iterate over frames
    for (auto& [frame_name, frame_data] : j.items()) {
        PoseEntry entry;
        entry.frame_name = frame_name;

        // Timestamp
        if (frame_data.contains("timestamp")) {
            entry.timestamp = frame_data["timestamp"].get<double>();
        } else {
            continue;
        }

        // Pose (4×4 camera-to-world, row-major in JSON)
        if (frame_data.contains("pose")) {
            auto& p = frame_data["pose"];
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    entry.pose(r, c) = p[r][c].get<double>();
        }

        // Aligned pose (GT reference)
        if (frame_data.contains("aligned_pose")) {
            auto& p = frame_data["aligned_pose"];
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    entry.aligned_pose(r, c) = p[r][c].get<double>();
        } else {
            entry.aligned_pose = entry.pose;
        }

        // Intrinsic (3×3)
        if (frame_data.contains("intrinsic")) {
            auto& k = frame_data["intrinsic"];
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    entry.intrinsic(r, c) = k[r][c].get<double>();
        }

        scene.poses.push_back(entry);

        // IMU data (if present in this frame)
        // ScanNet++ ARKit-Scanner stores IMU as 15 floats:
        // [rot_rate(3), user_accel(3), mag(3), attitude(3), gravity(3)]
        if (frame_data.contains("imu") && frame_data["imu"].is_array()) {
            auto& imu_arr = frame_data["imu"];
            // Can be a single sample or array of samples
            if (imu_arr.size() == 15) {
                // Single IMU sample embedded in frame
                IMUSample sample;
                sample.timestamp = entry.timestamp;
                double gyr_x = imu_arr[0].get<double>();
                double gyr_y = imu_arr[1].get<double>();
                double gyr_z = imu_arr[2].get<double>();
                double ua_x = imu_arr[3].get<double>();
                double ua_y = imu_arr[4].get<double>();
                double ua_z = imu_arr[5].get<double>();
                double grav_x = imu_arr[12].get<double>();
                double grav_y = imu_arr[13].get<double>();
                double grav_z = imu_arr[14].get<double>();

                sample.gyr = Eigen::Vector3d(gyr_x, gyr_y, gyr_z);
                // specific_force = -(gravity + userAcceleration) * 9.81
                sample.acc = Eigen::Vector3d(
                    -(grav_x + ua_x) * 9.81,
                    -(grav_y + ua_y) * 9.81,
                    -(grav_z + ua_z) * 9.81
                );
                scene.imu_samples.push_back(sample);
            } else if (imu_arr.size() > 15 && imu_arr[0].is_array()) {
                // Array of IMU samples
                for (auto& s : imu_arr) {
                    if (s.size() < 15) continue;
                    IMUSample sample;
                    // If timestamp field exists
                    if (s.size() >= 16) {
                        sample.timestamp = s[15].get<double>();
                    } else {
                        sample.timestamp = entry.timestamp;
                    }
                    double gyr_x = s[0].get<double>();
                    double gyr_y = s[1].get<double>();
                    double gyr_z = s[2].get<double>();
                    double ua_x = s[3].get<double>();
                    double ua_y = s[4].get<double>();
                    double ua_z = s[5].get<double>();
                    double grav_x = s[12].get<double>();
                    double grav_y = s[13].get<double>();
                    double grav_z = s[14].get<double>();

                    sample.gyr = Eigen::Vector3d(gyr_x, gyr_y, gyr_z);
                    sample.acc = Eigen::Vector3d(
                        -(grav_x + ua_x) * 9.81,
                        -(grav_y + ua_y) * 9.81,
                        -(grav_z + ua_z) * 9.81
                    );
                    scene.imu_samples.push_back(sample);
                }
            }
        }
    }

    // Sort by timestamp
    std::sort(scene.poses.begin(), scene.poses.end(),
        [](const PoseEntry& a, const PoseEntry& b) { return a.timestamp < b.timestamp; });
    std::sort(scene.imu_samples.begin(), scene.imu_samples.end(),
        [](const IMUSample& a, const IMUSample& b) { return a.timestamp < b.timestamp; });

    return !scene.poses.empty();
}

bool ScanNetLoader::loadDepthFrames(const std::string& depth_dir, ScanNetScene& scene) {
    if (!fs::exists(depth_dir)) {
        std::cerr << "[ScanNetLoader] Depth directory not found: " << depth_dir << std::endl;
        return false;
    }

    // Collect all PNG files
    std::vector<std::string> depth_files;
    for (auto& entry : fs::directory_iterator(depth_dir)) {
        if (entry.path().extension() == ".png") {
            depth_files.push_back(entry.path().filename().string());
        }
    }
    std::sort(depth_files.begin(), depth_files.end());

    // Match depth frames with poses by frame name
    std::map<std::string, size_t> pose_map;
    for (size_t i = 0; i < scene.poses.size(); i++) {
        pose_map[scene.poses[i].frame_name] = i;
    }

    for (auto& fname : depth_files) {
        // frame_000123.png → frame_000123
        std::string frame_name = fname.substr(0, fname.size() - 4);

        DepthFrame frame;
        frame.frame_name = frame_name;

        // Find matching pose for timestamp
        auto it = pose_map.find(frame_name);
        if (it != pose_map.end()) {
            frame.timestamp = scene.poses[it->second].timestamp;
        } else {
            // Try to extract frame index and interpolate
            frame.timestamp = scene.depth_frames.size() / 30.0; // fallback: assume 30fps
        }

        std::string full_path = depth_dir + "/" + fname;
        if (loadDepthPNG(full_path, frame)) {
            scene.depth_frames.push_back(std::move(frame));
        }
    }

    // Sort by timestamp
    std::sort(scene.depth_frames.begin(), scene.depth_frames.end(),
        [](const DepthFrame& a, const DepthFrame& b) { return a.timestamp < b.timestamp; });

    return !scene.depth_frames.empty();
}

bool ScanNetLoader::loadDepthPNG(const std::string& path, DepthFrame& frame) {
    // Load 16-bit PNG using stb_image
    int w, h, channels;
    uint16_t* data = reinterpret_cast<uint16_t*>(
        stbi_load_16(path.c_str(), &w, &h, &channels, 1)
    );
    if (!data) {
        return false;
    }

    frame.width = w;
    frame.height = h;
    frame.depth_mm.assign(data, data + w * h);
    stbi_image_free(data);
    return true;
}
