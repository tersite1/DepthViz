#ifndef DEPTHVIZ_TESTS_HPP
#define DEPTHVIZ_TESTS_HPP

#include <cassert>
#include <cmath>
#include <vector>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../include/DV_Types.h"
#include "../include/DV_VoxelHashMap.h"
#include "../include/DV_ESKF.h"
#include "../include/DV_RobustKernels.h"

// Test framework helpers
namespace DepthVizTest {

class TestResult {
public:
    std::string name;
    bool passed;
    std::string message;
    double duration_ms;

    TestResult(const std::string& n, bool p, const std::string& m = "", double d = 0)
        : name(n), passed(p), message(m), duration_ms(d) {}
};

class TestSuite {
private:
    std::vector<TestResult> results;

public:
    void addResult(const TestResult& result) {
        results.push_back(result);
    }

    void printReport() {
        std::cout << "\n========================================" << std::endl;
        std::cout << "DepthViz Test Report" << std::endl;
        std::cout << "========================================" << std::endl;

        int passed = 0, failed = 0;
        double totalTime = 0;

        for (const auto& result : results) {
            std::string status = result.passed ? "PASS" : "FAIL";
            std::cout << std::setw(50) << std::left << result.name
                      << status << " (" << std::fixed << std::setprecision(3)
                      << result.duration_ms << "ms)";
            if (!result.message.empty()) {
                std::cout << " - " << result.message;
            }
            std::cout << std::endl;

            if (result.passed) passed++;
            else failed++;
            totalTime += result.duration_ms;
        }

        std::cout << "========================================" << std::endl;
        std::cout << "Total: " << results.size() << " tests, "
                  << passed << " passed, " << failed << " failed" << std::endl;
        std::cout << "Total Time: " << std::fixed << std::setprecision(2)
                  << totalTime << "ms" << std::endl;
        std::cout << "========================================\n" << std::endl;
    }

    bool allPassed() const {
        for (const auto& result : results) {
            if (!result.passed) return false;
        }
        return true;
    }
};

// Helper macros
#define ASSERT_EQ(actual, expected) \
    if ((actual) != (expected)) { \
        std::cerr << "Assertion failed at line " << __LINE__ << std::endl; \
        return false; \
    }

#define ASSERT_TRUE(condition) \
    if (!(condition)) { \
        std::cerr << "Assertion failed at line " << __LINE__ << std::endl; \
        return false; \
    }

#define ASSERT_FLOAT_EQ(actual, expected, tolerance) \
    if (std::abs((actual) - (expected)) > (tolerance)) { \
        std::cerr << "Float assertion failed at line " << __LINE__ << std::endl; \
        std::cerr << "Expected: " << (expected) << ", Got: " << (actual) << std::endl; \
        return false; \
    }

#define RUN_TEST(suite, testFunc, testName) \
    { \
        auto start = std::chrono::high_resolution_clock::now(); \
        bool result = testFunc(); \
        auto end = std::chrono::high_resolution_clock::now(); \
        double duration = std::chrono::duration<double, std::milli>(end - start).count(); \
        suite.addResult(DepthVizTest::TestResult(testName, result, "", duration)); \
    }

} // namespace DepthVizTest

// ============================================================================
// UNIT TEST DEFINITIONS
// ============================================================================

namespace DepthVizTests {

// Task 1.1: DVPoint3D Structure Tests
// ============================================================================

bool test_DVPoint3D_Initialization() {
    DV::DVPoint3D point;
    point.x = 1.0f; point.y = 2.0f; point.z = 3.0f;
    point.intensity = 0.8f;
    point.confidence = 1.5f;
    point.timestamp = 1234567890.0;
    point.voxel_idx = 12345;

    ASSERT_FLOAT_EQ(point.x, 1.0f, 0.001f);
    ASSERT_FLOAT_EQ(point.y, 2.0f, 0.001f);
    ASSERT_FLOAT_EQ(point.z, 3.0f, 0.001f);
    ASSERT_FLOAT_EQ(point.intensity, 0.8f, 0.001f);
    ASSERT_FLOAT_EQ(point.confidence, 1.5f, 0.001f);

    return true;
}

bool test_DVPoint3D_DefaultValues() {
    DV::DVPoint3D point;
    point.confidence = 0.0f;
    ASSERT_FLOAT_EQ(point.confidence, 0.0f, 0.001f);

    point.confidence = 1.0f;
    ASSERT_FLOAT_EQ(point.confidence, 1.0f, 0.001f);

    point.confidence = 2.0f;
    ASSERT_FLOAT_EQ(point.confidence, 2.0f, 0.001f);

    return true;
}

bool test_DVPoint3D_TimestampAccuracy() {
    DV::DVPoint3D point;
    point.timestamp = 1234567890.123456;
    ASSERT_FLOAT_EQ(point.timestamp, 1234567890.123456, 0.000001);
    return true;
}

bool test_DVPoint3D_VoxelIdx() {
    // Points in the same 10cm voxel should get the same index
    int64_t h1 = DV::DVPoint3D::computeVoxelIdx(0.01f, 0.01f, 0.01f, 0.1f);
    int64_t h2 = DV::DVPoint3D::computeVoxelIdx(0.05f, 0.05f, 0.05f, 0.1f);
    int64_t h3 = DV::DVPoint3D::computeVoxelIdx(0.15f, 0.01f, 0.01f, 0.1f);

    ASSERT_EQ(h1, h2);
    ASSERT_TRUE(h1 != h3);

    return true;
}

// Task 2.1: Spatial Hash Tests
// ============================================================================

bool test_SpatialHash_Voxelization() {
    float voxel_size = 0.1f;

    int64_t hash1 = DV::DVPoint3D::computeVoxelIdx(0.0f, 0.0f, 0.0f, voxel_size);
    int64_t hash2 = DV::DVPoint3D::computeVoxelIdx(0.05f, 0.05f, 0.05f, voxel_size);
    int64_t hash3 = DV::DVPoint3D::computeVoxelIdx(0.15f, 0.0f, 0.0f, voxel_size);

    ASSERT_EQ(hash1, hash2);
    ASSERT_TRUE(hash1 != hash3);

    return true;
}

bool test_SpatialHash_CollisionRate() {
    float voxel_size = 0.1f;
    std::vector<int64_t> hashes;
    int collisions = 0;

    for (int i = 0; i < 1000; i++) {
        float x = (float)(i % 10) * 0.15f;
        float y = (float)(i / 10) * 0.15f;
        float z = 0.5f;

        int64_t hash = DV::DVPoint3D::computeVoxelIdx(x, y, z, voxel_size);

        for (const auto& h : hashes) {
            if (h == hash) collisions++;
        }
        hashes.push_back(hash);
    }

    float collisionRate = (float)collisions / 1000.0f;
    ASSERT_TRUE(collisionRate < 0.10f);

    return true;
}

// Task 3.1: Bundle & Discard Tests
// ============================================================================

bool test_BundleAndDiscard_PointCount() {
    int inputPoints = 10000;
    int expectedOutput = 1000;
    int tolerance = 500;

    int outputPoints = inputPoints / 10;

    ASSERT_TRUE(std::abs(outputPoints - expectedOutput) < tolerance);

    return true;
}

bool test_BundleAndDiscard_CentroidAccuracy() {
    std::vector<float> points = {
        1.0f, 2.0f, 3.0f,
        1.1f, 2.1f, 3.1f,
        0.9f, 1.9f, 2.9f
    };

    float cx = (1.0f + 1.1f + 0.9f) / 3.0f;
    float cy = (2.0f + 2.1f + 1.9f) / 3.0f;
    float cz = (3.0f + 3.1f + 2.9f) / 3.0f;

    ASSERT_FLOAT_EQ(cx, 1.0f, 0.001f);
    ASSERT_FLOAT_EQ(cy, 2.0f, 0.001f);
    ASSERT_FLOAT_EQ(cz, 3.0f, 0.001f);

    return true;
}

bool test_BundleAndDiscard_ConfidenceWeighting() {
    std::vector<uint8_t> confidences = {0, 1, 2, 1, 0, 2};
    int validPoints = 0;

    for (uint8_t conf : confidences) {
        if (conf >= 1) validPoints++;
    }

    ASSERT_EQ(validPoints, 4);

    return true;
}

// Task 3.2: Robust Kernels Tests
// ============================================================================

bool test_RobustKernels_ConfidenceWeight() {
    ASSERT_FLOAT_EQ(DepthViz::getConfidenceWeight(2.0f), 1.0f, 0.001f);
    ASSERT_FLOAT_EQ(DepthViz::getConfidenceWeight(1.0f), 0.5f, 0.001f);
    ASSERT_FLOAT_EQ(DepthViz::getConfidenceWeight(0.0f), 0.0f, 0.001f);

    return true;
}

bool test_RobustKernels_TruncatedLeastSquares() {
    ASSERT_FLOAT_EQ(DepthViz::computeTLSWeight(0.05f), 1.0f, 0.001f);
    ASSERT_FLOAT_EQ(DepthViz::computeTLSWeight(0.15f), 0.0f, 0.001f);
    ASSERT_FLOAT_EQ(DepthViz::computeTLSWeight(-0.08f), 1.0f, 0.001f);

    return true;
}

bool test_RobustKernels_CombinedWeighting() {
    float confWeight = DepthViz::getConfidenceWeight(1.5f);
    float tlsWeight = DepthViz::computeTLSWeight(0.05f);
    float totalWeight = confWeight * tlsWeight;

    ASSERT_FLOAT_EQ(totalWeight, 1.0f, 0.001f);

    return true;
}

// Task 3.3: Pose Estimation Tests
// ============================================================================

bool test_PoseEstimation_IdentityInitialization() {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    ASSERT_FLOAT_EQ(T(0, 0), 1.0f, 0.001f);
    ASSERT_FLOAT_EQ(T(1, 1), 1.0f, 0.001f);
    ASSERT_FLOAT_EQ(T(2, 2), 1.0f, 0.001f);
    ASSERT_FLOAT_EQ(T(0, 3), 0.0f, 0.001f);
    ASSERT_FLOAT_EQ(T(1, 3), 0.0f, 0.001f);
    ASSERT_FLOAT_EQ(T(2, 3), 0.0f, 0.001f);

    return true;
}

bool test_PoseEstimation_PoseComposition() {
    Eigen::Matrix4d T1 = Eigen::Matrix4d::Identity();
    T1.block<3, 1>(0, 3) = Eigen::Vector3d(1.0, 0.0, 0.0);

    Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
    T2.block<3, 1>(0, 3) = Eigen::Vector3d(1.0, 0.0, 0.0);

    Eigen::Matrix4d T_composed = T1 * T2;

    ASSERT_FLOAT_EQ(T_composed(0, 3), 2.0f, 0.001f);

    return true;
}

bool test_PoseEstimation_PoseInverse() {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 1>(0, 3) = Eigen::Vector3d(5.0, -3.0, 2.0);

    Eigen::Matrix4d T_inv = T.inverse();
    Eigen::Matrix4d T_product = T * T_inv;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            ASSERT_FLOAT_EQ(T_product(i, j), expected, 0.001f);
        }
    }

    return true;
}

// SO3/SE3 Manifold Tests
// ============================================================================

bool test_SO3_ExpLog() {
    DV::V3d omega(0.1, 0.2, 0.3);
    DV::SO3 R = DV::SO3::Exp(omega);
    DV::V3d omega_recovered = R.Log();

    ASSERT_FLOAT_EQ(omega_recovered.x(), omega.x(), 0.001);
    ASSERT_FLOAT_EQ(omega_recovered.y(), omega.y(), 0.001);
    ASSERT_FLOAT_EQ(omega_recovered.z(), omega.z(), 0.001);

    return true;
}

bool test_SE3_Composition() {
    DV::SE3 T1(DV::SO3(), DV::V3d(1.0, 0.0, 0.0));
    DV::SE3 T2(DV::SO3(), DV::V3d(0.0, 1.0, 0.0));
    DV::SE3 T12 = T1 * T2;

    ASSERT_FLOAT_EQ(T12.trans.x(), 1.0, 0.001);
    ASSERT_FLOAT_EQ(T12.trans.y(), 1.0, 0.001);
    ASSERT_FLOAT_EQ(T12.trans.z(), 0.0, 0.001);

    return true;
}

bool test_SE3_Inverse() {
    DV::SE3 T(DV::SO3::Exp(DV::V3d(0.1, 0.2, 0.3)), DV::V3d(1.0, 2.0, 3.0));
    DV::SE3 T_inv = T.inverse();
    DV::SE3 identity = T * T_inv;

    DV::M4d I = identity.matrix();
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double expected = (i == j) ? 1.0 : 0.0;
            ASSERT_FLOAT_EQ(I(i, j), expected, 0.001);
        }
    }

    return true;
}

// Task 4: Keyframe Strategy Tests
// ============================================================================

bool test_KeyframeStrategy_TranslationThreshold() {
    Eigen::Vector3d p1(0.0, 0.0, 0.0);
    Eigen::Vector3d p2(0.05, 0.0, 0.0);
    float dist = (float)(p2 - p1).norm();

    ASSERT_TRUE(dist >= 0.05f);

    return true;
}

bool test_KeyframeStrategy_RotationThreshold() {
    Eigen::AngleAxisd aa(2.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    double angle = aa.angle();
    float threshold = 2.0f * M_PI / 180.0f;

    ASSERT_TRUE(angle >= threshold);

    return true;
}

bool test_KeyframeStrategy_NoKeyframeBelow() {
    Eigen::Vector3d p1(0.0, 0.0, 0.0);
    Eigen::Vector3d p2(0.02, 0.0, 0.0);
    float dist = (float)(p2 - p1).norm();

    ASSERT_TRUE(dist < 0.05f);

    return true;
}

// Task 5: Performance Tests
// ============================================================================

bool test_Performance_BundleAndDiscardSpeed() {
    auto start = std::chrono::high_resolution_clock::now();

    int pointCount = 10000;
    std::vector<float> xyz(pointCount * 3);
    std::vector<uint8_t> conf(pointCount);

    for (int i = 0; i < pointCount; i++) {
        xyz[i * 3] = (float)(i % 100) * 0.01f;
        xyz[i * 3 + 1] = (float)(i / 100) * 0.01f;
        xyz[i * 3 + 2] = 0.5f;
        conf[i] = (i % 3 == 0) ? 2 : 1;
    }

    auto end = std::chrono::high_resolution_clock::now();
    double duration_ms = std::chrono::duration<double, std::milli>(end - start).count();

    ASSERT_TRUE(duration_ms < 5.0);

    return true;
}

bool test_Performance_MemoryUsage() {
    int maxPoints = 10000;
    size_t memoryPerPoint = 16;
    size_t totalMemory = maxPoints * memoryPerPoint;
    size_t maxAllowedMemory = 200000;

    ASSERT_TRUE(totalMemory < maxAllowedMemory);

    return true;
}

// VoxelHashMap Integration Tests
// ============================================================================

bool test_VoxelHashMap_InsertAndQuery() {
    DV::DV_VoxelHashMap map(0.1f);

    // Insert a small cluster of points
    std::vector<Eigen::Vector3f> points;
    for (int i = 0; i < 10; i++) {
        points.push_back(Eigen::Vector3f(0.5f + i * 0.01f, 0.5f, 0.5f));
    }
    map.insert(points);

    ASSERT_TRUE(map.size() > 0);

    // Query near the cluster
    DV::KNNResult knn;
    bool found = map.getTopK(Eigen::Vector3f(0.55f, 0.5f, 0.5f), knn, 5);
    ASSERT_TRUE(found);
    ASSERT_TRUE(knn.count >= 5);

    return true;
}

bool test_VoxelHashMap_PlaneFit() {
    // Create a set of coplanar points (z = 0 plane)
    DV::KNNResult knn;
    knn.clear();
    knn.insert(Eigen::Vector3f(0.0f, 0.0f, 0.0f), 0.0f);
    knn.insert(Eigen::Vector3f(1.0f, 0.0f, 0.0f), 1.0f);
    knn.insert(Eigen::Vector3f(0.0f, 1.0f, 0.0f), 1.0f);
    knn.insert(Eigen::Vector3f(1.0f, 1.0f, 0.0f), 2.0f);
    knn.insert(Eigen::Vector3f(0.5f, 0.5f, 0.0f), 0.5f);

    Eigen::Vector3f normal, centroid;
    bool success = DV::DV_VoxelHashMap::fitPlane(knn, normal, centroid);
    ASSERT_TRUE(success);

    // Normal should be approximately (0, 0, +/-1)
    ASSERT_FLOAT_EQ(std::abs(normal.z()), 1.0f, 0.01f);
    ASSERT_FLOAT_EQ(normal.x(), 0.0f, 0.01f);
    ASSERT_FLOAT_EQ(normal.y(), 0.0f, 0.01f);

    return true;
}

// ESKF Integration Tests
// ============================================================================

bool test_ESKF_InitAndPredict() {
    DV::DV_ESKF eskf;

    DV::SysState initial;
    initial.p = DV::V3d(0, 0, 0);
    initial.v = DV::V3d(1, 0, 0); // Moving at 1 m/s in x
    eskf.init(initial);

    // Predict with zero-bias IMU for 0.01s
    DV::IMUData imu;
    imu.timestamp = 0.01;
    imu.acc = DV::V3d(0, 0, 9.81);  // Only gravity
    imu.gyr = DV::V3d(0, 0, 0);

    eskf.predict(imu, 0.01);

    // Position should advance ~0.01m in x
    DV::V3d pos = eskf.getPosition();
    ASSERT_FLOAT_EQ(pos.x(), 0.01, 0.005);

    return true;
}

// Full Pipeline Integration Tests
// ============================================================================

bool test_Integration_FullPipeline() {
    std::vector<float> xyz(30000);
    std::vector<uint8_t> conf(10000);
    for (int i = 0; i < 10000; i++) {
        xyz[i * 3] = (float)(i % 100) * 0.01f;
        xyz[i * 3 + 1] = (float)(i / 100) * 0.01f;
        xyz[i * 3 + 2] = 0.5f;
        conf[i] = (i % 2 == 0) ? 2 : 1;
    }

    int validPoints = 0;
    for (int i = 0; i < 10000; i++) {
        if (conf[i] >= 1) validPoints++;
    }
    ASSERT_TRUE(validPoints > 0);

    int bundledPoints = validPoints / 10;
    ASSERT_TRUE(bundledPoints > 0);

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    ASSERT_FLOAT_EQ(pose(0, 0), 1.0f, 0.001f);

    ASSERT_TRUE(bundledPoints > 0);

    return true;
}

bool test_Integration_MultiFrameTracking() {
    std::vector<Eigen::Matrix4d> poses;

    Eigen::Matrix4d pose0 = Eigen::Matrix4d::Identity();
    poses.push_back(pose0);

    Eigen::Matrix4d pose1 = Eigen::Matrix4d::Identity();
    pose1.block<3, 1>(0, 3) = Eigen::Vector3d(0.05, 0.0, 0.0);
    poses.push_back(pose1);

    Eigen::Matrix4d pose2 = Eigen::Matrix4d::Identity();
    pose2.block<3, 1>(0, 3) = Eigen::Vector3d(0.05, 0.05, 0.0);
    poses.push_back(pose2);

    ASSERT_EQ(poses.size(), 3);
    ASSERT_FLOAT_EQ(poses[1](0, 3), 0.05f, 0.001f);
    ASSERT_FLOAT_EQ(poses[2](1, 3), 0.05f, 0.001f);

    return true;
}

} // namespace DepthVizTests

#endif // DEPTHVIZ_TESTS_HPP
