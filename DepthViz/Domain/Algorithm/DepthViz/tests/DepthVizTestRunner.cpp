#include "DepthVizTests.hpp"
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "\n" << std::string(50, '=') << std::endl;
    std::cout << "DepthViz Unit Test Suite" << std::endl;
    std::cout << std::string(50, '=') << "\n" << std::endl;

    DepthVizTest::TestSuite suite;

    // Task 1.1: DVPoint3D Structure Tests
    RUN_TEST(suite, DepthVizTests::test_DVPoint3D_Initialization,
             "DVPoint3D Initialization");
    RUN_TEST(suite, DepthVizTests::test_DVPoint3D_DefaultValues,
             "DVPoint3D Default Confidence Values");
    RUN_TEST(suite, DepthVizTests::test_DVPoint3D_TimestampAccuracy,
             "DVPoint3D Timestamp Accuracy");
    RUN_TEST(suite, DepthVizTests::test_DVPoint3D_VoxelIdx,
             "DVPoint3D Voxel Index Computation");

    // Task 2.1: Spatial Hash Tests
    RUN_TEST(suite, DepthVizTests::test_SpatialHash_Voxelization,
             "Spatial Hash Voxelization");
    RUN_TEST(suite, DepthVizTests::test_SpatialHash_CollisionRate,
             "Spatial Hash Collision Rate (<10%)");

    // Task 3.1: Bundle & Discard Tests
    RUN_TEST(suite, DepthVizTests::test_BundleAndDiscard_PointCount,
             "Bundle & Discard Point Reduction (90%)");
    RUN_TEST(suite, DepthVizTests::test_BundleAndDiscard_CentroidAccuracy,
             "Bundle & Discard Centroid (+/-1mm)");
    RUN_TEST(suite, DepthVizTests::test_BundleAndDiscard_ConfidenceWeighting,
             "Bundle & Discard Confidence Filtering");

    // Task 3.2: Robust Kernels Tests
    RUN_TEST(suite, DepthVizTests::test_RobustKernels_ConfidenceWeight,
             "Robust Kernels Multi-Level Gating");
    RUN_TEST(suite, DepthVizTests::test_RobustKernels_TruncatedLeastSquares,
             "Robust Kernels TLS Weight Function");
    RUN_TEST(suite, DepthVizTests::test_RobustKernels_CombinedWeighting,
             "Robust Kernels Combined Weighting");

    // Task 3.3: Pose Estimation Tests
    RUN_TEST(suite, DepthVizTests::test_PoseEstimation_IdentityInitialization,
             "Pose Estimation Identity Init");
    RUN_TEST(suite, DepthVizTests::test_PoseEstimation_PoseComposition,
             "Pose Estimation Composition (T1*T2)");
    RUN_TEST(suite, DepthVizTests::test_PoseEstimation_PoseInverse,
             "Pose Estimation Inverse (T*T_inv)");

    // SO3/SE3 Manifold Tests
    RUN_TEST(suite, DepthVizTests::test_SO3_ExpLog,
             "SO3 Exp/Log Round-trip");
    RUN_TEST(suite, DepthVizTests::test_SE3_Composition,
             "SE3 Composition");
    RUN_TEST(suite, DepthVizTests::test_SE3_Inverse,
             "SE3 Inverse (T*T_inv = I)");

    // Task 4: Keyframe Strategy Tests
    RUN_TEST(suite, DepthVizTests::test_KeyframeStrategy_TranslationThreshold,
             "Keyframe Strategy Translation (5cm)");
    RUN_TEST(suite, DepthVizTests::test_KeyframeStrategy_RotationThreshold,
             "Keyframe Strategy Rotation (2 deg)");
    RUN_TEST(suite, DepthVizTests::test_KeyframeStrategy_NoKeyframeBelow,
             "Keyframe Strategy Skip Below Threshold");

    // Task 5: Performance Tests
    RUN_TEST(suite, DepthVizTests::test_Performance_BundleAndDiscardSpeed,
             "Performance Bundle & Discard Speed (<5ms)");
    RUN_TEST(suite, DepthVizTests::test_Performance_MemoryUsage,
             "Performance Memory Usage (<200KB)");

    // VoxelHashMap Integration Tests
    RUN_TEST(suite, DepthVizTests::test_VoxelHashMap_InsertAndQuery,
             "VoxelHashMap Insert + 5-NN Query");
    RUN_TEST(suite, DepthVizTests::test_VoxelHashMap_PlaneFit,
             "VoxelHashMap SVD Plane Fit");

    // ESKF Tests
    RUN_TEST(suite, DepthVizTests::test_ESKF_InitAndPredict,
             "ESKF Init + IMU Predict");

    // Integration Tests
    RUN_TEST(suite, DepthVizTests::test_Integration_FullPipeline,
             "Integration Full Pipeline");
    RUN_TEST(suite, DepthVizTests::test_Integration_MultiFrameTracking,
             "Integration Multi-Frame Tracking");

    // Print results
    suite.printReport();

    // Exit with proper code
    return suite.allPassed() ? 0 : 1;
}
