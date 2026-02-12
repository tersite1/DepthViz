//
//  SLAMService.mm
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//  Abstract: Objective-C++ Wrapper implementing Strategy Pattern for multiple SLAM engines.
//

#import "SLAMService.h"
#import <simd/simd.h>
#import <vector>
#import <memory>
#import <random>
#import <cmath>

#import <Eigen/Core>
#import <Eigen/Dense>

// Bridge-layer SLAM interface using simd types (iOS-native).
// Distinct from the Eigen-based ISLAMSystem in Interfaces/ISLAMSystem.h.
struct SLAMPoint3D {
    float x, y, z;
    float intensity;
    uint8_t r, g, b;  // RGB color from camera
    double timestamp;
};

class IBridgeSLAMSystem {
public:
    virtual ~IBridgeSLAMSystem() {}
    virtual void init() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void pushIMU(double timestamp, const simd_float3& acc, const simd_float3& gyr) = 0;
    virtual void pushPointCloud(double timestamp, const std::vector<SLAMPoint3D>& points) = 0;
    virtual void pushPointCloudRaw(double timestamp, const float* xyz, const uint8_t* conf, const uint8_t* rgb, int count) {}
    virtual void pushARKitPose(double timestamp, const simd_float4x4& pose) {}
    virtual simd_float4x4 getCurrentPose() = 0;
    virtual std::vector<SLAMPoint3D> getDisplayCloud() = 0;
    virtual std::vector<SLAMPoint3D> getFullMap() { return {}; }
};

// Include DepthVizWrapper (which pulls in Eigen-based ISLAMSystem and Point3D)
#include "../DepthViz/include/DepthVizWrapper.h"

// DepthViz SLAM Adapter — bridges IBridgeSLAMSystem (simd) to DepthVizWrapper (Eigen)
class DepthVizSLAMAdapter : public IBridgeSLAMSystem {
    std::shared_ptr<DepthVizWrapper> _wrapper;

public:
    DepthVizSLAMAdapter() {
        _wrapper = std::make_shared<DepthVizWrapper>();
    }

    void init() override {
        _wrapper->init();
        printf("[DepthVizAdapter] Initialized\n");
    }

    void start() override {
        _wrapper->start();
        printf("[DepthVizAdapter] Started\n");
    }

    void stop() override {
        _wrapper->stop();
        printf("[DepthVizAdapter] Stopped\n");
    }

    void pushIMU(double timestamp, const simd_float3& acc, const simd_float3& gyr) override {
        Eigen::Vector3d acc_eigen(acc.x, acc.y, acc.z);
        Eigen::Vector3d gyr_eigen(gyr.x, gyr.y, gyr.z);
        _wrapper->pushIMU(timestamp, acc_eigen, gyr_eigen);
    }

    void pushPointCloud(double timestamp, const std::vector<SLAMPoint3D>& points) override {
        // Convert SLAMPoint3D -> Point3D (Eigen-based ISLAMSystem type)
        std::vector<Point3D> eigenPoints;
        eigenPoints.reserve(points.size());
        for (const auto& p : points) {
            Point3D ep;
            ep.x = p.x; ep.y = p.y; ep.z = p.z;
            ep.intensity = p.intensity;
            ep.r = p.r; ep.g = p.g; ep.b = p.b;
            ep.timestamp = p.timestamp;
            eigenPoints.push_back(ep);
        }
        _wrapper->pushPointCloud(timestamp, eigenPoints);
    }

    void pushPointCloudRaw(double timestamp, const float* xyz, const uint8_t* conf, const uint8_t* rgb, int count) override {
        _wrapper->pushPointCloudRaw(timestamp, xyz, conf, rgb, count);
    }

    void pushARKitPose(double timestamp, const simd_float4x4& pose) override {
        // Convert simd_float4x4 to Eigen::Matrix4d
        Eigen::Matrix4d eigen_pose;
        for (int col = 0; col < 4; col++) {
            for (int row = 0; row < 4; row++) {
                eigen_pose(row, col) = static_cast<double>(pose.columns[col][row]);
            }
        }
        _wrapper->pushARKitPose(timestamp, eigen_pose);
    }

    simd_float4x4 getCurrentPose() override {
        Eigen::Matrix4d pose = _wrapper->getCurrentPose();
        simd_float4x4 result;
        for (int col = 0; col < 4; col++) {
            for (int row = 0; row < 4; row++) {
                result.columns[col][row] = static_cast<float>(pose(row, col));
            }
        }
        return result;
    }

    std::vector<SLAMPoint3D> getDisplayCloud() override {
        auto eigenCloud = _wrapper->getDisplayCloud();
        std::vector<SLAMPoint3D> result;
        result.reserve(eigenCloud.size());
        for (const auto& ep : eigenCloud) {
            SLAMPoint3D p;
            p.x = ep.x; p.y = ep.y; p.z = ep.z;
            p.intensity = ep.intensity;
            p.r = ep.r; p.g = ep.g; p.b = ep.b;
            p.timestamp = ep.timestamp;
            result.push_back(p);
        }
        return result;
    }

    std::vector<SLAMPoint3D> getFullMap() override {
        auto eigenMap = _wrapper->getFullMap();
        std::vector<SLAMPoint3D> result;
        result.reserve(eigenMap.size());
        for (const auto& ep : eigenMap) {
            SLAMPoint3D p;
            p.x = ep.x; p.y = ep.y; p.z = ep.z;
            p.intensity = ep.intensity;
            p.r = ep.r; p.g = ep.g; p.b = ep.b;
            p.timestamp = ep.timestamp;
            result.push_back(p);
        }
        return result;
    }
};

// ARKit Passthrough Adapter — uses ARKit pose directly, no LIO optimization
// Accumulates points from depth frames using the raw ARKit camera transform.
class ARKitPassthroughAdapter : public IBridgeSLAMSystem {
    simd_float4x4 _currentPose;
    std::vector<SLAMPoint3D> _accumulatedPoints;
    std::vector<SLAMPoint3D> _displayCloud;
    bool _running;
    static const int MAX_DISPLAY_POINTS = 100000;

public:
    ARKitPassthroughAdapter() : _running(false) {
        _currentPose = matrix_identity_float4x4;
    }

    void init() override {
        printf("[ARKit Passthrough] Initialized\n");
    }

    void start() override {
        _running = true;
        _accumulatedPoints.clear();
        _displayCloud.clear();
        printf("[ARKit Passthrough] Started\n");
    }

    void stop() override {
        _running = false;
        printf("[ARKit Passthrough] Stopped — %zu points accumulated\n", _accumulatedPoints.size());
    }

    void pushIMU(double timestamp, const simd_float3& acc, const simd_float3& gyr) override {
        // ARKit passthrough does not use IMU
    }

    void pushPointCloud(double timestamp, const std::vector<SLAMPoint3D>& points) override {
        // Not used — raw data comes via pushPointCloudRaw
    }

    void pushPointCloudRaw(double timestamp, const float* xyz, const uint8_t* conf, const uint8_t* rgb, int count) override {
        if (!_running) return;

        // Transform camera-frame points to world frame using ARKit pose
        for (int i = 0; i < count; i++) {
            float cx = xyz[i * 3];
            float cy = xyz[i * 3 + 1];
            float cz = xyz[i * 3 + 2];

            // Transform by current ARKit pose
            simd_float4 camPt = simd_make_float4(cx, cy, cz, 1.0f);
            simd_float4 worldPt = simd_mul(_currentPose, camPt);

            SLAMPoint3D p;
            p.x = worldPt.x;
            p.y = worldPt.y;
            p.z = worldPt.z;
            p.intensity = (float)conf[i] / 2.0f;
            p.r = rgb[i * 3];
            p.g = rgb[i * 3 + 1];
            p.b = rgb[i * 3 + 2];
            p.timestamp = timestamp;
            _accumulatedPoints.push_back(p);
        }

        // Keep display cloud manageable
        if (_accumulatedPoints.size() > MAX_DISPLAY_POINTS) {
            _displayCloud.assign(
                _accumulatedPoints.end() - MAX_DISPLAY_POINTS,
                _accumulatedPoints.end()
            );
        } else {
            _displayCloud = _accumulatedPoints;
        }
    }

    void pushARKitPose(double timestamp, const simd_float4x4& pose) override {
        _currentPose = pose;
    }

    simd_float4x4 getCurrentPose() override {
        return _currentPose;
    }

    std::vector<SLAMPoint3D> getDisplayCloud() override {
        return _displayCloud;
    }

    std::vector<SLAMPoint3D> getFullMap() override {
        return _accumulatedPoints;
    }
};

@interface SLAMService () {
    std::shared_ptr<IBridgeSLAMSystem> _currentSystem;
    float _distanceLimit;
    float _confidenceThreshold;
    NSString *_currentAlgorithmName;
    std::vector<float> _xyzBuffer;
    std::vector<uint8_t> _confBuffer;
    std::vector<uint8_t> _rgbBuffer;
}
@end

@implementation SLAMService

@synthesize confidenceThreshold = _confidenceThreshold;

+ (instancetype)sharedInstance {
    static SLAMService *sharedInstance = nil;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        sharedInstance = [[self alloc] init];
    });
    return sharedInstance;
}

- (instancetype)init {
    self = [super init];
    if (self) {
        _isRunning = NO;
        _distanceLimit = 1000.0f;
        _confidenceThreshold = 0.0f;  // Default: no filtering
        _currentAlgorithmName = @"DepthViz";
        [self loadAlgorithmPreference];
    }
    return self;
}

- (void)loadAlgorithmPreference {
    NSString *savedAlgo = [[NSUserDefaults standardUserDefaults] stringForKey:@"ScanAlgorithm"];
    if (!savedAlgo) savedAlgo = @"DepthViz";

    float limit = [[NSUserDefaults standardUserDefaults] floatForKey:@"ScanDistanceLimit"];
    if (limit == 0) limit = 1000.0f;

    float confThreshold = [[NSUserDefaults standardUserDefaults] floatForKey:@"ConfidenceThreshold"];
    _confidenceThreshold = confThreshold;  // 0.0 = no filtering, 0-1 range from UI

    [self configureSystemWithAlgorithm:savedAlgo distanceLimit:limit];
}

- (void)reloadSettings {
    [self loadAlgorithmPreference];
}

- (void)configureSystemWithAlgorithm:(NSString *)algorithmName distanceLimit:(float)limit {
    _currentAlgorithmName = algorithmName;
    _distanceLimit = limit;

    if (_isRunning) [self stop];
    _currentSystem.reset();

    NSLog(@"[SLAMService] Configuring Engine: %@ (Limit: %.1fm)", algorithmName, limit);

    if ([algorithmName isEqualToString:@"DepthViz"]) {
        _currentSystem = std::make_shared<DepthVizSLAMAdapter>();
    } else if ([algorithmName isEqualToString:@"ARKit"]) {
        _currentSystem = std::make_shared<ARKitPassthroughAdapter>();
    } else {
        // Default fallback to ARKit passthrough
        _currentSystem = std::make_shared<ARKitPassthroughAdapter>();
    }

    if (_currentSystem) {
        _currentSystem->init();
    }
}

- (void)start {
    if (_isRunning) return;
    if (!_currentSystem) [self loadAlgorithmPreference];

    if (_currentSystem) {
        _currentSystem->start();
        _isRunning = YES;
        NSLog(@"[SLAMService] Engine Started");
    }
}

- (void)stop {
    if (!_isRunning) return;

    if (_currentSystem) {
        _currentSystem->stop();
    }
    _isRunning = NO;
    NSLog(@"[SLAMService] Engine Stopped");
}

- (void)processIMUData:(CMDeviceMotion *)motion {
    if (!_isRunning || !_currentSystem) return;

    double timestamp = motion.timestamp;

    simd_float3 acc = simd_make_float3(
        (float)(motion.gravity.x + motion.userAcceleration.x) * 9.81f,
        (float)(motion.gravity.y + motion.userAcceleration.y) * 9.81f,
        (float)(motion.gravity.z + motion.userAcceleration.z) * 9.81f
    );

    simd_float3 gyr = simd_make_float3(
        (float)motion.rotationRate.x,
        (float)motion.rotationRate.y,
        (float)motion.rotationRate.z
    );

    _currentSystem->pushIMU(timestamp, acc, gyr);
}

- (void)processARFrame:(ARFrame *)frame {
    if (!_isRunning || !_currentSystem) return;

    double timestamp = frame.timestamp;

    // Push ARKit pose as prior
    simd_float4x4 arkit_transform = frame.camera.transform;
    _currentSystem->pushARKitPose(timestamp, arkit_transform);

    // Extract real depth data from ARFrame
    CVPixelBufferRef depthMap = frame.sceneDepth.depthMap;
    CVPixelBufferRef confidenceMap = frame.sceneDepth.confidenceMap;
    CVPixelBufferRef capturedImage = frame.capturedImage;

    if (depthMap && confidenceMap && capturedImage) {
        CVPixelBufferLockBaseAddress(depthMap, kCVPixelBufferLock_ReadOnly);
        CVPixelBufferLockBaseAddress(confidenceMap, kCVPixelBufferLock_ReadOnly);
        CVPixelBufferLockBaseAddress(capturedImage, kCVPixelBufferLock_ReadOnly);

        int depthWidth = (int)CVPixelBufferGetWidth(depthMap);
        int depthHeight = (int)CVPixelBufferGetHeight(depthMap);

        float *depthData = (float *)CVPixelBufferGetBaseAddress(depthMap);
        uint8_t *confData = (uint8_t *)CVPixelBufferGetBaseAddress(confidenceMap);

        size_t depthBytesPerRow = CVPixelBufferGetBytesPerRow(depthMap);
        size_t confBytesPerRow = CVPixelBufferGetBytesPerRow(confidenceMap);

        // Camera image (YCbCr 420 format, 2 planes)
        int camWidth = (int)CVPixelBufferGetWidth(capturedImage);
        int camHeight = (int)CVPixelBufferGetHeight(capturedImage);
        uint8_t *yPlane = (uint8_t *)CVPixelBufferGetBaseAddressOfPlane(capturedImage, 0);
        uint8_t *cbcrPlane = (uint8_t *)CVPixelBufferGetBaseAddressOfPlane(capturedImage, 1);
        size_t yBytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(capturedImage, 0);
        size_t cbcrBytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(capturedImage, 1);

        // Scale factor from depth map to camera image
        float scaleX = (float)camWidth / (float)depthWidth;
        float scaleY = (float)camHeight / (float)depthHeight;

        // Get camera intrinsics (scaled to depth map resolution)
        simd_float3x3 intrinsics = frame.camera.intrinsics;
        float fx = intrinsics.columns[0][0] / scaleX;
        float fy = intrinsics.columns[1][1] / scaleY;
        float cx = intrinsics.columns[2][0] / scaleX;
        float cy = intrinsics.columns[2][1] / scaleY;

        // Subsample every 4th pixel for performance
        int step = 4;
        int maxPoints = (depthWidth / step) * (depthHeight / step);

        _xyzBuffer.resize(maxPoints * 3);
        _confBuffer.resize(maxPoints);
        _rgbBuffer.resize(maxPoints * 3);

        int pointIdx = 0;

        for (int row = 0; row < depthHeight; row += step) {
            float *depthRow = (float *)((uint8_t *)depthData + row * depthBytesPerRow);
            uint8_t *confRow = (uint8_t *)((uint8_t *)confData + row * confBytesPerRow);

            for (int col = 0; col < depthWidth; col += step) {
                float depth = depthRow[col];
                uint8_t confidence = confRow[col];

                // Skip invalid depth
                if (depth <= 0.0f || depth > _distanceLimit || std::isnan(depth)) continue;

                // Apply confidence threshold filtering
                // ARKit confidence: 0=low, 1=medium, 2=high
                // UI threshold: 0.0-1.0 mapped to 0-2 scale
                float confThresholdScaled = _confidenceThreshold * 2.0f;
                if ((float)confidence < confThresholdScaled) continue;

                // Unproject to 3D camera-frame coordinates
                float x = (col - cx) * depth / fx;
                float y = (row - cy) * depth / fy;
                float z = depth;

                // Map depth pixel to camera image pixel
                int camCol = (int)(col * scaleX);
                int camRow = (int)(row * scaleY);
                camCol = std::min(std::max(camCol, 0), camWidth - 1);
                camRow = std::min(std::max(camRow, 0), camHeight - 1);

                // Sample YCbCr and convert to RGB
                uint8_t Y = yPlane[camRow * yBytesPerRow + camCol];
                int cbcrRow = camRow / 2;
                int cbcrCol = (camCol / 2) * 2; // CbCr are interleaved
                uint8_t Cb = cbcrPlane[cbcrRow * cbcrBytesPerRow + cbcrCol];
                uint8_t Cr = cbcrPlane[cbcrRow * cbcrBytesPerRow + cbcrCol + 1];

                // YCbCr to RGB (BT.601)
                int r = (int)(Y + 1.402f * (Cr - 128));
                int g = (int)(Y - 0.344f * (Cb - 128) - 0.714f * (Cr - 128));
                int b = (int)(Y + 1.772f * (Cb - 128));
                r = std::min(std::max(r, 0), 255);
                g = std::min(std::max(g, 0), 255);
                b = std::min(std::max(b, 0), 255);

                if (pointIdx < maxPoints) {
                    _xyzBuffer[pointIdx * 3] = x;
                    _xyzBuffer[pointIdx * 3 + 1] = y;
                    _xyzBuffer[pointIdx * 3 + 2] = z;
                    _confBuffer[pointIdx] = confidence;
                    _rgbBuffer[pointIdx * 3] = (uint8_t)r;
                    _rgbBuffer[pointIdx * 3 + 1] = (uint8_t)g;
                    _rgbBuffer[pointIdx * 3 + 2] = (uint8_t)b;
                    pointIdx++;
                }
            }
        }

        CVPixelBufferUnlockBaseAddress(capturedImage, kCVPixelBufferLock_ReadOnly);
        CVPixelBufferUnlockBaseAddress(confidenceMap, kCVPixelBufferLock_ReadOnly);
        CVPixelBufferUnlockBaseAddress(depthMap, kCVPixelBufferLock_ReadOnly);

        // Push raw point cloud with RGB to engine
        if (pointIdx > 0) {
            _currentSystem->pushPointCloudRaw(timestamp, _xyzBuffer.data(), _confBuffer.data(), _rgbBuffer.data(), pointIdx);
        }
    } else {
        // No depth data available
        std::vector<SLAMPoint3D> dummy;
        _currentSystem->pushPointCloud(timestamp, dummy);
    }

    // Retrieve output
    simd_float4x4 pose = _currentSystem->getCurrentPose();

    if (self.delegate && [self.delegate respondsToSelector:@selector(didUpdatePose:)]) {
        [self.delegate didUpdatePose:pose];
    }

    // Display Cloud Update
    std::vector<SLAMPoint3D> displayCloud = _currentSystem->getDisplayCloud();
    if (!displayCloud.empty()) {
        if (self.delegate && [self.delegate respondsToSelector:@selector(didUpdateDisplayPoints:count:)]) {
            [self.delegate didUpdateDisplayPoints:displayCloud.data() count:displayCloud.size()];
        }
    }
}

- (void)getMapForExport:(void(^)(const void *points, NSUInteger count))completion {
    if (!_currentSystem) {
        completion(nullptr, 0);
        return;
    }

    std::vector<SLAMPoint3D> map = _currentSystem->getFullMap();
    if (map.empty()) {
        completion(nullptr, 0);
    } else {
        completion(map.data(), map.size());
    }
}

- (NSArray<NSNumber *> *)getDisplayCloudAsArray {
    if (!_currentSystem) {
        return @[];
    }

    std::vector<SLAMPoint3D> displayCloud = _currentSystem->getDisplayCloud();

    if (displayCloud.empty()) {
        return @[];
    }

    NSMutableArray<NSNumber *> *array = [NSMutableArray arrayWithCapacity:displayCloud.size() * 5];

    for (const auto& point : displayCloud) {
        [array addObject:@(point.x)];
        [array addObject:@(point.y)];
        [array addObject:@(point.z)];
        [array addObject:@(point.intensity)];
        [array addObject:@(point.timestamp)];
    }

    return array;
}

@end
