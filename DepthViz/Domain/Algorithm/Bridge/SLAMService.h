//
//  SLAMService.h
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//  Abstract: Interface for the Fast-LIO2 SLAM system.
//

#import <Foundation/Foundation.h>
#import <ARKit/ARKit.h>
#import <CoreMotion/CoreMotion.h>
#import <Metal/Metal.h>

NS_ASSUME_NONNULL_BEGIN

@protocol SLAMDelegate <NSObject>
- (void)didUpdatePose:(simd_float4x4)pose;
- (void)didUpdatePointCloud:(id<MTLBuffer>)pointCloudBuffer count:(NSUInteger)count;
// New method for Zero-Copy display points (ptr to Point3D struct array)
- (void)didUpdateDisplayPoints:(const void *)points count:(NSUInteger)count;
@end

@interface SLAMService : NSObject

@property (nonatomic, weak) id<SLAMDelegate> delegate;
@property (nonatomic, readonly) BOOL isRunning;
@property (nonatomic, assign) float confidenceThreshold;

+ (instancetype)sharedInstance;

- (void)start;
- (void)stop;
- (void)reloadSettings;

/// Process ARFrame data (Depth + Camera Pose hint)
- (void)processARFrame:(ARFrame *)frame;

/// Process high-frequency IMU data
- (void)processIMUData:(CMDeviceMotion *)motion;

/// Retrieve full optimized map for export (Async to allow copy)
- (void)getMapForExport:(void(^)(const void *points, NSUInteger count))completion;

/// Get current display cloud points for Swift (Returns array of Point3D structs)
- (NSArray<NSNumber *> *)getDisplayCloudAsArray;

@end

NS_ASSUME_NONNULL_END
