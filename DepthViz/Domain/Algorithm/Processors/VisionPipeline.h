//
//  VisionPipeline.h
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//  Abstract: OpenCV pipeline for Fast-LIVO2 (VIO) processing.
//

#ifndef VisionPipeline_h
#define VisionPipeline_h

#import <Foundation/Foundation.h>
#import <CoreMedia/CoreMedia.h>
#import <UIKit/UIKit.h>

// Forward declaration for OpenCV cv::Mat
#ifdef __cplusplus
namespace cv {
    class Mat;
}
#endif

@interface VisionPipeline : NSObject

+ (instancetype)sharedInstance;

// Convert CMSampleBuffer (from ARKit) to OpenCV Mat (Grayscale for tracking)
#ifdef __cplusplus
- (cv::Mat)convertSampleBufferToMat:(CVPixelBufferRef)pixelBuffer;
#endif

@end

#endif /* VisionPipeline_h */
