//
//  VisionPipeline.mm
//  DepthViz
//
//  Created by DepthViz Refactoring Agent.
//  Abstract: Implementation of OpenCV conversion logic.
//

#import "VisionPipeline.h"

// Check if OpenCV is available
#if __has_include(<opencv2/core/core.hpp>)
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#else
// Fallback if OpenCV is not linked
namespace cv {
    class Mat {};
}
#endif

@implementation VisionPipeline

+ (instancetype)sharedInstance {
    static VisionPipeline *sharedInstance = nil;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        sharedInstance = [[self alloc] init];
    });
    return sharedInstance;
}

#if __has_include(<opencv2/core/core.hpp>)
- (cv::Mat)convertSampleBufferToMat:(CVPixelBufferRef)pixelBuffer {
    CVPixelBufferLockBaseAddress(pixelBuffer, 0);
    
    int bufferWidth = (int)CVPixelBufferGetWidth(pixelBuffer);
    int bufferHeight = (int)CVPixelBufferGetHeight(pixelBuffer);
    unsigned char *pixel = (unsigned char *)CVPixelBufferGetBaseAddress(pixelBuffer);
    
    // Create Mat from buffer (Assuming YCbCr format, tracking uses Y channel/Grayscale)
    // If format is BGRA, use CV_8UC4
    cv::Mat mat = cv::Mat(bufferHeight, bufferWidth, CV_8UC1, pixel);
    
    // Deep copy if necessary (or return processing result)
    cv::Mat result = mat.clone();
    
    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);
    
    return result;
}
#endif

@end
