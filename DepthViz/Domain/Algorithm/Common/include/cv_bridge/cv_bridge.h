#ifndef CV_BRIDGE_CV_BRIDGE_H
#define CV_BRIDGE_CV_BRIDGE_H

#include <opencv2/core/core.hpp>
#include "../sensor_msgs/Image.h"
#include <memory>
#include <string>

namespace cv_bridge {

struct CvImage {
    using Ptr = std::shared_ptr<CvImage>;
    using ConstPtr = std::shared_ptr<const CvImage>;
    
    std_msgs::Header header;
    std::string encoding;
    cv::Mat image;
    
    sensor_msgs::ImagePtr toImageMsg() const {
        return std::make_shared<sensor_msgs::Image>();
    }
};

inline CvImage::Ptr toCvCopy(const sensor_msgs::ImageConstPtr& source, const std::string& encoding = "") {
    CvImage::Ptr ptr = std::make_shared<CvImage>();
    ptr->header = source->header;
    ptr->encoding = encoding;
    // Stub: No actual data conversion in mock
    return ptr;
}

}

#endif
