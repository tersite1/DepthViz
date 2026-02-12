#ifndef SENSOR_MSGS_IMAGE_H
#define SENSOR_MSGS_IMAGE_H

#include <vector>
#include <string>
#include <memory>
#include "std_msgs/Header.h"

namespace sensor_msgs {

struct Image {
    using Ptr = std::shared_ptr<Image>;
    using ConstPtr = std::shared_ptr<const Image>;

    std_msgs::Header header;
    uint32_t height;
    uint32_t width;
    std::string encoding;
    uint8_t is_bigendian;
    uint32_t step;
    std::vector<uint8_t> data;
};

using ImagePtr = Image::Ptr;
using ImageConstPtr = Image::ConstPtr;

} // namespace sensor_msgs

#endif // SENSOR_MSGS_IMAGE_H
