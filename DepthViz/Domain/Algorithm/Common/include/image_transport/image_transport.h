#ifndef IMAGE_TRANSPORT_IMAGE_TRANSPORT_H
#define IMAGE_TRANSPORT_IMAGE_TRANSPORT_H

#include "../ros/ros.h"
#include "../sensor_msgs/Image.h"

namespace image_transport {

class Publisher {
public:
    void publish(const sensor_msgs::Image& msg) {}
};

class Subscriber {
public:
    void shutdown() {}
};

class ImageTransport {
public:
    ImageTransport(const ros::NodeHandle& nh) {}
    
    Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false) {
        return Publisher();
    }
    
    Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(*fp)(const sensor_msgs::ImageConstPtr&), void* obj = nullptr) {
        return Subscriber();
    }
};

}

#endif
