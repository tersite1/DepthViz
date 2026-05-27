#ifndef VIKIT_CAMERA_LOADER_H
#define VIKIT_CAMERA_LOADER_H

#include "abstract_camera.h"
#include "pinhole_camera.h"
#include <string>

namespace vk {
namespace camera_loader {

inline bool loadFromRosNs(const std::string& ns, vk::AbstractCamera*& cam) {
    // Stub implementation: Hardcoded camera parameters for testing
    // Assuming 640x480, focal length ~500
    cam = new vk::PinholeCamera(500.0, 500.0, 320.0, 240.0, 640, 480);
    return true;
}

}
}

#endif
