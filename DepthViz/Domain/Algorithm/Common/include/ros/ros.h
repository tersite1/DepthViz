#ifndef ROS_H
#define ROS_H

#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <functional>
// #include <boost/bind.hpp>

namespace ros {

struct Time {
    double sec;
    Time(double s = 0) : sec(s) {}
    static Time now() { return Time(0); }
    double toSec() const { return sec; }
    static Time fromSec(double s) { return Time(s); }
};

struct Duration {
    double sec;
    Duration(double s) : sec(s) {}
};

struct TimerEvent {};

struct Publisher {
    template<typename T>
    void publish(const T& msg) {}
};

struct Subscriber {
    void shutdown() {}
};

struct NodeHandle {
    template<typename T>
    void param(const std::string& key, T& param, const T& default_val) {
        param = default_val;
    }
    
    template<typename T>
    void getParam(const std::string& key, T& param) {
        // No-op or set default if possible, but without value map it's hard.
        // Assuming params are hardcoded/overridden in Engines.
    }

    // Generic subscribe for function pointer
    template<typename F>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size, F fp, void* obj = nullptr) {
        return Subscriber();
    }

    template<typename T>
    Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false) {
        return Publisher();
    }
    
    // Timer creation
    template<typename T, typename O>
    void createTimer(Duration period, void(T::*fp)(const TimerEvent&), O* obj) {}
};

struct TransportHints {
    TransportHints tcpNoDelay() { return *this; }
};

struct SubscribeOptions {
    TransportHints transport_hints;
    template<typename M>
    void init(const std::string& topic, uint32_t queue_size, const std::function<void(const std::shared_ptr<M const>&)>& callback) {}
};

inline void init(int argc, char** argv, const std::string& name) {}
inline bool ok() { return true; }
inline void spinOnce() {}

}

#define ROS_INFO(...) 
#define ROS_WARN(...) 
#define ROS_ERROR(...) 

#endif
