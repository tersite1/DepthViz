#ifndef GLOG_LOGGING_H
#define GLOG_LOGGING_H

#include <iostream>
#include <sstream>

namespace google {

enum LogSeverity {
    INFO,
    WARNING,
    ERROR,
    FATAL
};

class LogMessage {
public:
    LogMessage(LogSeverity severity) : severity_(severity) {}
    ~LogMessage() {
        std::cout << stream_.str() << std::endl;
        if (severity_ == FATAL) {
            std::abort();
        }
    }
    std::ostream& stream() { return stream_; }

private:
    LogSeverity severity_;
    std::ostringstream stream_;
};

#define LOG(severity) google::LogMessage(google::severity).stream()

} // namespace google

#endif // GLOG_LOGGING_H
