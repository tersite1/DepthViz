#ifndef MSGS_H
#define MSGS_H

#include <string>
#include <vector>
#include <memory>
#include <cmath>

typedef unsigned long int uli;
typedef unsigned long long int ulli;
typedef unsigned short int usi;

namespace custom_messages
{
    struct Time
    {
        Time() = default;
        Time(Time const&) = default;
        
        void normalizeSecNSec(unsigned long& sec, unsigned long& nsec){
            unsigned long nsec_part= nsec % 1000000000UL;
            unsigned long sec_part = nsec / 1000000000UL;
            sec += sec_part;
            nsec = nsec_part;
        }

        unsigned long toNsec() const { return (unsigned long)sec*1000000000ull + (unsigned long)nsecs; };
        Time& fromSec(double t) 
        { 
            sec = (unsigned long) floor(t); 
            nsecs = (unsigned long) round((t-sec) * 1e9);
            normalizeSecNSec(sec, nsecs);
            return *this; 
        };
        double toSec() const { return (double)sec + 1e-9*(double)nsecs; };

        uli sec;
        uli nsecs;
    };

    // This represents an orientation in free space in quaternion form.
    struct Quaternion
    {
        Quaternion() = default;
        Quaternion(Quaternion const&) = default;

        double x;
        double y;
        double z;
        double w;
    };

    struct Vector3
    {
        Vector3() = default;
        Vector3(Vector3 const&) = default;

        double x;
        double y;
        double z;
    };

    // This contains the position of a point in free space
    struct Point
    {
        Point() = default;
        Point(Point const&) = default;

        double x;
        double y;
        double z;
    };

    struct PointField
    {
        PointField() = default;
        PointField(PointField const&) = default;

        const usi INT8 = 1;
        const usi UINT8 = 2;
        const usi INT16 = 3;
        const usi UINT16 = 4;
        const usi INT32 = 5;
        const usi UINT32 = 6;
        const usi FLOAT32 = 7;
        const usi FLOAT64 = 8;

        std::string name;
        uli offset;
        usi datatype;
        uli count;
    };

    // Standard metadata for higher-level stamped data types.
    struct Header
    {
        Header() = default;
        Header(Header const&) = default;

        uli seq;
        Time stamp;
        std::string frame_id;
    };

    struct Pose
    {
        Pose() = default;
        Pose(Pose const&) = default;

        Point position;
        Quaternion orientation;
    };

    struct Twist
    {
        Twist() = default;
        Twist(Twist const&) = default;

        Vector3 linear;
        Vector3 angular;
    };

    // This represents a pose in free space with uncertainty.
    struct PoseWithCovariance
    {
        PoseWithCovariance() = default;
        PoseWithCovariance(PoseWithCovariance const&) = default;
        PoseWithCovariance& operator=(PoseWithCovariance other)
        {
            pose = other.pose;
            for (int i = 0; i < 36; ++i)
                covariance[i] = other.covariance[i];
            
            return *this;
        }

        Pose pose;
        double covariance[36];
    };

    struct PoseStamped
    {
        PoseStamped() = default;
        PoseStamped(PoseStamped const&) = default;

        Header header;
        Pose pose;
    };

    struct TwistWithCovariance
    {
        TwistWithCovariance() = default;
        TwistWithCovariance(TwistWithCovariance const&) = default;

        TwistWithCovariance& operator=(TwistWithCovariance other)
        {
            twist = other.twist;
            for (int i = 0; i < 36; ++i)
                covariance[i] = other.covariance[i];
            
            return *this;
        }

        Twist twist;
        double covariance[36];
    };

    // An array of poses that represents a Path for a robot to follow
    // struct Path
    // {
    //     Header header;
    //     PoseStamped *poses;
    // };

    struct Pose6D
    {
        Pose6D() = default;
        Pose6D(Pose6D const&) = default;

        Pose6D& operator=(Pose6D other)
        {
            offset_time = other.offset_time;
            for (int i = 0; i < 3; ++i)
            {
                acc[i] = other.acc[i];
                gyr[i] = other.gyr[i];
                vel[i] = other.vel[i];
                pos[i] = other.pos[i];
            }
            for (int i = 0; i < 9; ++i)
                rot[i] = other.rot[i];
            
            return *this;
        }

        double offset_time;  // the offset time of IMU measurement w.r.t the first lidar point
        double acc[3];       // the preintegrated total acceleration (global frame) at the Lidar origin
        double gyr[3];       // the unbiased angular velocity (body frame) at the Lidar origin
        double vel[3];       // the preintegrated velocity (global frame) at the Lidar origin
        double pos[3];       // the preintegrated position (global frame) at the Lidar origin
        double rot[9];       // the preintegrated rotation (global frame) at the Lidar origin
    };

    struct Imu
    {
        Imu() = default;
        Imu(Imu const&) = default;

        Imu& operator=(Imu other)
        {
            header = other.header;
            orientation = other.orientation;
            for (int i = 0; i < 9; ++i)
            {
                orientation_covariance[i] = other.orientation_covariance[i];
                angular_velocity_covariance[i] = other.angular_velocity_covariance[i];
                linear_acceleration_covariance[i] = other.linear_acceleration_covariance[i];
            }

            angular_velocity = other.angular_velocity;
            linear_acceleration = other.linear_acceleration;

            return *this;
        }

        Header header;
        Quaternion orientation;
        double orientation_covariance[9];
        Vector3 angular_velocity;
        double angular_velocity_covariance[9];
        Vector3 linear_acceleration;
        double linear_acceleration_covariance[9];
    };

    struct Odometry
    {
        Odometry() = default;
        Odometry(Odometry const&) = default;

        Header header;
        std::string child_frame_id;
        PoseWithCovariance pose;
        TwistWithCovariance twist;
    };

    // This message holds a collection of N-dimensional points, which may
    // contain additional information such as normals, intensity, etc. The
    // point data is stored as a binary blob, its layout described by the
    // contents of the "fields" array.
    struct PointCloud2
    {
        PointCloud2() = default;
        PointCloud2(PointCloud2 const&) = default;

        PointCloud2& operator=(PointCloud2 other)
        {
            header = other.header;
            height = other.height;
            width = other.width;
            fields = std::move(other.fields);
            is_bigendian = other.is_bigendian;
            point_step = other.point_step;
            row_step = other.row_step;
            data = std::move(other.data);
            is_dense = other.is_dense;

            return *this;
        }

        Header header;
        uli height;
        uli width;
        std::vector<PointField> fields;
        bool is_bigendian;
        uli point_step;
        uli row_step;
        std::vector<usi> data;
        bool is_dense;
    };

    struct CustomPoint
    {
        uli offset_time; // offset time relative to the base time
        double x; // X axis, unit:m
        double y; // Y axis, unit:m
        double z; // Z axis, unit:m
        usi reflectivity; // reflectivity, 0~255
        usi tag; // livox tag
        usi line; // laser number in lidar
    };
    

    struct CustomMsg
    {
        Header header;
        ulli timebase;                          // The time of first point
        uli point_num;                          // Total number of pointclouds
        usi lidar_id;                           // Lidar device id number
        usi rsvd[3];                            // Reserved use
        std::vector<CustomPoint> points;        // Pointcloud data
    };

    typedef std::shared_ptr<Imu const> ImuConstPtr;
    typedef std::shared_ptr<Imu> ImuPtr;
    typedef std::shared_ptr<PointCloud2 const> PC2ConstPtr;
    typedef std::shared_ptr<Odometry> OdomMsgPtr;
    typedef std::shared_ptr<CustomMsg const> CstMsgConstPtr;
}

#endif