//
// Created by xlz on 17-9-23.
//

#ifndef TEST_CARTOGRAPHER_MSG_H
#define TEST_CARTOGRAPHER_MSG_H

#include <iostream>
//#include <tiff.h>
#include <time.h>
#include <sstream>

#include <iomanip>
#include <fstream>
#include <vector>

#include "cartographer/mapping/map_builder.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/sensor/point_cloud.h"

using namespace cartographer::transform;
using ::cartographer::transform::Rigid3d;
using namespace std;

struct Time
{
    int64_t sec;//时间的秒部分
    int64_t nsec;//时间的纳秒部分
};

struct Header
{
    int32 seq;
    Time stamp;
    string frame_id;
};

struct GeometryMsgsVector3
{
    double x;
    double y;
    double z;
};


struct HeometryMsgsQuaternion
{
    double x;
    double y;
    double z;
    double w;
};

struct Imu
{
    Header header;
    HeometryMsgsQuaternion orientation;///积分得到
    GeometryMsgsVector3 linear_acceleration;
    //af:    forward acceleration (m/s^2)
    //al:    leftward acceleration (m/s^2)
    //au:    upward acceleration (m/s^2)
    GeometryMsgsVector3 angular_velocity;
    //wf:    angular rate around forward axis (rad/s)
    //wl:    angular rate around leftward axis (rad/s)
    //wu:    angular rate around upward axis (rad/s)
};

struct Pose
{
    GeometryMsgsVector3 position;
    HeometryMsgsQuaternion orientation;
};

struct Odometry
{
    Header header;
    Pose pose;
    GeometryMsgsVector3 linear;
    //vf:    forward velocity, i.e. parallel to earth-surface (m/s)
    //vl:    leftward velocity, i.e. parallel to earth-surface (m/s)
    //vu:    upward velocity, i.e. perpendicular to earth-surface (m/s)
    GeometryMsgsVector3 angular;
    //wf:    angular rate around forward axis (rad/s)
    //wl:    angular rate around leftward axis (rad/s)
    //wu:    angular rate around upward axis (rad/s)
};


struct LaserScan
{
    Header header;
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    float ranges[803];
};


//数据转换

class MsgBridge {
public:

    explicit MsgBridge(
            int num_subdivisions_per_laser_scan, const string& tracking_frame,
            ::cartographer::mapping::TrajectoryBuilder* trajectory_builder);

    MsgBridge(const MsgBridge&) = delete;
    MsgBridge& operator=(const MsgBridge&) = delete;

    ::cartographer::common::Time change_to_cartographer_time(const Time &stamp);

    std::unique_ptr<::cartographer::sensor::OdometryData> ToOdometryData(const Odometry &msg);
    void HandleOdometryMessage(const string& sensor_id, const Odometry &msg);//
//数据转换 处理laser数据
    void HandleLaserScanMessage(const string &sensor_id, const vector<Eigen::Vector3f>& msg, const Time& time_stamp);

    ::cartographer::sensor::PointCloudWithIntensities ToPointCloudWithIntensities(const LaserScan &msg);

    void HandlePointCloudMessage(const string &sensor_id, const vector<Eigen::Vector3f>& msg ,const Time& time_);

    void SplitString(const string &s, vector<string> &v, const string &c);//字符分割
    //数学转换
//pose(position+orientation)->rigid3
    Rigid3d ToRigid3d(const Pose &pose);

    static Rigid3d create_tf_eigen(double dx, double dy, double dz, double x, double y, double z, double w);

    Eigen::Vector3d ToEigen(const GeometryMsgsVector3 &vector3);

    Eigen::Quaterniond ToEigen(const HeometryMsgsQuaternion &quaternion);

    Rigid3d odom_to_baselink;
    Rigid3d velolink_to_baselink;
private:
    void HandleLaserScan(const string &sensor_id,
                         ::cartographer::common::Time start_time,
                         const string &frame_id,
                         const ::cartographer::sensor::PointCloud &points,
                         double seconds_between_points);

    void HandleRangefinder(const string &sensor_id,
                           ::cartographer::common::Time time,
                           const string &frame_id,
                           const ::cartographer::sensor::PointCloud &ranges);

    const int num_subdivisions_per_laser_scan_;
    ///新版应该指向TrajectoryBuilderInterface
    ::cartographer::mapping::TrajectoryBuilder* const trajectory_builder_;


};

HeometryMsgsQuaternion createQuaternionMsgFromYaw(double yaw);//根据yaw创建四维xyzw
HeometryMsgsQuaternion createQuaternionMsgFromRPY(double Roll, double Pitch, double Yaw);//根据roll pitch yaw计算xyzw



#endif //TEST_CARTOGRAPHER_MSG_H



