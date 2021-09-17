//
// Created by xlz on 17-9-27.
//

#ifndef TEST_CARTOGRAPHER_NODE_H
#define TEST_CARTOGRAPHER_NODE_H

#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/common/mutex.h"
#include "cartographer/mapping/pose_extrapolator.h"

#include <zcm/zcm.h>
#include "zcm/zcm-cpp.hpp"

#include "msg.h"
#include "node_option.h"
#include "map_builder_bridge.h"
#include "map_offset.h"
#include <stdio.h>
#include <thread>
#include <termios.h>
#include "virtualscan.h"
#include "image_fusion.h"
#include "ParameterReader.h"
#include "common/nature.h"
#include <mutex>
#include "linux/input.h"
#include <fcntl.h>
#include <boost/thread/thread.hpp>
#include <pthread.h>

namespace TiEV{

    enum Mode{
        ABOVE_GROUND_LOCALIZATION = 1,
        ABOVE_GROUND__MAPPING = 2,
        UNDER_GROUND_LOCALIZATION = 3,
        UNDER_GROUND_MAPPING = 4,
    };

    class PerceptionNode {

    public:
    //initial pose
    bool BoolLoadMap;
    bool BoolInitialPose;
    bool BoolGlobalLocation;
    double initial_translation[3];//x y z
    double initial_rotation[3];
    Mode mode_;
public:

    PerceptionNode(const NodeOptions& node_options , int mode);

    ~PerceptionNode();

        PerceptionNode(const PerceptionNode&) = delete;
        //PerceptionNode& operator=(const PerceptionNode&) = delete;

    // Finishes all yet active trajectories.
    void FinishAllTrajectories();
    // Finishes a single given trajectory.
    void FinishTrajectory(int trajectory_id);

    // Runs final optimization. All trajectories have to be finished when calling.
    void RunFinalOptimization();

    // Starts the first trajectory with the default topics.
    void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);

    // Compute the default topics for the given 'options'.
    std::unordered_set<string> ComputeDefaultTopics(const TrajectoryOptions& options);

    // Returns the set of topic names we want to subscribe to.
    std::unordered_set<string> ComputeExpectedTopics(const TrajectoryOptions& options);

    // Adds a trajectory for offline processing, i.e. not listening to topics.
    int AddOfflineTrajectory(
        const std::unordered_set<string>& expected_sensor_ids,
        const TrajectoryOptions& options);

    // The following functions handle adding sensor data to a trajectory.
    /*
    void HandleOdometryMessage(int trajectory_id, const string& sensor_id,
                               const nav_msgs::Odometry::ConstPtr& msg);
    void HandleImuMessage(int trajectory_id, const string& sensor_id,
                          const sensor_msgs::Imu::ConstPtr& msg);
    void HandleLaserScanMessage(int trajectory_id, const string& sensor_id,
                                const sensor_msgs::LaserScan::ConstPtr& msg);
    void HandleMultiEchoLaserScanMessage(
            int trajectory_id, const string& sensor_id,
            const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
    void HandlePointCloud2Message(int trajectory_id, const string& sensor_id,
                                  const sensor_msgs::PointCloud2::ConstPtr& msg);*/
/**这个函数的作用输出全局位姿*/
    Rigid3d PublishTrajectoryStates();

    // Serializes the complete Node state.
    void SerializeState(const string& filename);

    // Loads a persisted state to use as a map.
    void LoadMap(const std::string& map_filename);

    cv::Mat GetImageByPose(double reckonx , double reckony ,double yaw,  int rows, int cols, int cen_row, int cen_col, float reso ,int search_num ,float threshold);

    //thread for fusion map by JOHN2017.10
    static void * GenerateFusionMap(void*);
    static void * GenerateSLAMLocation(void*);
    static void * PublishNavInfo(void*);

public:
   // void AddOdomData(const string& sensor_id , const Odometry& odom , int trajectory_id);
    void AddScanData(const string& sensor_id , const vector<Eigen::Vector3f>& msg ,const int64_t& sec,const int64_t& nsec , int trajectory_id);
    void AddPointCloudData(const string& sensor_id , const vector<Eigen::Vector3f>& msg ,const int64_t& sec,const int64_t& nsec , int trajectory_id);
    int AddTrajectory(const TrajectoryOptions& options,
      std::unordered_set<string> expected_topics);

    void AddData(int trajectory_id);
    void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
    bool ValidateTrajectoryOptions(const TrajectoryOptions& options);

    const NodeOptions node_options_;

    cartographer::common::Mutex mutex_;
    MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);
    //zz
    MsgBridge* msg_bridge;

    // These are keyed with 'trajectory_id'.
    std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
    std::unordered_map<int, bool> is_active_trajectory_ GUARDED_BY(mutex_);

};

}

#endif //TEST_CARTOGRAPHER_NODE_H
