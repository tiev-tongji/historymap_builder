//
// Created by xlz on 17-9-27.
//

#define THE_SEARCH_THS 0.7//搜索阈值
#include "node.h"
#include "msg.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/sensor/point_cloud.h"
#include "map_offset.h"
#include "serial.h"
#include "gps_processing.h"
#include "WGS84UTM.h"
#include "coordinate_converter/coordinate_converter.h"

using namespace cartographer::transform;
using ::cartographer::transform::Rigid3d;

//zz
#include "MsgNavInfoSignal.hpp"
#include "MsgHistoricalMap.hpp"
#include "Msg40LidarFrameList.hpp"
#include "MsgObjectTrackingList.hpp"
#include "MsgCanInfoSignal.hpp"
#include "MsgPredictedObjectTrajectoryList.hpp"
#include "MsgRTK.hpp"
zcm::ZCM pubzcm{"ipc"};
int64_t laser_map_num=0, laser_map_num_old=0;
int frame_count = 0;
ParameterReader reader("../parameters.txt");
bool UpdateMapFlag = atoi(reader.getData("UpdateMapFlag").c_str());
bool debug_show_map = atoi(reader.getData("debug_show_map").c_str());

float ths = atof(reader.getData("ths").c_str());
const float IMU_LASER_DISTANCE =0.83;
Msg40LidarFrameList msg_lidar_temp;
std::mutex lidar_mtx;
constexpr uint32_t GpsAndSystemDiffSeconds = 315964800; // 1980-01-06 vs 1970-01-01
constexpr uint32_t SecondsPerWeek = 60 * 60 * 24 * 7;

constexpr double AngleBetweenNorthAndMag = 1.5 / 180 * 3.14159265358979;
constexpr double ImuToFrontAxisX = 3.175;
constexpr double ImuToFrontAxisY = -0.24;

const std::string GPCHC_BEGIN_CHAR = "$";
const std::string GPCHC_ENDLINE = "\r\n"; // 结束符
bool BoolStopMapping = false;
int odom_num = 0;
bool FirstHeading;
double first_heading;
GpsData gps;
std::mutex gps_mtx;
std::mutex nav_mtx;
Imu imu_from_port;
MsgNavInfoSignal msg_nav_temp;
MsgRTK msg_rtk;
namespace TiEV{
//void SubscribeGpsAndImu(void* __this) {
//    //*****JR 2020-05-14********//
//    PerceptionNode *_this = (PerceptionNode *) __this;
//    //*****JR 2020-05-14********//
//
//    serial::Serial ser;
//    GpsData gps_temp;
//    try {
//        ser.setPort("/dev/ttyUSB1");
//        ser.setBaudrate(460800);
//        serial::Timeout time_out = serial::Timeout::simpleTimeout(1000); // 1second
//        ser.setTimeout(time_out);
//        ser.open();
//    }
//    catch (serial::IOException &err) {
//        cerr << "Unable to open port " << "\n";
//    }
//    // 检测串口是否已经打开，并给出提示信息
//    if (ser.isOpen()) {
//        cout << "Serial Port initialized" << "\n";
//    } else {
//        cerr << "[FATAL]~~~~~~~~Serial Port uninitialized" << "\n";
//    }
//
//    string buffer; // 读取的串口数据
//    string buff_string; // 缓存区
//    size_t max_size = 65536;
//
//    while (!BoolStopMapping) {
//        while (ser.available()) {
//            buffer = ser.readline(max_size, GPCHC_ENDLINE);
//            buff_string += buffer;
//            int index = 0, start_index = -1, end_index = -1;
//            while (index < buff_string.length()) {
//                start_index = buff_string.find(GPCHC_BEGIN_CHAR);
//                if (start_index == -1) {
//                    cout << "datas in this time is useless" << "\n";
//                    buff_string.clear();
//                    break;
//                } // 没有起始位,常出现于刚开始,丢弃所有数据
//                else // 找到起始位
//                {
//                    end_index = buff_string.find(GPCHC_ENDLINE);
//                    if (end_index == -1) { // 有起始位没有结束位
//                        buff_string = buff_string.substr(start_index); // 留下后半段
//                        break;
//                    } else { // 有起始位也有结束位
//                        index = end_index;
//                        string buff_substr = buff_string.substr(start_index, end_index - start_index + 2);
//                        int valid_gps_data = gps_temp.GpsProcessing(buff_substr, gps_temp);
//                        if (valid_gps_data > 0) // 正确解析数据
//                        {
//                            gps_mtx.lock();
//                            gps = gps_temp;
//                            gps_mtx.unlock();
//                            if (!FirstHeading) {
//                                if(!_this->BoolInitialPose) {
//                                    cout << "setting initial pose to utm coordinate>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << "\n";
//                                    TiEV::Angle angle_lon, angle_lat,heading;
//                                    angle_lon.setByDegree(gps_temp.longitude);
//                                    angle_lat.setByDegree(gps_temp.latitude);
//                                    TiEV::WGS84Coor latlon_point((TiEV::LAT) angle_lat, (TiEV::LON) angle_lon);
//                                    TiEV::UTMCoor utm_point = TiEV::latLonToUTMXY(latlon_point);
//                                    _this->initial_translation[0] = utm_point.x;
//                                    _this->initial_translation[1] = utm_point.y;
//                                    _this->initial_translation[2] = 0;
//                                    LOG(INFO) << "utm:::::::::::::::::::::::::::::" << utm_point.x << "," << utm_point.y << "\n";
//                                    if (gps_temp.heading > 270 && gps_temp.heading < 360) {
//                                        heading.setByDegree(450 - gps_temp.heading);
//                                        _this->initial_rotation[2] = heading.getRad();
//                                        first_heading = heading.getRad();
//                                    } else {
//                                        heading.setByDegree(90 - gps_temp.heading); // north to east as zero, counterclockwise, -180~180 degree
//                                        _this->initial_rotation[2] = heading.getRad();
//                                        first_heading = heading.getRad();
//                                    }
//                                    _this->BoolInitialPose = true;
//                                }
//                                FirstHeading = true;
//                            }
//
//                        }
//                        /**不管解析的gps数据是否发布,都应该从缓存中剔除,进入到这个条件分支下的缓存区还是原来的状态,
//                         **没有减小,如果缓存中去掉此次gps的字符串还有剩余字符,则取子串,再进入while循环判断,
//                         **否则清空缓存区跳出循环.因此,不需要检查缓存的大小模块,缓存不会超标*/
//                        if (index + 2 < buff_string.length()) {
//                            buff_string = buff_string.substr(end_index + 2);
//                            index = 0;
//                            start_index = -1;
//                            end_index = -1; // reset
//                            continue;
//                        } else {
//                            buff_string.clear();
//                            break;
//                        }
//                    }
//                }
//            }
//        }
//        usleep(1000); /// cjf:放在这一层,如果串口发比收快,则里面的循环不休息,如果发慢,则跳出里层循环,在这里休息
//    }
//    ser.close();
//}







/**LASERMAP SICKMAP pointcloud区别是不同的雷达所建的地图，格式都相同，FusionMap是几个融合的结果*

    mutex LASER_mtx;
    mutex SICK_mtx;
//    mutex LUX_mtx;
    mutex OBJLIST_mtx;
    mutex PointCloud_mtx;

    mutex NAVI_mtx;
    mutex FUSION_mtx;
/**memcpy函数，从源内存地址复制特定长度(字节)里面的内容至目标内存地址*/
    class Handler
    {
    public:
        Handler(const TrajectoryOptions& options,int trajectory_id, void* __this)
                :options_(options),trajectory_id_(trajectory_id){
            _this = (PerceptionNode*)__this;
        }
        ~Handler() {}
        /**处理各传感器消息，利用zcm传递，在互斥锁保护下赋给temp*/
        /**类似于ros中的回调函数，可以在其中对接收到的消息进行二次处理*/

        void HandleMsg40LidarFrameList(const zcm::ReceiveBuffer* rbuf,
                                       const std::string& channel, const Msg40LidarFrameList *msg)
        {
            lidar_mtx.lock();
            msg_lidar_temp = *msg;
            lidar_mtx.unlock();
                vector<Eigen::Vector3f> pointcloud;
                int64_t sec_from_40_lidar;
                int64_t nsec_from_40_lidar;

                for(auto &iter_point : msg_lidar_temp.frame){
                    /// 坐标系逆时针转180°,front-x,left-y,up-z by cjf

                    pointcloud.emplace_back(- iter_point.point_x,- iter_point.point_y,iter_point.point_z);
                }

                sec_from_40_lidar = msg_lidar_temp.time_stamp/pow(10,6);
                long double time_all = msg_lidar_temp.time_stamp;
                nsec_from_40_lidar = (time_all /pow(10,6) - sec_from_40_lidar) * pow(10,9);
//            cout << "second is: " << sec_from_40_lidar << " and nsecond is: " << nsec_from_40_lidar << "\n";

                //_this->AddPointCloudData("points2",pointcloud,sec_from_40_lidar,nsec_from_40_lidar,trajectory_id_);

        }


        void HandleMsgCanInfoSignal(const zcm::ReceiveBuffer* rbuf,
                                    const std::string& channel, const MsgCanInfoSignal *msg) {

        }






        void HandleMsgRTK(const zcm::ReceiveBuffer *rbuf,
                          const std::string &channel, const MsgRTK *msg) {
//LOG(INFO)<<"1"<<std::endl;
            gps_mtx.lock();
//LOG(INFO)<<"2"<<std::endl;
//        memcpy(&msg_rtk, msgrtk, sizeof(*msgrtk));
            msg_rtk = *msg;

//            LOG(INFO)<<msg->latitude<<","<<msg->longitude<<"\n";
//            LOG(INFO)<<msg_rtk.nsec<<"\n";
            gps.latitude = msg_rtk.latitude;
            gps.longitude = msg_rtk.longitude;
            gps.altitude = msg_rtk.altitude;
//            LOG(INFO)<<msg_rtk.latitude<<","<<msg_rtk.longitude<<"\n";
            gps.acceleration_x = msg_rtk.acceleration_x;
            gps.acceleration_y = msg_rtk.acceleration_y;
            gps.acceleration_z = msg_rtk.acceleration_z;
            gps.angular_x = msg_rtk.angular_vel_x;
            gps.angular_y = msg_rtk.angular_vel_y;
            gps.angular_z = msg_rtk.angular_vel_z;
            gps.heading = msg_rtk.angle_heading;
            gps_mtx.unlock();
            if (!FirstHeading) {
                if (!_this->BoolInitialPose) { /// 设置utm初始值
                    cout << "setting initial pose to utm coordinate>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << "\n";
                    TiEV::Angle angle_lon, angle_lat, heading;
                    angle_lon.setByDegree(msg_rtk.longitude);
                    angle_lat.setByDegree(msg_rtk.latitude);
                    TiEV::WGS84Coor latlon_point((TiEV::LAT) angle_lat, (TiEV::LON) angle_lon);
                    TiEV::UTMCoor utm_point = TiEV::latLonToUTMXY(latlon_point);

//                    _this->initial_translation[0] = utm_point.x;
//                    _this->initial_translation[1] = utm_point.y;
                    LOG(INFO) << "utm:::::::::::::::::::::::::::::" << utm_point.x << "," << utm_point.y << "\n";
//                    _this->initial_translation[2] = 0;
                    if (msg_rtk.angle_heading > 270 && msg_rtk.angle_heading < 360) {
                        heading.setByDegree(450 - msg_rtk.angle_heading);
//                        _this->initial_rotation[2] = heading.getRad();
                        first_heading = heading.getRad();
                    } else {
                        heading.setByDegree(
                                90 - msg_rtk.angle_heading); // north to east as zero, counterclockwise, -pi~pi
//                        _this->initial_rotation[2] = heading.getRad();
                        first_heading = heading.getRad();
                    }
                    LOG(INFO) << "initial yaw:::::::::::::::::::::::::::::" << _this->initial_rotation[2] << " rad"
                              << "\n";
                    _this->BoolInitialPose = true;
                }

                FirstHeading = true;
            }
            imu_from_port.header.stamp.sec = msg_rtk.sec - 18;
            imu_from_port.header.stamp.nsec = msg_rtk.nsec;
            /// by cjf transform imu to x_front-y_left-z_up
            imu_from_port.angular_velocity.x = msg_rtk.angular_vel_y;
            imu_from_port.angular_velocity.y = -msg_rtk.angular_vel_x;
            imu_from_port.angular_velocity.z = msg_rtk.angular_vel_z;
            imu_from_port.linear_acceleration.x = msg_rtk.acceleration_y;
            imu_from_port.linear_acceleration.y = -msg_rtk.acceleration_x;
            imu_from_port.linear_acceleration.z = msg_rtk.acceleration_z;

//            if (_this->node_options_.map_builder_options.use_trajectory_builder_3d() ||
//                (_this->node_options_.map_builder_options.use_trajectory_builder_2d() &&
//                 options_.trajectory_builder_options.trajectory_builder_2d_options()
//                         .use_imu_data())) {
//                _this->AddImuData(trajectory_id_, "imu", imu_from_port);
//            }
        }

        const TrajectoryOptions& options_;
        int trajectory_id_;
        PerceptionNode* _this;
};

/**主函数*/
void ZcmFunction(const TrajectoryOptions& options,int trajectory_id, void* __this)///对下面操作的封装，注意zcm的启动，消息队列(传感器至程序)的发布不在程序中
{
    PerceptionNode* _this = (PerceptionNode*)__this;
    zcm::ZCM zcm_ipc{"ipc"};///进程间通信

    if (!zcm_ipc.good())
    {
        cout << "zcm init error!" << endl;
        return;
    }

    Handler HandlerObject(options,trajectory_id,_this);
    zcm_ipc.subscribe("RTK", &Handler::HandleMsgRTK, &HandlerObject);
    zcm_ipc.subscribe("Msg40LidarFrameList", &Handler::HandleMsg40LidarFrameList, &HandlerObject);
    zcm_ipc.subscribe("MsgCanInfoSignal", &Handler::HandleMsgCanInfoSignal, &HandlerObject);

    cout << "step11111111111111111111111111" << "\n";
    zcm_ipc.start();
    while (!BoolStopMapping){
//        cout << " data has been received! " << endl;
        sleep(1);
    }
    cout << "No more data has been received! " << endl;
    zcm_ipc.stop();
}

// SLAM模式切换，GUI窗口显示
void KeyRecord()
{
    while(true)
    {
        cv::Mat captureMat(200, 200, CV_8UC1);
        cv::namedWindow("keycapture=f");
        cv::imshow("keycapture=f", captureMat);
        char change = cv::waitKey(1);
        //char ch;
        if(change == 'f')
        {

            cout<< "=-=======================================================BoolStopMapping is modified: "<<change <<endl;
            BoolStopMapping = true;
        }
        //sleep(0.01);
        usleep(10000);
    }
}
///构造函数初始化一些变量(设置)默认值
PerceptionNode::PerceptionNode(const NodeOptions& node_options , int mode)
    : node_options_(node_options), map_builder_bridge_(node_options_) {
        cartographer::common::MutexLocker lock(&mutex_);

        for (int i = 0; i < 3; ++i)
        {
            initial_translation[i] = 0.0;
            initial_rotation[i] = 0.0;
        }
        initial_rotation[3] = 0.0;
        BoolLoadMap = false;
        BoolInitialPose = false;
        BoolGlobalLocation = false;

        if ( mode == 1)
            mode_ = Mode::ABOVE_GROUND_LOCALIZATION;
        if ( mode == 2)
            mode_ = Mode::ABOVE_GROUND__MAPPING;
        if ( mode == 3)
            mode_ = Mode::UNDER_GROUND_LOCALIZATION;
        if ( mode == 4)
            mode_ = Mode::UNDER_GROUND_MAPPING;
        if (!pubzcm.good())
        {
            cout<<"zcm for publishing is not good";
            return;
        }

        //TODO 新增模式后的逻辑如何修改
        if ( mode == 1){
            std::thread nav_thread(PublishNavInfo, (void*) this);
            nav_thread.detach();
            std::thread map_thread(GenerateFusionMap, (void*) this);
            map_thread.detach();
        }

        if ( mode == 3){
            std::thread pub_thread(GenerateSLAMLocation, (void*) this);
            pub_thread.detach();
        }

//        std::thread GpsAndImu_thread(&SubscribeGpsAndImu,(void*) this);
//        GpsAndImu_thread.detach();
        if(!debug_show_map)
        {
            std::thread key_record_thread(KeyRecord);
            key_record_thread.detach();
        }
        //TODO 226~242行需要修改
    BoolStopMapping = false;
    }

PerceptionNode::~PerceptionNode() {}

void PerceptionNode::AddExtrapolator(const int trajectory_id, const TrajectoryOptions& options) {
    constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
    CHECK(extrapolators_.count(trajectory_id) == 0);
    const double gravity_time_constant =
            node_options_.map_builder_options.use_trajectory_builder_3d()
            ? options.trajectory_builder_options.trajectory_builder_3d_options()
                    .imu_gravity_time_constant()
            : options.trajectory_builder_options.trajectory_builder_2d_options()
                    .imu_gravity_time_constant();
    extrapolators_.emplace(
            std::piecewise_construct, std::forward_as_tuple(trajectory_id),
            std::forward_as_tuple(
                    ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
                    gravity_time_constant));
}

void PerceptionNode::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
    //cartographer::common::MutexLocker lock(&mutex_);
    CHECK(ValidateTrajectoryOptions(options));
    //AddTrajectory  expected_sensor_ids set here
    const std::unordered_set<string> expected_sensor_ids =
            ComputeExpectedTopics(options);
    /// 可以帮助看到此次slam所用的topics
    cout << "check out -> Need input data :  ";
    for(const auto& iterator : expected_sensor_ids)
        std::cout << iterator << " ; "<<endl;

    ///下面有两个函数的具体实现
    int trajectory_id = AddTrajectory(options,expected_sensor_ids);
    ZcmFunction(options,trajectory_id,(void*)this);
}

///确认配置文件引入了那些传感器进行组合定位及建图，似乎没有gps？
std::unordered_set<string> PerceptionNode::ComputeExpectedTopics(const TrajectoryOptions& options) {
    std::unordered_set<string> expected_topics;
    if (options.num_laser_scans > 0) {
        expected_topics.insert("scan");
    }

    if (options.num_multi_echo_laser_scans > 0) {
        expected_topics.insert("echoes");
    }

    if (options.num_point_clouds > 0) {
        expected_topics.insert("points2");
    }

    // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
    // required.
    if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
        (node_options_.map_builder_options.use_trajectory_builder_2d() &&
         options.trajectory_builder_options.trajectory_builder_2d_options()
                 .use_imu_data())) {
        expected_topics.insert("imu");
    }
    // Odometry is optional.
    if (options.use_odometry) {
        expected_topics.insert("odom");
    }
    ///created by cjf
    if (options.use_nav_sat) {
        expected_topics.insert("gps");
    }

    return expected_topics;
}


int PerceptionNode::AddTrajectory(const TrajectoryOptions& options, const std::unordered_set<string> expected_sensor_ids) {

    const int trajectory_id =
    map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
    AddExtrapolator(trajectory_id, options);
    is_active_trajectory_[trajectory_id] = true;

    return trajectory_id;
}


void * PerceptionNode::PublishNavInfo(void* __this){
    PerceptionNode* _this = (PerceptionNode*) __this;
    // cartographer输出第一个slam结果后才开始正常输出nav_info
        while(!_this->BoolInitialPose) {
        if (_this->BoolInitialPose) {
            break;
        }
        usleep(1000);
    }
        while(_this->BoolInitialPose){
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        GpsData gps_temp_nav;
        gps_mtx.lock();
        gps_temp_nav = gps;
        gps_mtx.unlock();

        nav_mtx.lock();
        msg_nav_temp.timestamp = (GpsAndSystemDiffSeconds + gps_temp_nav.week * SecondsPerWeek + gps_temp_nav.time - 18) * pow(10,6);
        msg_nav_temp.latitude = gps_temp_nav.latitude;
        msg_nav_temp.longitude = gps_temp_nav.longitude;
        msg_nav_temp.altitude = gps_temp_nav.altitude;
        if(gps_temp_nav.heading > 270 && gps_temp_nav.heading < 360){
            msg_nav_temp.angle_head = (450 - gps_temp_nav.heading) / 180 * M_PI;
        }
        else {
            msg_nav_temp.angle_head = (90 - gps_temp_nav.heading) / 180 * M_PI; // north to east as zero, counterclockwise, -pi~pi
        }
        TiEV::Angle angle_lon, angle_lat,heading;
        angle_lon.setByDegree(gps_temp_nav.longitude);
        angle_lat.setByDegree(gps_temp_nav.latitude);
        TiEV::WGS84Coor latlon_point((TiEV::LAT) angle_lat, (TiEV::LON) angle_lon);
        TiEV::UTMCoor utm_point = TiEV::latLonToUTMXY(latlon_point);
        msg_nav_temp.utm_x = utm_point.x;
        msg_nav_temp.utm_y = utm_point.y;

//        LOG(INFO) << msg_nav_temp.utm_x << ", " << msg_nav_temp.utm_y <<std::endl;

        msg_nav_temp.speed = gps_temp_nav.speed;
        msg_nav_temp.velocity_east = gps_temp_nav.speed_east;
        msg_nav_temp.velocity_north = gps_temp_nav.speed_north;
        msg_nav_temp.acceleration_x = gps_temp_nav.acceleration_y * 9.7964; /// by cjf 2020-11-09 单位g,右前上坐标系转成前左上车辆坐标系
        msg_nav_temp.acceleration_y = - gps_temp_nav.acceleration_x * 9.7964;
        msg_nav_temp.angle_pitch = - gps_temp_nav.pitch / 180 * M_PI; // degree to rad,downstairs as positive
        msg_nav_temp.angle_roll = gps_temp_nav.roll / 180 * M_PI; // degree to rad,right as positive
        msg_nav_temp.angular_vel_z = gps_temp_nav.angular_z / 180 * M_PI; // degree to rad
        msg_nav_temp.curvature = 0;
        msg_nav_temp.gps_num_satellites = gps_temp_nav.num_satellites;
        msg_nav_temp.HPOS_accuracy = 0;
        if (gps_temp_nav.status == 42 || gps_temp_nav.status == 82) {
            msg_nav_temp.RTK_status = 1;
            msg_nav_temp.is_reckoning_vaild = 1;
        } else {
            msg_nav_temp.RTK_status = 0;
            msg_nav_temp.is_reckoning_vaild = 0;
        }
        nav_mtx.unlock();
        pubzcm.publish("MsgNavInfoSignal", &msg_nav_temp);
            std::chrono::duration<double, ratio<1, 1000000>> duration = std::chrono::steady_clock::now() - t1;
            if(duration.count() < 10000)
                usleep(10000 - duration.count());
    }
}

/// 融合地图发布
void * PerceptionNode::GenerateFusionMap(void* __this)
{
    PerceptionNode* _this = (PerceptionNode*) __this;
        while(!_this->BoolInitialPose) {
        if (_this->BoolInitialPose) {
            break;
        }
        usleep(1000);
    }
    while (_this->BoolInitialPose) {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        MsgNavInfoSignal nav_info_for_map;
        nav_mtx.lock();
        nav_info_for_map = msg_nav_temp;
        nav_mtx.unlock();

        MsgHistoricalMap historical_map;
        historical_map.utm_x = nav_info_for_map.utm_x ;
        historical_map.utm_y = nav_info_for_map.utm_y;
        historical_map.angle_head = nav_info_for_map.angle_head;
        historical_map.rows = GRID_ROW;
        historical_map.cols = GRID_COL;
        historical_map.center_row = CAR_CEN_ROW;
        historical_map.center_col = CAR_CEN_COL;
        historical_map.resolution = GRID_RESOLUTION;
        historical_map.timestamp = nav_info_for_map.timestamp;




        //nav_x, nav_y, nav_z, yaw, rows, cols, cen_row, cen_col, reso ,
        //search_num , search_top, search_bottom, threshold
        //TODO utm转local坐标系


        Eigen::Quaterniond quaterniond{_this->initial_rotation[3],_this->initial_rotation[0],_this->initial_rotation[1],_this->initial_rotation[2]};
        double yaw = GetYaw(quaterniond);

        LOG(INFO) << yaw << std::endl;
        cv::Mat current_map = _this->GetImageByPose(historical_map.utm_x - _this->initial_translation[0],
                                                    historical_map.utm_y  - _this->initial_translation[1],
                                                    historical_map.angle_head,
                                                      501, 251, 351,
                                                      126, 0.2,
                                                      15,THE_SEARCH_THS);


        historical_map.cells.resize(GRID_ROW);
        for(int u=0;u<GRID_ROW;u++){
            historical_map.cells[u].resize(GRID_COL,0);
        }
        for(int i = 0;i<GRID_ROW;i++){
            for(int j = 0;j<GRID_COL;j++){
                historical_map.cells[i][j] = current_map.at<uint8_t>(i,j);
            }
        }

        pubzcm.publish("MsgHistoricalMap", &historical_map);

        std::chrono::duration<double, ratio<1, 1000000>> duration = std::chrono::steady_clock::now() - t1;
        if(duration.count() < 40000)
            usleep(40000 - duration.count());
    }
}
/// SLAM结果发布, nav_info里的x,y,heading角需要从PublishTrajectoryStates()的结果中获取
void * PerceptionNode::GenerateSLAMLocation(void* __this){



}
///冻结所有轨迹
void PerceptionNode::FinishAllTrajectories() {
    cartographer::common::MutexLocker lock(&mutex_);
    for (auto& entry : is_active_trajectory_) {
        const int trajectory_id = entry.first;
        if (entry.second) {
            map_builder_bridge_.FinishTrajectory(trajectory_id);
            entry.second = false;
        }
    }
}
///冻结一条特定id的轨迹
void PerceptionNode::FinishTrajectory(const int trajectory_id) {
    cartographer::common::MutexLocker lock(&mutex_);
    CHECK(is_active_trajectory_.at(trajectory_id));
    map_builder_bridge_.FinishTrajectory(trajectory_id);
    is_active_trajectory_[trajectory_id] = false;
}

void PerceptionNode::RunFinalOptimization(){
    {
        cartographer::common::MutexLocker lock(&mutex_);
        for (const auto& entry : is_active_trajectory_) {
            CHECK(!entry.second);
        }
    }
    // Assuming we are not adding new data anymore, the final optimization
    // can be performed without holding the mutex.
    map_builder_bridge_.RunFinalOptimization();
}

void PerceptionNode::SerializeState(const string& filename)
{
    cartographer::common::MutexLocker lock(&mutex_);
    map_builder_bridge_.SerializeState(filename);
}

void PerceptionNode::LoadMap(const std::string& map_filename) {
    cartographer::common::MutexLocker lock(&mutex_);
    map_builder_bridge_.LoadMap(map_filename);
}

bool PerceptionNode::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
    if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
        return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
    }
    if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
        return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
    }
    return false;
}

/// 由GenerateSLAMLocation调用
Rigid3d PerceptionNode::PublishTrajectoryStates() {
    ::cartographer::common::MutexLocker lock(&mutex_);
    for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
        const auto& trajectory_state = entry.second;

        auto& extrapolator = extrapolators_.at(entry.first);
        // We only publish a point cloud if it has changed. It is not needed at high
        // frequency, and republishing it would be computationally wasteful.
        if (trajectory_state.pose_estimate.time != extrapolator.GetLastPoseTime()) {
            //不发送配准点云
//            scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
//                    carto::common::ToUniversal(trajectory_state.pose_estimate.time),
//                    node_options_.map_frame,
//                    carto::sensor::TransformPointCloud(
//                            trajectory_state.pose_estimate.point_cloud,
//                            trajectory_state.local_to_map.cast<float>())));
            extrapolator.AddPose(trajectory_state.pose_estimate.time,
                                 trajectory_state.pose_estimate.pose);
        }
        // If we do not publish a new point cloud, we still allow time of the
        // published poses to advance. If we already know a newer pose, we use its
        // time instead. Since tf knows how to interpolate, providing newer
        // information is better.

        //定位成功后 trajectory_state.local_to_map 有值    则tracking_to_map为全局坐标
        const ::cartographer::common::Time now = extrapolator.GetLastPoseTime();
        const Rigid3d tracking_to_local = extrapolator.ExtrapolatePose(now);
        const Rigid3d tracking_to_map =
                trajectory_state.local_to_map * tracking_to_local;

//        LOG(INFO) << "tracking_to_local:"<< tracking_to_local.DebugString() << std::endl;
//        LOG(INFO) << "tracking_to_map:"<< tracking_to_map.DebugString() << std::endl;
//        LOG(INFO) << "trajectory_state.local_to_map:"<< trajectory_state.local_to_map.DebugString() << std::endl;

        Rigid3d global_pose;
        if (trajectory_state.published_to_tracking != nullptr) {
            if (trajectory_state.trajectory_options.provide_odom_frame) {
                global_pose = tracking_to_local * (*trajectory_state.published_to_tracking);
            } else {
                global_pose = tracking_to_map * (*trajectory_state.published_to_tracking);
            }
        }

        if ( BoolGlobalLocation == false && abs(trajectory_state.local_to_map.translation().x())
             + abs(trajectory_state.local_to_map.translation().y())
             + abs(trajectory_state.local_to_map.translation().z()) >  0.5 )
        {
            BoolGlobalLocation = true;
        }

        return tracking_to_map;
//        cout << global_pose.translation().x()
//                <<"," << global_pose.translation().y()
//                <<"," << global_pose.translation().z()
//                <<"," << global_pose.rotation().x()
//                <<"," << global_pose.rotation().y()
//                <<"," << global_pose.rotation().z()
//                <<"," << global_pose.rotation().w()
//                <<std::endl;
    }
}

///输入zcm发布的odom消息，转成cartographer格式，并添加到pose_extrapolator进行位姿融合
///需添加gps消息，imu消息
/*void PerceptionNode::AddOdomData(const string& sensor_id , const Odometry& odom , int trajectory_id)
{
    cartographer::common::MutexLocker lock(&mutex_);
    auto msg_bridge_ptr = map_builder_bridge_.msg_bridge(trajectory_id);
    auto odometry_data_ptr = msg_bridge_ptr->ToOdometryData(odom);///转cartographer格式
    if (odometry_data_ptr != nullptr) {
        extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
    }
    map_builder_bridge_.msg_bridge(trajectory_id)
    ->HandleOdometryMessage("odom", odom);
}*/
/////scan和pointcloud都添加给map_builder_bridge处理
//void PerceptionNode::AddScanData( const string& sensor_id , const vector<Eigen::Vector3f>& msg ,const int64_t& sec,const int64_t& nsec , int trajectory_id )
//{
//    cartographer::common::MutexLocker lock(&mutex_);
//
//
//    Time time_temp;
//    time_temp.sec = sec;
//    time_temp.nsec = nsec;
//
//    map_builder_bridge_.msg_bridge(trajectory_id)
//            ->HandleLaserScanMessage("scan", msg ,time_temp);
//}

void PerceptionNode::AddPointCloudData(const string& sensor_id , const vector<Eigen::Vector3f>& msg ,const int64_t& sec,const int64_t& nsec , int trajectory_id )
{
    cartographer::common::MutexLocker lock(&mutex_);

    Time time_temp;
    time_temp.sec = sec;
    time_temp.nsec = nsec;

    map_builder_bridge_.msg_bridge(trajectory_id)
    ->HandlePointCloudMessage("points2", msg ,time_temp);

}

/// 由GenerateFusionMap调用
cv::Mat PerceptionNode::GetImageByPose(double reckon_x , double reckon_y ,double yaw, int rows, int cols, int cen_row, int cen_col, float reso ,int search_num ,float threshold)
{
    cartographer::common::MutexLocker lock(&mutex_);
    return map_builder_bridge_.GetImageByPose(reckon_x , reckon_y , yaw,  rows,  cols,  cen_row,  cen_col, reso , search_num , threshold );
}
}
