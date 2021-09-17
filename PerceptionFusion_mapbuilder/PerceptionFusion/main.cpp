//
//  PerceptionFusion
//
//  Created by Xinglian Zhang on 2017/10. modified by 赵君峤 on 2017/10/20.
//  Copyright © 2017年. All rights reserved.
//

#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <vector>

#include "cartographer/mapping/map_builder.h"
#include "trajectory_option.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "node_option.h"
#include "node.h"
#include "ParameterReader.h"
//#include "common/nature.h"

using namespace TiEV;

int main() {
    ParameterReader reader(std::string("../parameters.txt"));
    int mode = atoi(reader.getData("mode").c_str());
    string configuration_directory =reader.getData("configuration_directory");
    string configuration_basename = reader.getData("configuration_basename");

    if( mode == 1 )
    {
        configuration_directory += "/above_ground_localization";
    }


    NodeOptions node_options;
    TrajectoryOptions trajectory_options;
    /// LoadOptions在node_Options.cc中实现，
    /// 实际分别调用了node_options和 trajectory_options的create函数，
    ///返回一个options
    std::tie(node_options, trajectory_options) =
            LoadOptions(configuration_directory, configuration_basename);

    PerceptionNode node(node_options,mode);// 采用mode方式传入设置,所以不需要源版中的map_builder

    ///load historical map or not
    bool LoadMapFlag = atoi(reader.getData("LoadMapFlag").c_str());
    if (LoadMapFlag)
    {
        string map_filename = reader.getData("map_filename");
        node.LoadMap(map_filename + ".pbstream");
        std::cout<<"Load map:"<<map_filename<<endl;
        ///initial pose
        string initial_pose_file = map_filename + ".pose";
        cout<<"pose_file :"<<initial_pose_file<<endl;
        std::ifstream fin_pose(initial_pose_file);
        if (fin_pose.is_open())
        {
            cout<<"loading<<<<<<<<<<"<<endl;
            ///格式顺序：x,y,z,q(x,y,z,w)
            double pose_x ,pose_y,pose_z, orientation_x,orientation_y,orientation_z,orientation_w;
            fin_pose >> pose_x >>pose_y >> pose_z >>
                   orientation_x >>orientation_y >>orientation_z >>orientation_w;

            node.initial_translation[0] = pose_x;
            node.initial_translation[1] = pose_y;
            node.initial_translation[2] = pose_z;
//            node.initial_rotation[0] = orientation_x;
//            node.initial_rotation[1] = orientation_y;
//            node.initial_rotation[2] = orientation_z;
//            node.initial_rotation[3] = orientation_w;
            fin_pose.close();
            //cout<<pose_x<<node.initial_tran[1]<<endl;
            ///设置两个bool值，确保重定位模式下初始化位姿和加载地图成功才进行后续操作
//            node.BoolInitialPose = true;
//            node.BoolLoadMap = true;
        }
        else
        {
            std::cout<<"Fail to load initial pose file from: "<< initial_pose_file <<std::endl;
            return 0;
        }
    }



    ///建立map_builder 并向trajectory_builder添加数据
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
    node.FinishAllTrajectories();
//    node.RunFinalOptimization();
    ///save map or not
    bool SaveMapFlag = atoi(reader.getData("SaveMapFlag").c_str());
//    if (SaveMapFlag || mode == 2 || mode == 4)
//    {
//        ///save as pbstream
//        string save_path = reader.getData("save_path");
//        std::tm *now =  TiEV::getTime(TiEV::getTimeStamp());
//        string save_map_file = save_path + std::to_string(now->tm_year+1900) + "_" + std::to_string(now->tm_mon+1) + "_" +
//        std::to_string(now->tm_mday) + "_" + std::to_string(now->tm_hour) + "_" + std::to_string(now->tm_min) +
//        ".pbstream";
//
//        LOG(INFO) << "Writing state to '" << save_map_file << "...";
//        node.SerializeState(save_map_file);
//        ///save initial pose
//        string save_pose_file = save_path + std::to_string(now->tm_year+1900) + "_" + std::to_string(now->tm_mon+1) + "_" +
//        std::to_string(now->tm_mday) + "_" + std::to_string(now->tm_hour) + "_" + std::to_string(now->tm_min) +
//        ".pose";
//        LOG(INFO) << "Writing pose to '" << save_pose_file << "'...";
//        std::ofstream fout_pose(save_pose_file);
//        if (fout_pose.is_open())
//        {
//            fout_pose << setprecision(16) << node.initial_translation[0] <<" "<<node.initial_translation[1] << " "<<node.initial_translation[2] <<
//            " "<<node.initial_rotation[0] <<" "<<node.initial_rotation[1] <<" "<<node.initial_rotation[2] <<" "<<
//            node.initial_rotation[3]<<std::endl;
//            fout_pose.close();
//            std::cout<<"Finish writing pose file"<<std::endl;
//        }
//        else
//        {
//            std::cout<<"Fail to open pose file for writing"<<std::endl;
//            std::cout<<"Current initial pose (x, y) is :"<<node.initial_translation[0] << ", " <<node.initial_translation[1]<< std::endl;
//        }
//    }
    cout << "Congratulations! finish here!" << endl;
    return 0;
}
