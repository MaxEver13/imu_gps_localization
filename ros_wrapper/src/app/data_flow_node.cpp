/*
 * @Description: 数据预处理的node文件
 * @Author: Ji Jiawen
 * @Date: 2020-04-02
 */

#include <ros/ros.h>
#include <glog/logging.h>

#include "data_flow/data_flow.h"

using namespace ImuGpsLocalization;


int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  // Set glog.
  FLAGS_colorlogtostderr = true;
  // 创建节点
  ros::init(argc, argv, "data_flow_node");
  ros::NodeHandle nh;

  std::shared_ptr<DataFlow> data_flow_ptr = std::make_shared<DataFlow>(nh);

  ros::Rate rate(100);

  while (ros::ok()) {
    ros::spinOnce();

    data_flow_ptr->Run();

    rate.sleep();
  }
  
  return 0;
}