/*
 * @Description: 定位的node文件
 * @Author: Ji Jiawen
 * @Date: 2020-04-06
 */

#include <memory>
#include <ros/ros.h>

#include "localizer/localization_wrapper.h"

using namespace ImuGpsLocalization;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "imu_gps_localization_node");

  ros::NodeHandle nh;

  std::shared_ptr<LocalizationWrapper> localization_ptr = std::make_shared<LocalizationWrapper>(nh);

  ros::spin();

  return 0;
}

