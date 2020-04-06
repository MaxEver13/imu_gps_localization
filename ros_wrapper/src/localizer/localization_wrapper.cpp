/*
 * @Description: imu,gnss融合定位
 * @Author: Ji Jiawen
 * @Date: 2020-04-06
 */

#include "localizer/localization_wrapper.h"

#include "state.h"

namespace ImuGpsLocalization {


LocalizationWrapper::LocalizationWrapper(ros::NodeHandle nh) {
  // imu 噪声参数
  double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
  nh.param("acc_noise",       acc_noise, 1e-2);
  nh.param("gyro_noise",      gyro_noise, 1e-4);
  nh.param("acc_bias_noise",  acc_bias_noise, 1e-6);
  nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);

  // imu,gnss外参平移部分
  double x, y, z;
  nh.param("I_p_Gps_x", x, 0.);
  nh.param("I_p_Gps_y", y, 0.);
  nh.param("I_p_Gps_z", z, 0.);
  const Eigen::Vector3d I_p_Gps(x, y, z);

  // 初始化融合定位
  imu_gps_localizer_ptr_ = std::make_unique<ImuGpsLocalizer>(acc_noise, 
                              gyro_noise, acc_bias_noise, gyro_bias_noise, I_p_Gps);

  // 订阅同步后的消息  
  imu_sub_ = nh.subscribe("/synced_imu", 10,  &LocalizationWrapper::ImuCallback, this);
  vel_sub_ = nh.subscribe("/synced_vel", 10,  &LocalizationWrapper::GpsVelocityCallback, this);
  gnss_sub_ = nh.subscribe("/synced_gnss", 10,  &LocalizationWrapper::GpsPositionCallback, this);

  // 发布融合定位路径
  path_pub_ = nh.advertise<nav_msgs::Path>("fused_path", 10);
}

// 订阅imu消息并处理
void LocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
  ImuDataPtr imu_data_ptr = std::make_shared<ImuData>();
  imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
  imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x, 
                        imu_msg_ptr->linear_acceleration.y,
                        imu_msg_ptr->linear_acceleration.z;
  imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
                        imu_msg_ptr->angular_velocity.y,
                        imu_msg_ptr->angular_velocity.z;
  
  State fused_state;
  const bool ok = imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state);
  if (!ok) {
      return;
  }

  // Publish fused path.
  ConvertStateToRosTopic(fused_state);
  path_pub_.publish(path_);

}

// 订阅gps位置消息并处理
void LocalizationWrapper::GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr) {
  // Check the gps_status. 2 means that with ground-based augmentation.
  // if (gps_msg_ptr->status.status != 2) {
  //     LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
  //     return;
  // }
  
  GnssDataPtr gnss_data_ptr = std::make_shared<GnssData>();
  gnss_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
  gnss_data_ptr->lla << gps_msg_ptr->latitude,
                        gps_msg_ptr->longitude,
                        gps_msg_ptr->altitude;
  gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());

  imu_gps_localizer_ptr_->ProcessGnssData(gnss_data_ptr);
}

// 订阅gps速度消息并处理
void LocalizationWrapper::GpsVelocityCallback(const geometry_msgs::TwistStampedConstPtr& gps_vel_msg_ptr) {    
  VelocityDataPtr vel_data_ptr = std::make_shared<VelocityData>();
  vel_data_ptr->timestamp = gps_vel_msg_ptr->header.stamp.toSec();
  vel_data_ptr->vel(0) = gps_vel_msg_ptr->twist.linear.x;
  vel_data_ptr->vel(1) = gps_vel_msg_ptr->twist.linear.y;
  vel_data_ptr->vel(2) = gps_vel_msg_ptr->twist.linear.z;
  vel_data_ptr->cov = 2. * 2. * Eigen::Matrix3d::Identity();
  vel_data_ptr->cov(2, 2) = 1e10;

  imu_gps_localizer_ptr_->ProcessVelData(vel_data_ptr);
}

// 转换成path用于ros发布
void LocalizationWrapper::ConvertStateToRosTopic(const State& state) {
  path_.header.frame_id = "world";
  path_.header.stamp = ros::Time::now();  

  geometry_msgs::PoseStamped pose;
  pose.header = path_.header;

  pose.pose.position.x = state.G_p_I[0];
  pose.pose.position.y = state.G_p_I[1];
  pose.pose.position.z = state.G_p_I[2];

  const Eigen::Quaterniond G_q_I(state.G_R_I);
  pose.pose.orientation.x = G_q_I.x();
  pose.pose.orientation.y = G_q_I.y();
  pose.pose.orientation.z = G_q_I.z();
  pose.pose.orientation.w = G_q_I.w();

  path_.poses.push_back(pose);
}



}