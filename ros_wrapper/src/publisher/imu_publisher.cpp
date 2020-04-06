/*
 * @Description: 发布imu数据
 * @Author: Ji Jiawen
 * @Date: 2020-04-03
 */

#include "publisher/imu_publisher.h"


namespace ImuGpsLocalization {

ImuPublisher::ImuPublisher(ros::NodeHandle nh, std::string topic_name, std::string frame_id, size_t buff_size) :
  nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::Imu>(topic_name, buff_size);
}

void ImuPublisher::Publish(ImuDataConstPtr imu_data_ptr, double time) {
  ros::Time ros_time(time);
  PublishData(imu_data_ptr, ros_time);
}

bool ImuPublisher::HasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}

void ImuPublisher::PublishData(ImuDataConstPtr imu_data_ptr, ros::Time time) {
  // std::cout << "PublishData imu timestamp: " << std::fixed << time.toSec() << std::endl;
  imu_msg_.header.stamp = time;
  imu_msg_.header.frame_id = frame_id_;

  imu_msg_.linear_acceleration.x = imu_data_ptr->acc(0);
  imu_msg_.linear_acceleration.y = imu_data_ptr->acc(1);
  imu_msg_.linear_acceleration.z = imu_data_ptr->acc(2);
  
  imu_msg_.angular_velocity.x = imu_data_ptr->gyro(0);
  imu_msg_.angular_velocity.y = imu_data_ptr->gyro(1);
  imu_msg_.angular_velocity.z = imu_data_ptr->gyro(2);

  // publish
  publisher_.publish(imu_msg_);
}


}