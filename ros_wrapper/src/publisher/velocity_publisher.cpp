/*
 * @Description: 发布速度消息
 * @Author: Ji Jiawen
 * @Date: 2020-04-03
 */

#include "publisher/velocity_publisher.h"


namespace ImuGpsLocalization {

VelocityPublisher::VelocityPublisher(ros::NodeHandle nh, std::string topic_name, std::string frame_id, size_t buff_size) :
  nh_(nh), frame_id_(frame_id) {
  publisher_ = nh_.advertise<geometry_msgs::TwistStamped>(topic_name, buff_size);
}


void VelocityPublisher::Publish(VelocityDataConstPtr velo_data_ptr, double time) {
  ros::Time ros_time(time);
  PublishData(velo_data_ptr, ros_time);
}

bool VelocityPublisher::HasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}

void VelocityPublisher::PublishData(VelocityDataConstPtr velo_data_ptr, ros::Time time) {
  // std::cout << "PublishData vel timestamp: " << std::fixed << time.toSec() << std::endl;
  velo_msg_.header.stamp = time;
  velo_msg_.header.frame_id = frame_id_;

  velo_msg_.twist.linear.x = velo_data_ptr->vel(0);
  velo_msg_.twist.linear.y = velo_data_ptr->vel(1);
  velo_msg_.twist.linear.z = velo_data_ptr->vel(2);

  // publish
  publisher_.publish(velo_msg_);
}


}