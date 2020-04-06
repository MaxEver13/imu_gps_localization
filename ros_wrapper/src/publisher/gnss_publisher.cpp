/*
 * @Description: 发布gnss消息
 * @Author: Ji Jiawen
 * @Date: 2020-04-05
 */

#include "publisher/gnss_publisher.h"


namespace ImuGpsLocalization {

GnssPublisher::GnssPublisher(ros::NodeHandle nh, std::string topic_name, std::string frame_id, size_t buff_size) :
  nh_(nh), frame_id_(frame_id) {
  publisher_ = nh_.advertise<sensor_msgs::NavSatFix>(topic_name, buff_size);
}

void GnssPublisher::Publish(GnssDataConstPtr gnss_data_ptr, double time) {
  ros::Time ros_time(time);
  PublishData(gnss_data_ptr, ros_time);
}

bool GnssPublisher::HasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}

void GnssPublisher::PublishData(GnssDataConstPtr gnss_data_ptr, ros::Time time) {
  // std::cout << "PublishData gnss timestamp: " << std::fixed << time.toSec() << std::endl;
  gnss_msg_.header.stamp = time;
  gnss_msg_.header.frame_id = frame_id_;

  gnss_msg_.latitude = gnss_data_ptr->lla(0);
  gnss_msg_.longitude = gnss_data_ptr->lla(1);
  gnss_msg_.altitude = gnss_data_ptr->lla(2);

  gnss_msg_.position_covariance = { gnss_data_ptr->cov(0, 0), 
                                   gnss_data_ptr->cov(0, 1),
                                   gnss_data_ptr->cov(0, 2),
                                   gnss_data_ptr->cov(1, 0),
                                   gnss_data_ptr->cov(1, 1),
                                   gnss_data_ptr->cov(1, 2),
                                   gnss_data_ptr->cov(2, 0),
                                   gnss_data_ptr->cov(2, 1),
                                   gnss_data_ptr->cov(2, 2) };

  // publish
  publisher_.publish(gnss_msg_);
}


}