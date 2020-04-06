/*
 * @Description: 订阅GNSS数据
 * @Author: Ji Jiawen
 * @Date: 2020-04-03
 */

#include "subscriber/gnss_subscriber.h"
#include <glog/logging.h>
#include <iostream>

namespace ImuGpsLocalization {

GnssSubscriber::GnssSubscriber(ros::NodeHandle nh, std::string topic_name, size_t buff_size) : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &GnssSubscriber::msg_callback, this);
}

void GnssSubscriber::ParseData(std::deque<GnssDataPtr>& deque_gnss_data) {
  buffer_mutex_.lock();
  if (gnss_data_buffer_.size() > 0) {
    deque_gnss_data.insert(deque_gnss_data.end(), gnss_data_buffer_.begin(), gnss_data_buffer_.end());
    gnss_data_buffer_.clear();
  }
  buffer_mutex_.unlock();
}

void GnssSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr& gnss_msg_ptr) {
  buffer_mutex_.lock();

  // Check the gps_status. 2 means that with ground-based augmentation.
  if (gnss_msg_ptr->status.status != 2) {
      LOG(WARNING) << "[GnssMsgCallBack]: Bad gnss message!";
      return;
  }
  GnssDataPtr gnss_data_ptr = std::make_shared<GnssData>();

  gnss_data_ptr->timestamp = gnss_msg_ptr->header.stamp.toSec();
  gnss_data_ptr->lla << gnss_msg_ptr->latitude,
                        gnss_msg_ptr->longitude,
                        gnss_msg_ptr->altitude;
  gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gnss_msg_ptr->position_covariance.data());

  // std::cout << "[GnssSubscriber::msg_callback]: " << std::fixed << "timestamp: " << gnss_data_ptr->timestamp << std::endl;  
  // std::cout << "lla: " << gnss_data_ptr->lla << std::endl;  
  // std::cout << "cov: " << gnss_data_ptr->cov << std::endl;    

  gnss_data_buffer_.push_back(gnss_data_ptr);
  buffer_mutex_.unlock();
}

}