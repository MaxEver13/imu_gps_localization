#include "initializer.h"

#include <Eigen/Dense>
#include <glog/logging.h>

#include "utils.h"

namespace ImuGpsLocalization {

Initializer::Initializer(const Eigen::Vector3d& init_I_p_Gps) 
    : init_I_p_Gps_(init_I_p_Gps) { }

void Initializer::AddImuData(const ImuDataPtr imu_data_ptr) {
  imu_buffer_.push_back(imu_data_ptr);
}

void Initializer::AddVelData(const VelocityDataPtr vel_data_ptr) {
  vel_buffer_.push_back(vel_data_ptr);
}

bool Initializer::AddGnssData(const GnssDataPtr gnss_data_ptr, State* state) {
  // 如果imu数据太少，下面初始化的时候不能很好地设置pitch和roll
  if (imu_buffer_.size() < kImuDataBufferLength) {
    LOG(WARNING) << "[AddGnssData]: No enough imu data!";
    return false;
  }

  // 当imu数据足够且vel数据都有时，才初始化系统
  if (vel_buffer_.size() == 0)
    return false;

  // 最新的imu和vel数据
  ImuDataPtr latest_imu_data = imu_buffer_.back();
  VelocityDataPtr latest_vel_data = vel_buffer_.back();

  // 设置初始状态时间戳和imu数据
  state->timestamp = gnss_data_ptr->timestamp;
  state->imu_data_ptr = latest_imu_data;

  // Set initial mean.
  state->G_p_I.setZero();
  // We have no information to set initial velocity. 
  // So, just set it to zero and given big covariance.
  state->G_v_I.setZero();
  // We can use the direction of gravity to set roll and pitch. 
  // But, we cannot set the yaw. 
  // So, we set yaw to zero and give it a big covariance.
  if (!ComputeG_R_IFromImuData(&state->G_R_I)) {
    LOG(WARNING) << "[AddGpsPositionData]: Failed to compute G_R_I!";
    return false;
  }

  const double yaw = std::atan2(latest_vel_data->vel(1), latest_vel_data->vel(0));
  state->G_R_I = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * state->G_R_I.eval();

  // Set bias to zero.
  state->acc_bias.setZero();
  state->gyro_bias.setZero();

  // Set covariance.
  state->cov.setZero();
  state->cov.block<3, 3>(0, 0) = 100. * Eigen::Matrix3d::Identity(); // position std: 10 m
  state->cov.block<3, 3>(3, 3) = 100. * Eigen::Matrix3d::Identity(); // velocity std: 10 m/s
  // roll pitch std 10 degree.
  state->cov.block<2, 2>(6, 6) = 10. * kDegreeToRadian * 10. * kDegreeToRadian * Eigen::Matrix2d::Identity();
  state->cov(8, 8)             = 100. * kDegreeToRadian * 100. * kDegreeToRadian; // yaw std: 100 degree.
  // Acc bias.
  state->cov.block<3, 3>(9, 9) = 0.0004 * Eigen::Matrix3d::Identity();
  // Gyro bias.
  state->cov.block<3, 3>(12, 12) = 0.0004 * Eigen::Matrix3d::Identity();

  // clear buffer
  imu_buffer_.clear();
  vel_buffer_.clear();

  return true;
}



bool Initializer::ComputeG_R_IFromImuData(Eigen::Matrix3d* G_R_I) {
  // Compute mean and std of the imu buffer.
  Eigen::Vector3d sum_acc(0., 0., 0.);
  for (const auto imu_data : imu_buffer_) {
      sum_acc += imu_data->acc;
  }
  const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();

  Eigen::Vector3d sum_err2(0., 0., 0.);
  for (const auto imu_data : imu_buffer_) {
      sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
  }
  const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();

  if (std_acc.maxCoeff() > kAccStdLimit) {
      LOG(WARNING) << "[ComputeG_R_IFromImuData]: Too big acc std: " << std_acc.transpose();
      return false;
  }

  // Compute rotation.
  const Eigen::Vector3d acc_norm = mean_acc.normalized();  
  const double rot_y = std::atan2(acc_norm(0), acc_norm(2));
  double rot_x = std::atan2(std::abs(acc_norm(1)), std::sqrt(acc_norm(0) * acc_norm(0) + acc_norm(2) * acc_norm(2)));
  if (-std::sin(rot_x) * acc_norm(1) < 0.) {
      rot_x = -rot_x;
  }

  Eigen::AngleAxisd RotX(rot_x, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd RotY(rot_y, Eigen::Vector3d::UnitY());

  *G_R_I = (RotY * RotX).toRotationMatrix().transpose();

  return true;
}

}  // namespace ImuGpsLocalization