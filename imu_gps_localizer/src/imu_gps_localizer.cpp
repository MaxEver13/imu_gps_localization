#include "imu_gps_localizer.h"
#include "utils.h"

#include <iostream>
#include <glog/logging.h>


namespace ImuGpsLocalization {

ImuGpsLocalizer::ImuGpsLocalizer(const double acc_noise, const double gyro_noise,
                                 const double acc_bias_noise, const double gyro_bias_noise,
                                 const Eigen::Vector3d& I_p_Gps) :
  initialized_(false) {
  initializer_ = std::make_unique<Initializer>(I_p_Gps);
  imu_processor_ = std::make_unique<ImuProcessor>(acc_noise, gyro_noise, 
                                                  acc_bias_noise, gyro_bias_noise,
                                                  Eigen::Vector3d(0., 0., -9.81007));
  gps_processor_ = std::make_unique<GpsProcessor>(I_p_Gps);
}

bool ImuGpsLocalizer::ProcessImuData(const ImuDataPtr imu_data_ptr, State* fused_state) {
  std::cout << "[ProcessImuData] timestamp: " << std::fixed << imu_data_ptr->timestamp << std::endl;
  if (!initialized_) {
    initializer_->AddImuData(imu_data_ptr);
    return false;
  }
  
  // Predict.
  imu_processor_->Predict(state_.imu_data_ptr, imu_data_ptr, &state_);

  // Convert ENU state to lla.
  ConvertENUToLLA(init_lla_, state_.G_p_I, &(state_.lla));
  *fused_state = state_;
  return true;
}

bool ImuGpsLocalizer::ProcessGnssData(const GnssDataPtr gnss_data_ptr) {
  std::cout << "[ProcessGnssData] timestamp: " << std::fixed << gnss_data_ptr->timestamp << std::endl;
  if (!initialized_) {
    // IMU data and velocity data are ready. 
    // Initialized system succeed(initial system state and cov).
    if (!initializer_->AddGnssData(gnss_data_ptr, &state_)) {
      return false;
    }

    // Initialize the initial gps point used to convert lla to ENU.
    init_lla_ = gnss_data_ptr->lla;
    
    initialized_ = true;

    LOG(INFO) << "[ProcessGnssData]: System initialized!";
    return true;
  }

  // Update.
  gps_processor_->UpdateStateByGpsPosition(init_lla_, gnss_data_ptr, &state_);

  return true;
}

bool ImuGpsLocalizer::ProcessVelData(const VelocityDataPtr vel_data_ptr) {
  std::cout << "[ProcessVelData] timestamp: " << std::fixed << vel_data_ptr->timestamp << std::endl;

  // if (vel_data_ptr->vel.norm() < kGpsVelLimit) {
  //   return false;
  // }
  
  if (!initialized_) {
    initializer_->AddVelData(vel_data_ptr);
    return false;
  }

  gps_processor_->UpdateStateByGpsVelocity(init_lla_, vel_data_ptr, &state_);

  return true;
}

}  // namespace ImuGpsLocalization