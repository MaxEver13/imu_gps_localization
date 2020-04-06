#pragma once 

#include <Eigen/Dense>

#include "state.h"
#include "sensor_data/gnss_data.h"
#include "sensor_data/velocity_data.h"

namespace ImuGpsLocalization {

class GpsProcessor {
public:
  GpsProcessor(const Eigen::Vector3d& I_p_Gps);

  bool UpdateStateByGpsPosition(const Eigen::Vector3d& init_lla, const GnssDataPtr gnss_data_ptr, State* state);

  bool UpdateStateByGpsVelocity(const Eigen::Vector3d& init_lla, const VelocityDataPtr vel_data_ptr, State* state);
    
private:    
  void ComputeJacobianAndResidual(const Eigen::Vector3d& init_lla,  
                                  const GnssDataPtr gnss_data, 
                                  const State& state,
                                  Eigen::Matrix<double, 3, 15>* jacobian,
                                  Eigen::Vector3d* residual);
  
  void ComputeJacobianAndResidual(const Eigen::Vector3d& init_lla,  
                                  const VelocityDataPtr vel_data, 
                                  const State& state,
                                  Eigen::Matrix<double, 3, 15>* jacobian,
                                  Eigen::Vector3d* residual);

  void AddDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state);
                                      
  const Eigen::Vector3d I_p_Gps_;  
};



}  // namespace ImuGpsLocalization