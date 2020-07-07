#include "imu_processor.h"

#include <glog/logging.h>
#include <Eigen/Dense>

#include "utils.h"

using namespace Eigen;

namespace ImuGpsLocalization {

ImuProcessor::ImuProcessor(const double acc_noise, const double gyro_noise,
                           const double acc_bias_noise, const double gyro_bias_noise,
                           const Eigen::Vector3d& gravity) :
  acc_noise_(acc_noise), gyro_noise_(gyro_noise), 
  acc_bias_noise_(acc_bias_noise), gyro_bias_noise_(gyro_bias_noise),
  gravity_(gravity) { }


void ImuProcessor::PredictNewState(const double& dt,
    const Vector3d& gyro, const Vector3d& acc,
    State* state) {

  // TODO: Will performing the forward integration using
  //    the inverse of the quaternion give better accuracy?
  double gyro_norm = gyro.norm();
  Matrix4d Omega = Matrix4d::Zero();
  Omega.block<3, 3>(0, 0) = -skewSymmetric(gyro);
  Omega.block<3, 1>(0, 3) = gyro;
  Omega.block<1, 3>(3, 0) = -gyro;

  Vector3d p = state->G_p_I;
  Vector3d v = state->G_v_I;  
  Vector4d q = rotationToQuaternion(state->G_R_I.transpose()); // from wold frame to imu frame

  // Some pre-calculation
  Vector4d dq_dt, dq_dt2;
  if (gyro_norm > 1e-5) {
    dq_dt = (cos(gyro_norm*dt*0.5)*Matrix4d::Identity() +
      1/gyro_norm*sin(gyro_norm*dt*0.5)*Omega) * q;
    dq_dt2 = (cos(gyro_norm*dt*0.25)*Matrix4d::Identity() +
      1/gyro_norm*sin(gyro_norm*dt*0.25)*Omega) * q;
  }
  else {
    dq_dt = (Matrix4d::Identity()+0.5*dt*Omega) *
      cos(gyro_norm*dt*0.5) * q;
    dq_dt2 = (Matrix4d::Identity()+0.25*dt*Omega) *
      cos(gyro_norm*dt*0.25) * q;
  }
  Matrix3d dR_dt_transpose = quaternionToRotation(dq_dt).transpose();
  Matrix3d dR_dt2_transpose = quaternionToRotation(dq_dt2).transpose();

  // k1 = f(tn, yn)
  Vector3d k1_v_dot = quaternionToRotation(q).transpose()*acc +
    gravity_;
  Vector3d k1_p_dot = v;

  // k2 = f(tn+dt/2, yn+k1*dt/2)
  Vector3d k1_v = v + k1_v_dot*dt/2;
  Vector3d k2_v_dot = dR_dt2_transpose*acc +
    gravity_;
  Vector3d k2_p_dot = k1_v;

  // k3 = f(tn+dt/2, yn+k2*dt/2)
  Vector3d k2_v = v + k2_v_dot*dt/2;
  Vector3d k3_v_dot = dR_dt2_transpose*acc +
    gravity_;
  Vector3d k3_p_dot = k2_v;

  // k4 = f(tn+dt, yn+k3*dt)
  Vector3d k3_v = v + k3_v_dot*dt;
  Vector3d k4_v_dot = dR_dt_transpose*acc +
    gravity_;
  Vector3d k4_p_dot = k3_v;

  // yn+1 = yn + dt/6*(k1+2*k2+2*k3+k4)
  q = dq_dt;
  quaternionNormalize(q);
  v = v + dt/6*(k1_v_dot+2*k2_v_dot+2*k3_v_dot+k4_v_dot);
  p = p + dt/6*(k1_p_dot+2*k2_p_dot+2*k3_p_dot+k4_p_dot);

  // update state
  state->G_p_I = p;
  state->G_v_I = v;
  state->G_R_I = quaternionToRotation(q).transpose();
}    

void ImuProcessor::Predict(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State* state) {
  // Time.
  const double delta_t = cur_imu->timestamp - last_imu->timestamp;
  const double delta_t2 = delta_t * delta_t;

  // Set last state.
  State last_state = *state;

  // Acc and gyro.
  const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + cur_imu->acc) - last_state.acc_bias;
  const Eigen::Vector3d gyro_unbias = 0.5 * (last_imu->gyro + cur_imu->gyro) - last_state.gyro_bias;

  // Normal state. 
  // Using P58. of "Quaternion kinematics for the error-state Kalman Filter".
  // state->G_p_I = last_state.G_p_I + last_state.G_v_I * delta_t + 
  //                0.5 * (last_state.G_R_I * acc_unbias + gravity_) * delta_t2;
  // state->G_v_I = last_state.G_v_I + (last_state.G_R_I * acc_unbias + gravity_) * delta_t;
  // const Eigen::Vector3d delta_angle_axis = gyro_unbias * delta_t;
  // if (delta_angle_axis.norm() > 1e-12) {
  //   state->G_R_I = last_state.G_R_I * Eigen::AngleAxisd(delta_angle_axis.norm(),
  //                                       delta_angle_axis.normalized()).toRotationMatrix();
  // }

  const Eigen::Vector3d delta_angle_axis = gyro_unbias * delta_t;

  // More precise integration
  // Propogate the state using 4th order Runge-Kutta
  PredictNewState(delta_t, gyro_unbias, acc_unbias, state);

  // Error-state. Not needed.

  // Covariance of the error-state.   
  Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
  Fx.block<3, 3>(0, 3)   = Eigen::Matrix3d::Identity() * delta_t;
  Fx.block<3, 3>(3, 6)   = - state->G_R_I * skewSymmetric(acc_unbias) * delta_t;
  Fx.block<3, 3>(3, 9)   = - state->G_R_I * delta_t;  
  Fx.block<3, 3>(6, 12)  = - Eigen::Matrix3d::Identity() * delta_t;

  if (delta_angle_axis.norm() > 1e-12) {
    Fx.block<3, 3>(6, 6)   = Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix().transpose();
  }

  Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
  Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

  Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
  Qi.block<3, 3>(0, 0) = delta_t2 * acc_noise_ * Eigen::Matrix3d::Identity();
  Qi.block<3, 3>(3, 3) = delta_t2 * gyro_noise_ * Eigen::Matrix3d::Identity();
  Qi.block<3, 3>(6, 6) = delta_t * acc_bias_noise_ * Eigen::Matrix3d::Identity();
  Qi.block<3, 3>(9, 9) = delta_t * gyro_bias_noise_ * Eigen::Matrix3d::Identity();

  state->cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();

  // Time and imu.
  state->timestamp = cur_imu->timestamp;
  state->imu_data_ptr = cur_imu;
}

}  // namespace ImuGpsLocalization