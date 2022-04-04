/*******************************************************
 * Copyright (C) 2020, RAM-LAB, Hong Kong University of Science and Technology
 *
 * This file is part of M-LOAM (https://ram-lab.com/file/jjiao/m-loam).
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Jianhao JIAO (jiaojh1994@gmail.com)
 *******************************************************/

#include "pose_local_parameterization.h"

template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta) {
  typedef typename Derived::Scalar Scalar_t;

  Eigen::Quaternion<Scalar_t> dq;
  Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
  half_theta /= static_cast<Scalar_t>(2.0);
  dq.w() = static_cast<Scalar_t>(1.0);
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

void PoseLocalParameterization::setParameter() {
  is_degenerate_ = false;
  V_update_ = Eigen::Matrix<double, 6, 6>::Identity();
}

// state update
// description of update rule: LIC-Fusion: LiDAR-Inertial-Camera Odometry, IROS 2019
// description of solution remapping: On Degeneracy of Optimization-based State Estimation Problems, ICRA 2016
// The pointer coeffs must reference the four coefficients of Quaternion in the following order: *coeffs == {x, y, z, w}
bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const {
  Eigen::Map<const Eigen::Vector3d> p(x);
  Eigen::Map<const Eigen::Quaterniond> q(x + 3);

  Eigen::Map<const Eigen::Matrix<double, 6, 1>> dx(delta);  // dx = [dp, dq]
  Eigen::Matrix<double, 6, 1> dx_update = V_update_ * dx;
  Eigen::Vector3d dp(dx_update.head<3>());
  Eigen::Quaterniond dq = deltaQ(dx_update.tail<3>());

  // Eigen::Map<const Eigen::Vector3d> dp(delta);
  // Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3)); // using theta to
  // approximate q

  Eigen::Map<Eigen::Vector3d> p_plus(x_plus_delta);
  Eigen::Map<Eigen::Quaterniond> q_plus(x_plus_delta + 3);
  p_plus = p + dp;
  q_plus = (q * dq).normalized();  // q = _q * [0.5*delta, 1] 右乘

  return true;
}

// calculate the jacobian of [p, q] w.r.t [dp, dq]
// turtial: https://blog.csdn.net/hzwwpgmwy/article/details/86490556
// 这里为什么是这种形式？
bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  j.topRows<6>().setIdentity();  //旋转部分为什么是Identity?
  j.bottomRows<1>().setZero();
  return true;
}

// bool PoseLocalParameterization::ComputeJacobianNew(const double *x, double *jacobian) const {
//   Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
//   j.topRows<6>().setIdentity();  //旋转部分为什么是Identity?
//   j.bottomRows<1>().setZero();
//   return true;
// }

//
