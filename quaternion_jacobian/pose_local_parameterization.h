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

#pragma once

#include <eigen3/Eigen/Dense>

// x: [tx ty tz; qx qy qz qw]
class PoseLocalParameterization {
 public:
  virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
  virtual bool ComputeJacobian(const double *x, double *jacobian) const;
  virtual bool ComputeJacobianNew(const double *x, double *jacobian) const;
  // 表示参数 x 的自由度（可能有冗余），比如四元数的自由度是4，旋转矩阵的自由度是9
  virtual int GlobalSize() const { return 7; };
  // 表示 \Delta x 所在的正切空间（tangent space）的自由度
  virtual int LocalSize() const { return 6; };
  void setParameter();

  bool is_degenerate_;
  Eigen::Matrix<double, 6, 6> V_update_;
};
