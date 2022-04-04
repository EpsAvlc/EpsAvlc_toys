#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>

#include "pose_local_parameterization.h"

int main(int argc, char const *argv[]) {
  PoseLocalParameterization pose_param;
  pose_param.setParameter();
  double pose_data[7] = {10, 10, 10, 1, 0, 0, 0};
  Eigen::Map<Eigen::Matrix<double, 7, 1>> pose_vec(pose_data);
  // Eigen::Map
  double pose_jacobian();
  Eigen::Matrix<double, 7, 6, Eigen::RowMajor> jacobian;
  
  pose_param.ComputeJacobian(pose_data, jacobian.data());
  double disturbance[6] = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
  Eigen::Map<Eigen::Matrix<double, 6, 1>> disturbed_vec(disturbance);
  double pose_disturbed[7];
  pose_param.Plus(pose_data, disturbance, pose_disturbed);
  Eigen::Map<Eigen::Matrix<double, 7, 1>> pose_disturbed_vec(pose_disturbed);

  // TODO(caoming): mismatch. 
  std::cout << "disturbed: " << (pose_disturbed_vec - pose_vec).transpose() << std::endl;
  std::cout << "disturbed (calculated by jacobian): " << (jacobian * disturbed_vec).transpose() << std::endl;

  // Eigen::VectorXd()
  return 0;
}
