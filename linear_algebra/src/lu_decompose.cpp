#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <iostream>

namespace la {

class ParialPivLu {

};
}

int main(int argc, char const *argv[]) {
  Eigen::Matrix2d A;
  A << 4, 6, 3, 3;
  std::cout << "original matrix: " << std::endl << A << std::endl;
  std::cout << "matrix LU: " << std::endl << A.partialPivLu().matrixLU() << std::endl;

  Eigen::PartialPivLU<Eigen::Matrix2d> lu;
  return 0;
}
