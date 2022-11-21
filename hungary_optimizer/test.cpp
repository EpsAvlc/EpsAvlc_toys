#include "hungary_optimizer.h"

int main(int argc, char const *argv[]) {
  Eigen::Matrix3d cost_matrix;
  cost_matrix << 1, 2, 3, 2, 4, 6, 3, 6, 9;
  std::cout << "ori cost matrix: " << std::endl;
  std::cout << cost_matrix << std::endl;
  HungaryOptimizer optimizer;
  std::vector<int> results;
  optimizer.minimize(cost_matrix, &results, true);
  return 0;
}
