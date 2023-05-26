#include <gtest/gtest.h>
#include "sliding_grid/auto_sliding_grid_map.h"



TEST(AutoSlidingGridMap1D, move) {
  lpnp_mapping::AutoSlidingGridMap<1, double>::Config config;
  config.initial_val     = 0;
  config.size[0]        = 20;
  config.move_step      = 10;
  config.move_threshold = 5;
  config.resolution     = 0.25;
  lpnp_mapping::AutoSlidingGridMap<1, double> grid_map_1d(config);
  for (size_t pi = 6; pi < 15; ++pi) {
    grid_map_1d.at(pi) = 5;
  }
  EXPECT_EQ(grid_map_1d.origin()(0), 0.0);
  grid_map_1d.at(30) = 10;
  EXPECT_EQ(grid_map_1d.origin()(0), 20);
  grid_map_1d.at(79.8) = 10;
  EXPECT_EQ(grid_map_1d.origin()(0), 70);
  // grid_map_1d.print();
}

TEST(AutoSlidingGridMap2D, move) {
  lpnp_mapping::AutoSlidingGridMap<2, double>::Config config;
  config.initial_val     = 0;
  config.size[0]        = 10;
  config.size[1]        = 10;
  config.move_step      = 5;
  config.move_threshold = 2.5;
  config.resolution     = 1;
  lpnp_mapping::AutoSlidingGridMap<2, double> grid_map_2d(config);
}

int main(int argc, char** argv) {
  // google::InitGoogleLogging(argv[0]);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
