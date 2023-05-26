#include <iostream>
#include <queue>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "sliding_grid/auto_sliding_grid_map.h"
#include "sliding_grid/auto_sliding_octomap.h"

constexpr int    kImgWidth         = 800;
constexpr int    kImgHeight        = 800;
constexpr double kDistPerPixel     = 0.02;
const double     kMoveMaxRange     = std::min(kImgHeight * kDistPerPixel, kImgWidth* kDistPerPixel) / 2 * 0.6;
constexpr size_t kGridMapPixLength = 60;

// #define USE_GRIDMAP
#ifdef USE_GRIDMAP
using MapT         = lpnp_mapping::AutoSlidingGridMap<2, uchar>;
using MapIteratorT = typename MapT::ConstIterator;
#else
constexpr double kCoarseResolution = 1;
constexpr double kFineResolution   = 0.5;
using MapT                         = lpnp_mapping::AutoSlidingOctomap<2, uchar>;
using MapIteratorT                 = typename MapT::ConstIteratorCoarse;
using FineMapIteratorT             = typename MapT::ConstIteratorFine;
#endif

cv::Point2d posToPixel(const Eigen::Vector2d& loc) {
  cv::Point2d img_center{kImgWidth / 2, kImgHeight / 2};
  cv::Point2d img_loc{static_cast<int>(loc(0) / kDistPerPixel), static_cast<int>(loc(1) / kDistPerPixel)};
  return img_loc + img_center;
}

void plotGrid(const MapIteratorT& it, const cv::Rect& rect, cv::Mat* img) {
#ifdef USE_GRIDMAP
  if (*it == 0 || !it.valid()) {
    cv::rectangle(*img, rect, cv::Scalar(0, 255, 0), 1);
  } else {
    cv::rectangle(*img, rect, cv::Scalar(0, *it, 0), -1);
  }
#else
  cv::rectangle(*img, rect, cv::Scalar(0, 255, 0), 1);

#endif
  if (it.isOrigin()) {
    cv::rectangle(*img, rect, cv::Scalar(0, 0, 255), -1);
  }
}

#ifndef USE_GRIDMAP
void plotSubGrid(const FineMapIteratorT& it, const cv::Rect& rect, cv::Mat* img) {
  if (*it == 0 || !it.valid()) {
    cv::rectangle(*img, rect, cv::Scalar(0, 255, 0), 1);
  } else {
    cv::rectangle(*img, rect, cv::Scalar(0, *it, 0), -1);
    cv::rectangle(*img, rect, cv::Scalar(0, 255, 0), 1);
  }
}
#endif

void plotFullMap(const std::deque<Eigen::Vector2d>& traj, const MapT& grid_map, cv::Mat* img) {
#ifdef USE_GRIDMAP
  const double grid_pix_length = grid_map.resolution() / kDistPerPixel;
#else
  const double grid_pix_length = grid_map.coarseResolution() / kDistPerPixel;
#endif
  for (auto it = grid_map.begin(); it != grid_map.end(); ++it) {
    Eigen::Vector2d grid_loc = it.location();
#ifdef USE_GRIDMAP
    Eigen::Vector2d grid_corner = grid_loc + Eigen::Vector2d{grid_map.resolution(), grid_map.resolution()};
#else
    Eigen::Vector2d grid_corner = grid_loc + Eigen::Vector2d{grid_map.coarseResolution(), grid_map.coarseResolution()};
#endif
    cv::Rect grid_rect{posToPixel(grid_loc), posToPixel(grid_corner)};
    plotGrid(it, grid_rect, img);

#ifndef USE_GRIDMAP
    auto submap_ptr = *it;
    if (!submap_ptr || !it.valid()) {
      continue;
    }
    for (auto sub_it = submap_ptr->begin(); sub_it != submap_ptr->end(); ++sub_it) {
      Eigen::Vector2d sub_grid_loc = sub_it.location();
      auto            ind_2d       = sub_it.indexXd();
      Eigen::Vector2d sub_grid_corner =
          sub_grid_loc + Eigen::Vector2d{grid_map.fineResolution(), grid_map.fineResolution()};

      cv::Rect grid_rect{posToPixel(sub_grid_loc), posToPixel(sub_grid_corner)};
      plotSubGrid(sub_it, grid_rect, img);
    }
#endif
  }

  for (size_t pi = 0; pi < traj.size() - 1; ++pi) {
    cv::Point2d pixel0 = posToPixel(traj[pi]);
    cv::Point2d pixel1 = posToPixel(traj[pi + 1]);
    cv::line(*img, pixel0, pixel1, cv::Scalar(255, 0, 0), 2);
  }
}

Eigen::Vector2d move(const Eigen::Vector2d& curr_pos) {
  static int          dir  = 0;
  static const double step = kMoveMaxRange / 30;

  Eigen::Vector2d next_pos;
  while (1) {
    switch (dir) {
      case 0:
        next_pos = curr_pos + Eigen::Vector2d(step, 0);
        break;
      case 1:
        next_pos = curr_pos + Eigen::Vector2d(0, step);
        break;
      case 2:
        next_pos = curr_pos + Eigen::Vector2d(-step, 0);
        break;
      case 3:
        next_pos = curr_pos + Eigen::Vector2d(0, -step);
        break;
      default:
        throw std::runtime_error("invalid dir.");
    }
    if ((std::fabs(next_pos(0)) > kMoveMaxRange && ((dir + 1) % 2)) ||
        (std::fabs(next_pos(1)) > kMoveMaxRange && (dir % 2))) {
      dir = (dir + 1) % 4;
    } else {
      break;
    }
  }
  return next_pos;
}

void plotGridMap(const MapT& grid_map, cv::Mat* img) {
  for (auto it = grid_map.begin(); it != grid_map.end(); ++it) {
    auto        grid_ind = it.indexXd();
    cv::Point2d grid_loc{grid_ind[0] * kGridMapPixLength, grid_ind[1] * kGridMapPixLength};
    cv::Point2d grid_corner{grid_loc.x + kGridMapPixLength, grid_loc.y + kGridMapPixLength};
    cv::Rect    grid_rect_img{grid_loc, grid_corner};
    cv::Point2d txt_loc{grid_loc.x, grid_loc.y + kGridMapPixLength / 2};

    plotGrid(it, grid_rect_img, img);

#ifndef USE_GRIDMAP
    auto submap_ptr = *it;
    if (submap_ptr && it.valid()) {
      for (auto sub_it = submap_ptr->begin(); sub_it != submap_ptr->end(); ++sub_it) {
        auto        sub_grid_ind = sub_it.indexXd();
        cv::Point2d sub_grid_loc{grid_loc.x + sub_grid_ind[0] * kGridMapPixLength / 2,
                                 grid_loc.y + sub_grid_ind[1] * kGridMapPixLength / 2};
        cv::Point2d sub_grid_corner{sub_grid_loc.x + kGridMapPixLength / 2, sub_grid_loc.y + kGridMapPixLength / 2};
        cv::Rect    sub_grid_rect{sub_grid_loc, sub_grid_corner};
        plotSubGrid(sub_it, sub_grid_rect, img);
      }
    }
#endif

    std::stringstream coor_text_ss;
    coor_text_ss << "(" << grid_ind[0] << ", " << grid_ind[1] << ")";
    cv::putText(*img, coor_text_ss.str(), txt_loc, CV_FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 255, 255));
  }
}

int main(int argc, char const* argv[]) {
  MapT::Config config;
#ifdef USE_GRIDMAP
  config.initial_val = 0;
  // config.initial_method = []() { return 255; };
  config.size[0]      = 10;
  config.size[1]      = 10;
  config.move_step[0] = config.move_step[1] = 5;
  config.move_threshold[0] = config.move_threshold[1] = 2.5;
  config.resolution                                   = 1;
#else
  config.initial_val  = 0;
  config.size[0]      = 10;
  config.size[1]      = 10;
  config.move_step[0] = config.move_step[1] = 5;
  config.move_threshold[0] = config.move_threshold[1] = 2.5;
  config.origin                                       = {0, 0};
  config.fine_resolution                              = kFineResolution;
  config.coarse_resolution                            = kCoarseResolution;
#endif
  MapT grid_map_2d(config);

  cv::Mat paint(kImgHeight, kImgWidth, CV_8UC3, cv::Scalar(0));
#ifdef USE_GRIDMAP
  size_t grid_size[2] = {static_cast<size_t>(std::ceil(config.size[0] / config.resolution)),
                         static_cast<size_t>(std::ceil(config.size[1] / config.resolution))};
#else
  size_t grid_size[2] = {static_cast<size_t>(std::ceil(config.size[0] / config.coarse_resolution)),
                                     static_cast<size_t>(std::ceil(config.size[1] / config.coarse_resolution))};
#endif
  cv::Mat grid_map_disp(grid_size[0] * kGridMapPixLength, grid_size[1] * kGridMapPixLength, CV_8UC3, cv::Scalar(0));

  cv::Mat     full_paint(kImgHeight + 100, kImgWidth + 100 + grid_map_disp.cols, CV_8UC3, cv::Scalar::all(30));
  cv::Rect    paint_rect(cv::Point2i{50, 50}, paint.size());
  cv::Point2i gm_tl = paint_rect.tl() + cv::Point2i(kImgWidth + 50, 0);
  cv::Rect    gridmap_rect(gm_tl, grid_map_disp.size());
  std::deque<Eigen::Vector2d> deq_traj;
  Eigen::Vector2d             curr_pos{-kMoveMaxRange, -kMoveMaxRange};
  double                      traj_dist = 0;

  char c = '0';
  do {
    if (c == 'x') {
      break;
    }

    if (!deq_traj.empty()) {
      traj_dist += (curr_pos - deq_traj.back()).norm();
      if (traj_dist > kMoveMaxRange && deq_traj.size() >= 2) {
        traj_dist -= (deq_traj[0] - deq_traj[1]).norm();
        deq_traj.pop_front();
      }
    }
    deq_traj.push_back(curr_pos);
    curr_pos                 = move(curr_pos);
    grid_map_2d.at(curr_pos) = cv::saturate_cast<uchar>(grid_map_2d.at(curr_pos) + 50);

    cv::Mat paint_clone = paint.clone();
    plotFullMap(deq_traj, grid_map_2d, &paint_clone);
    // cv::imshow("map", paint_clone);

    cv::Mat grid_map_disp_clone = grid_map_disp.clone();
    plotGridMap(grid_map_2d, &grid_map_disp_clone);

    cv::Mat full_paint_clone = full_paint.clone();
    paint_clone.copyTo(full_paint_clone(paint_rect));
    grid_map_disp_clone.copyTo(full_paint_clone(gridmap_rect));
    cv::imshow("disp", full_paint_clone);
  } while (c = cv::waitKey(50));

  return 0;
}
