#ifndef SLIDING_GRID_AUTO_SLIDING_OCTOMAP_H
#define SLIDING_GRID_AUTO_SLIDING_OCTOMAP_H

#include <vector>
#include <array>
#include <Eigen/Core>

#include "sliding_grid/auto_sliding_grid_map.h"

namespace lpnp_mapping {

template <size_t Dim, class GridType>
class AutoSlidingOctomap {
 public:
  using VectorX    = Eigen::Matrix<double, Dim, 1>;
  using IndexXd    = Eigen::Matrix<int, Dim, 1>;
  using FineMapT   = GridMap<Dim, GridType>;
  using CoarseMapT = AutoSlidingGridMap<Dim, typename FineMapT::Ptr>;

  using IteratorCoarse      = typename CoarseMapT::Iterator;
  using ConstIteratorCoarse = typename CoarseMapT::ConstIterator;
  using IteratorFine        = typename FineMapT::Iterator;
  using ConstIteratorFine   = typename FineMapT::ConstIterator;

  struct Config {
    double                  coarse_resolution;
    double                  fine_resolution;
    std::array<double, Dim> size;
    VectorX                 origin;
    std::array<double, Dim> move_step;
    std::array<double, Dim> move_threshold;
    GridType                initial_val;
  };

  AutoSlidingOctomap(const Config& config) : config_(config) {
    typename CoarseMapT::Config asgm_config;
    asgm_config.origin         = config.origin;
    asgm_config.resolution     = config.coarse_resolution;
    asgm_config.size           = config.size;
    asgm_config.move_step      = config.move_step;
    asgm_config.move_threshold = config.move_threshold;
    asgm_config.initial_method = std::bind(&AutoSlidingOctomap::initSubMap, this, std::placeholders::_1);

    octomap_ptr_.reset(new CoarseMapT(asgm_config));
  }

  GridType& at(const VectorX& vec) { 
    // octomap_ptr_->locToIndexXd()
    return octomap_ptr_->at(vec)->at(vec); };

  ConstIteratorCoarse begin() const { return octomap_ptr_->begin(); }
  ConstIteratorCoarse end() const { return octomap_ptr_->end(); }

  double coarseResolution() const { return config_.coarse_resolution; }
  double fineResolution() const { return config_.fine_resolution; }

 private:
  typename FineMapT::Ptr initSubMap(const size_t ind_1d) {
    VectorX                   submap_origin = octomap_ptr_->indexXdToLoc(octomap_ptr_->index1dToXd(ind_1d));
    typename FineMapT::Config submap_config_;
    submap_config_.initial_val = config_.initial_val;
    submap_config_.origin      = submap_origin;
    submap_config_.resolution  = config_.fine_resolution;
    for (size_t di = 0; di < Dim; ++di) {
      submap_config_.size[di] = config_.coarse_resolution;
    }
    return std::make_shared<FineMapT>(submap_config_);
  }

  typename CoarseMapT::Ptr octomap_ptr_;
  Config                   config_;
  // std::array<double, Dim>  submap_size_;
};

}  // namespace lpnp_mapping

#endif  // SLIDING_GRID_AUTO_SLIDING_OCTOMAP_H
