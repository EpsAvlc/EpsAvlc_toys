#ifndef SLIDING_GRID_SLIDING_GRID_H
#define SLIDING_GRID_SLIDING_GRID_H

#include <Eigen/Core>
#include <exception>
#include <vector>
#include <iterator>
#ifdef STAND_ALONE_BUILD
#include "sliding_grid/global.h"
#include "sliding_grid/grid_map.h"
#else
#include "lpnp_mapper/common/sliding_grid/global.h"
#endif

// #define VIS_DEBUG

#ifdef VIS_DEBUG
#include "sliding_grid/grid_output_maker.h"
#endif

namespace lpnp_mapping {
template <size_t Dim, class GridType>
class AutoSlidingGridMap : public GridMap<Dim, GridType> {
 public:
  using BaseT = GridMap<Dim, GridType>;
  using typename BaseT::Index1d;
  using typename BaseT::IndexXd;
  using typename BaseT::VectorX;
  using Ptr = std::shared_ptr<AutoSlidingGridMap>;

  template <typename T>
  struct IteratorTemplate;
  using Iterator      = IteratorTemplate<GridType>;
  using ConstIterator = IteratorTemplate<const GridType>;

  struct Config : public BaseT::Config {
    std::array<double, Dim> move_threshold;
    std::array<double, Dim> move_step;
  };

  AutoSlidingGridMap(const Config& config) : config_(config), BaseT(config) {
    GL_EXPECT_TRUE(config_.resolution > 0);
    GL_LOG("Grid size: %ld", grids_.size());
    origin_grid_ind_.setZero();
  }

  ConstIterator begin() const { return ConstIterator(*this, &grids_[0]); }

  ConstIterator end() const { return ConstIterator(*this, &grids_.data()[grids_.size()]); }

  GridType& at(const VectorX& x) override {
    // GL_LOG("call at in x: %lf, %lf, %lf", x(0), x(1), x(2));
    for (size_t di = 0; di < Dim; ++di) {
      double move_dist = calcMoveDist(di, x(di));
      if (move_dist != 0) {
        move(di, move_dist);
      }
    }
    return atImpl(x);
  }

  const VectorX& origin() const { return origin_; }

  const double& resolution() const { return config_.resolution; }

  IndexXd locToIndexXd(const VectorX& x) const override {
    IndexXd grid_ind_per_dim;
    for (size_t di = 0; di < Dim; ++di) {
      // GL_LOG("x(di): %lf, origin_(di): %lf, config_.size[di]: %lf", x(di), origin_(di), config_.size[di]);
      GL_EXPECT_TRUE(x(di) >= origin_(di) && x(di) <= origin_(di) + config_.size[di]);
      grid_ind_per_dim[di] =
          (origin_grid_ind_(di) + static_cast<size_t>(std::floor((x(di) - origin_(di)) / config_.resolution))) %
          grid_size_[di];
    }
    return grid_ind_per_dim;
  }

  VectorX indexXdToLoc(const IndexXd& ind) const override {
    VectorX loc;
    for (size_t di = 0; di < Dim; ++di) {
      int diff = ind[di] - origin_grid_ind_[di];
      if (diff < 0) {
        diff += grid_size_[di];
      }
      loc(di) = origin_(di) + diff * config_.resolution;
    }
    return loc;
  }

 private:
  double calcMoveDist(const size_t dim, const double& x) {
    if (x + config_.move_threshold[dim] >= origin_[dim] + config_.size[dim]) {
      return x + config_.move_step[dim] - origin_[dim] - config_.size[dim];
    } else if (x - config_.move_threshold[dim] <= origin_[dim]) {
      return x - config_.move_step[dim] - origin_[dim];
    } else {
      return 0;
    }
  }

  void move(const size_t dim, const double dist) {
    // GL_LOG("move_dist: %lf, dim: %ld", dist, dim);
    // GL_LOG("before move origin: %lf, %lf, %lf", origin_(0), origin_(1), origin_(2));
    int    origin_grid_move_dist = std::floor(dist / config_.resolution);
    double origin_move_dist      = origin_grid_move_dist * config_.resolution;
    origin_[dim] += origin_move_dist;
    // GL_LOG("after move origin: %lf, %lf, %lf", origin_(0), origin_(1), origin_(2));
    size_t prev_origin_grid_ind = origin_grid_ind_[dim];
    if (origin_grid_move_dist < 0) {
      origin_grid_move_dist += grid_size_[dim];
    }
    origin_grid_ind_[dim] = (prev_origin_grid_ind + origin_grid_move_dist) % grid_size_[dim];

    if (dist >= config_.size[dim]) {
      GL_LOG("clear full grid.");
      clearGrids(dim, 0, grid_size_[dim]);
    } else {
      clearGrids(dim, prev_origin_grid_ind, origin_grid_ind_[dim]);
    }
  }

  void clearGrids(const size_t dim, size_t start, size_t end) {
    GL_EXPECT_TRUE(start >= 0 && start <= grid_size_[dim] && end >= 0 && end <= grid_size_[dim]);
    // size_t range_end = end <= start ? end + grid_size_[dim] : end;
    std::vector<long> v_possible_range_start{start, start, end};
    std::vector<long> v_possible_range_end{end, end + grid_size_[dim], start};

    size_t range_start = 0, range_end = 0;
    size_t min_range = grid_size_[dim] + 1;
    for (size_t pi = 0; pi < v_possible_range_start.size(); ++pi) {
      int clear_range = v_possible_range_end[pi] - v_possible_range_start[pi];
      if (clear_range > 0 && clear_range < min_range) {
        min_range   = clear_range;
        range_start = v_possible_range_start[pi];
        range_end   = v_possible_range_end[pi];
      }
    }

    GL_LOG("Clear grids in dim %ld from %ld to %ld, min_range: %ld.", dim, range_start, range_end, min_range);
    for (size_t ri = range_start; ri < range_end; ++ri) {
      clearOneDim(dim, ri % grid_size_[dim]);
    }
  }

  void clearOneDim(const size_t dim, const size_t val) {
    IndexXd ind;
    clearOneDimRecusive(dim, val, 0, &ind);
  }

  void clearOneDimRecusive(const size_t select_dim, const size_t val, const size_t di, IndexXd* ind) {
    GL_EXPECT_TRUE(di < Dim);
    if (di != select_dim) {
      for (size_t ri = 0; ri < grid_size_[di]; ++ri) {
        (*ind)(di) = ri;
        if (di == Dim - 1) {
          size_t grid_ind        = BaseT::indexXdTo1d(*ind);
          grid_status_[grid_ind] = false;
        } else {
          clearOneDimRecusive(select_dim, val, di + 1, ind);
        }
      }
    } else {
      (*ind)(di) = val;
      if (di == Dim - 1) {
        size_t grid_ind        = BaseT::indexXdTo1d(*ind);
        grid_status_[grid_ind] = false;
      } else {
        clearOneDimRecusive(select_dim, val, di + 1, ind);
      }
    }
  }

  GridType& atImpl(const VectorX& x) {
    // GL_LOG("call atImpl in x: %lf, %lf, %lf", x(0), x(1), x(2));
    IndexXd grid_ind_xd = locToIndexXd(x);
    Index1d grid_ind_1d = BaseT::indexXdTo1d(grid_ind_xd);

    BaseT::tryInitGrid(grid_ind_1d);

    return grids_.at(grid_ind_1d);
  }

  using BaseT::grid_size_;
  using BaseT::grid_status_;
  using BaseT::grids_;
  using BaseT::origin_;
  const Config               config_;
  Eigen::Matrix<int, Dim, 1> origin_grid_ind_;
};

template <size_t Dim, class GridType>
template <typename T>
struct AutoSlidingGridMap<Dim, GridType>::IteratorTemplate : public BaseT::template IteratorTemplate<T> {
  using IteratorBaseT = typename BaseT::template IteratorTemplate<T>;
  using typename IteratorBaseT::iterator_category;
  using value_type      = T;
  using difference_type = std::ptrdiff_t;
  using pointer         = T*;
  using reference       = T&;

  IteratorTemplate(const AutoSlidingGridMap& grid_map, pointer const ptr)
      : grid_map_(grid_map), IteratorBaseT(grid_map, ptr) {}

  bool isOrigin() const override {
    auto index_xd = IteratorBaseT::indexXd();
    return grid_map_.origin_grid_ind_ == index_xd;
  }

 private:
  using IteratorBaseT::ptr_;
  AutoSlidingGridMap const& grid_map_;
};

};  // namespace lpnp_mapping

#endif  // SLIDING_GRID_SLIDING_GRID_H
