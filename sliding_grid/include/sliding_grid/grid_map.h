#ifndef SLIDING_GRID_GRID_MAP_H
#define SLIDING_GRID_GRID_MAP_H

#include <Eigen/Core>
#include <vector>
#include <memory>
#include <functional>

#include "sliding_grid/global.h"

namespace lpnp_mapping {

template <size_t Dim, class GridType>
class GridMap {
 public:
  using VectorX       = Eigen::Matrix<double, Dim, 1>;
  using IndexXd       = Eigen::Matrix<int, Dim, 1>;
  using Index1d       = size_t;
  using Ptr           = std::shared_ptr<GridMap>;
  using DynamicBitset = std::vector<bool>;

  template <typename T>
  struct IteratorTemplate;
  using Iterator      = IteratorTemplate<GridType>;
  using ConstIterator = IteratorTemplate<const GridType>;

  struct Config {
    double                                        resolution;
    std::array<double, Dim>                       size;
    std::function<GridType(const Index1d ind_1d)> initial_method;
    GridType                                      initial_val;
    VectorX                                       origin;
  };

  GridMap(const Config& config) : config_(config) {
    GL_EXPECT_TRUE(config_.resolution > 0);
    size_t num_grids = 1;
    for (size_t di = 0; di < Dim; ++di) {
      GL_EXPECT_TRUE(config_.size[di] > 0);
      grid_size_[di] = static_cast<size_t>(std::ceil(config_.size[di] / config_.resolution));
      num_grids *= grid_size_[di];
    }
    grids_.resize(num_grids);
    grid_status_.resize(num_grids, false);
    GL_LOG("Create a grid map with num grids: %ld", num_grids);
    origin_ = config_.origin;
  }

  ConstIterator begin() const { return ConstIterator(*this, &grids_[0]); }

  ConstIterator end() const { return ConstIterator(*this, &grids_.data()[grids_.size()]); }

  template <size_t U = Dim>
  typename std::enable_if<U == 1, GridType>::Type& at(const double& x) {
    GL_STATIC_EXPECT_TRUE(Dim == 1);
    VectorX loc;
    loc(0) = x;
    return at(loc);
  }

  virtual GridType& at(const VectorX& x) {
    IndexXd ind_xd = locToIndexXd(x);
    Index1d ind_1d = indexXdTo1d(ind_xd);
    tryInitGrid(ind_1d);
    // return grids_[ind_1d];
    return grids_.at(ind_1d);
  }

  const VectorX& origin() const { return origin_; }

  const double& resolution() const { return config_.resolution; }

  Index1d indexXdTo1d(const IndexXd& ind) {
    size_t  factor = 1;
    Index1d ret    = 0;
    for (int di = Dim - 1; di >= 0; --di) {
      ret += factor * ind[di];
      factor *= grid_size_[di];
    }
    return ret;
  }

  IndexXd index1dToXd(const Index1d& ind) const {
    size_t  factor = 1;
    IndexXd ret;
    for (int di = Dim - 1; di >= 0; --di) {
      ret[di] = ind / factor % grid_size_[di];
      factor *= grid_size_[di];
    }
    return ret;
  }

  virtual VectorX indexXdToLoc(const IndexXd& ind) const {
    VectorX loc;
    for (size_t di = 0; di < Dim; ++di) {
      int diff = ind[di];
      GL_EXPECT_TRUE(diff >= 0);
      loc(di) = origin_(di) + diff * config_.resolution;
    }
    return loc;
  }

  virtual IndexXd locToIndexXd(const VectorX& x) const {
    IndexXd grid_ind_per_dim;
    for (size_t di = 0; di < Dim; ++di) {
      GL_EXPECT_TRUE(x(di) >= origin_(di) && x(di) <= origin_(di) + config_.size[di]);
      grid_ind_per_dim[di] = static_cast<size_t>(std::floor((x(di) - origin_(di)) / config_.resolution));
    }
    return grid_ind_per_dim;
  }

 protected:
  void tryInitGrid(const Index1d& ind) {
    if (grid_status_[ind]) {
      return;
    }
    grid_status_[ind] = true;
    if (config_.initial_method) {
      grids_[ind] = config_.initial_method(ind);
    } else {
      grids_[ind] = config_.initial_val;
    }
  }

  std::vector<GridType>   grids_;
  std::array<size_t, Dim> grid_size_;
  VectorX                 origin_;
  DynamicBitset           grid_status_;

 private:
  const Config config_;
};

template <size_t Dim, class GridType>
template <typename T>
struct GridMap<Dim, GridType>::IteratorTemplate {
  using iterator_category = std::bidirectional_iterator_tag;
  using value_type        = T;
  using difference_type   = std::ptrdiff_t;
  using pointer           = T*;
  using reference         = T&;

  IteratorTemplate(const GridMap& grid_map, pointer const ptr) : grid_map_(grid_map), ptr_(ptr) {}

  reference operator*() const { return *ptr_; }
  pointer   operator->() const { return *ptr_; }

  IteratorTemplate& operator++() {
    ptr_++;
    return *this;
  }

  IteratorTemplate& operator++(int) {
    IteratorTemplate tmp = *this;
    ++(*this);
    return tmp;
  }

  friend bool operator==(const IteratorTemplate& lhs, const IteratorTemplate& rhs) { return lhs.ptr_ == rhs.ptr_; }

  friend bool operator!=(const IteratorTemplate& lhs, const IteratorTemplate& rhs) { return !(lhs == rhs); }

  VectorX location() const {
    Index1d ind    = ptr_ - grid_map_.begin().ptr_;
    IndexXd ind_xd = grid_map_.index1dToXd(ind);
    return grid_map_.indexXdToLoc(ind_xd);
  }

  IndexXd indexXd() const {
    Index1d ind    = ptr_ - grid_map_.begin().ptr_;
    IndexXd ind_xd = grid_map_.index1dToXd(ind);
    return ind_xd;
  }

  Index1d index1d() const {
    Index1d ind = ptr_ - grid_map_.begin().ptr_;
    return ind;
  }

  bool valid() const {
    size_t ind = ptr_ - grid_map_.begin().ptr_;
    return grid_map_.grid_status_[ind];
  }

  virtual bool isOrigin() const {
    Index1d ind = ptr_ - grid_map_.begin().ptr_;
    return ind == 0;
  }

 protected:
  pointer ptr_;

 private:
  GridMap const& grid_map_;
};
}  // namespace lpnp_mapping

#endif  // SLIDING_GRID_GRID_MAP_H
