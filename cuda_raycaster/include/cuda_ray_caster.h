/*
 * Created on Tue Jun 22 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#ifndef ACTIVE_MAPPING_WORLD_REPRESENTATION_CUDA_ACC_CUDA_RAY_CASTER_H_
#define ACTIVE_MAPPING_WORLD_REPRESENTATION_CUDA_ACC_CUDA_RAY_CASTER_H_

#ifdef __CUDACC__
#ifndef CUDA_CALLABLE
#define CUDA_CALLABLE __device__
#endif
#else
#define CUDA_CALLABLE
#endif

#include "active_mapping/world_representation/cuda_acc/cuda_hashtable.h"
#include <octomap/octomap_types.h>
#include <vector>

namespace octomap {

struct CudaOcTreeKey {
  uint16_t k[3];
};

class CudaPoint3d {
 public:
  CUDA_CALLABLE CudaPoint3d() { data_[0] = data_[1] = data_[2] = 0.0; }

  CUDA_CALLABLE CudaPoint3d(const CudaPoint3d &pt) {
    data_[0] = pt.data_[0];
    data_[1] = pt.data_[1];
    data_[2] = pt.data_[2];
  }

  CUDA_CALLABLE float &operator()(unsigned int i) { return data_[i]; }

  CUDA_CALLABLE CudaPoint3d normalized() {
    double norm = sqrt(x() * x() + y() * y() + z() * z());
    CudaPoint3d res(*this);
    res.x() /= norm;
    res.y() /= norm;
    res.z() /= norm;
    return res;
  }

  CUDA_CALLABLE float &x() { return data_[0]; }
  CUDA_CALLABLE float &y() { return data_[1]; }
  CUDA_CALLABLE float &z() { return data_[2]; }

 private:
  float data_[3];
};

class CudaRayCaster {
 public:
  struct OcTreeData {
    std::vector<cu_uint64_t> keys;
    std::vector<cu_uint64_t> occupancy;
    double resolution;
  };

  enum OccStatus {
    Occupied = 1,
    Free = 2,
  };

  explicit CudaRayCaster(const OcTreeData &octree_data, bool print_info = true);

  bool* castRay(const std::vector<octomap::point3d> &origins,
                 const std::vector<octomap::point3d> &dirs,
                 std::vector<octomap::point3d> *end_pts, bool ignore_unknown,
                 const std::vector<double> &max_range);

 private:
  CudaHashTable cu_hash_table_;
  double resolution_;
};
}  // namespace octomap

#endif  // ACTIVE_MAPPING_WORLD_REPRESENTATION_CUDA_ACC_CUDA_RAY_CASTER_H_
