/*
 * Created on Tue Jun 23 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#include "active_mapping/world_representation/cuda_acc/cuda_ray_caster.h"
// #include <octomap/octomap.h>
#include <limits>

namespace octomap {

/* const values */
const int kTreeMaxVal = 32768;

/* cuda utility functions */
__device__ CudaOcTreeKey coordToKey(double resolution_factor,
                                    octomap::CudaPoint3d coord) {
  CudaOcTreeKey key;
  for (int i = 0; i < 3; ++i) {
    key.k[i] = ((int)floor(resolution_factor * coord(i))) + kTreeMaxVal;
  }
  return key;
}

__device__ double keyToCoord(double resolution, uint16_t key) {
  return (double((int)key - (int)kTreeMaxVal) + 0.5) * resolution;
}

__device__ octomap::CudaPoint3d keyToCoord(double resolution,
                                           CudaOcTreeKey key) {
  octomap::CudaPoint3d pt;
  for (int i = 0; i < 3; ++i) {
    pt(i) = (float)keyToCoord(resolution, key.k[i]);
  }
  return pt;
}

__device__ cu_uint64_t keyToIndex(const CudaOcTreeKey &key) {
  cu_uint64_t res = 0;
  res = ((cu_uint64_t)key.k[0] << 32) + ((cu_uint64_t) key.k[1] << 16) + (key.k[2]);
  return res;
}

__global__ void
kernelCastRay(KeyValue *hash_table, double resolution,
              const unsigned int cast_num, octomap::CudaPoint3d *origins,
              octomap::CudaPoint3d *dirs, octomap::CudaPoint3d *end_pts,
              bool ignore_unknown, double *max_ranges, bool *find_end_pt) {
  // octomap::CudaOcTreeKey key;
  // key.k[0] = 32798;
  // key.k[1] = 32841;
  // key.k[2] = 32784;
  unsigned int thread_id = blockIdx.x * blockDim.x + threadIdx.x;
  if (thread_id < cast_num) {
    double resolution_factor = 1 / resolution;
    // cu_uint64_t key_ind = keyToIndex(key);
    // cu_uint64_t key_occ = CudaHashTable::gpuQuery(hash_table, key_ind);
    // printf("key_ind: %lld, key_occ: %lld\n", key_ind, key_occ);
    octomap::CudaPoint3d origin = origins[thread_id];
    octomap::CudaPoint3d direction = dirs[thread_id].normalized();
    octomap::CudaPoint3d *end = &end_pts[thread_id];
    double max_range = max_ranges[thread_id];

    CudaOcTreeKey current_key = coordToKey(resolution_factor, origin);
    cu_uint64_t current_key_ind = keyToIndex(current_key);
    cu_uint64_t starting_node_occ =
        CudaHashTable::gpuQuery(hash_table, current_key_ind);
    if (starting_node_occ != vEmpty) {
      if (starting_node_occ == CudaRayCaster::OccStatus::Occupied) {
        *end = keyToCoord(resolution, current_key);
        find_end_pt[thread_id] = true;
        return;
      }
    } else if (!ignore_unknown) {
      *end = keyToCoord(resolution, current_key);
      find_end_pt[thread_id] = false;
      return;
    }

    bool max_range_set = (max_range > 0.0);

    int step[3];
    double tMax[3];
    double tDelta[3];

    for (unsigned int i = 0; i < 3; ++i) {
      // compute step direction
      if (direction(i) > 0.0)
        step[i] = 1;
      else if (direction(i) < 0.0)
        step[i] = -1;
      else
        step[i] = 0;

      // compute tMax, tDelta
      if (step[i] != 0) {
        // corner point of voxel (in direction of ray)
        double voxelBorder = keyToCoord(resolution, current_key.k[i]);
        voxelBorder += static_cast<double>(step[i] * resolution * 0.5);

        tMax[i] = (voxelBorder - origin(i)) / direction(i);
        tDelta[i] = resolution / fabs(direction(i));
      } else {
        tMax[i] = __DBL_MAX__;
        tDelta[i] = __DBL_MAX__;
      }
    }

    if (step[0] == 0 && step[1] == 0 && step[2] == 0) {
      printf(
          "[CudaRayCaster]:Raycasting in direction (0,0,0) is not possible!");
      find_end_pt[thread_id] = false;
      return;
    }
    // for speedup:
    float maxrange_sq = max_range * max_range;

    // Incremental phase
    // ---------------------------------------------------------
    bool done = false;

    while (!done) {
      // printf("[cuda_ray_caster] current key: %d, %d, %d \n", current_key.k[0], current_key.k[1], current_key.k[2]);
      unsigned int dim;

      // find minimum tMax:
      if (tMax[0] < tMax[1]) {
        if (tMax[0] < tMax[2])
          dim = 0;
        else
          dim = 2;
      } else {
        if (tMax[1] < tMax[2])
          dim = 1;
        else
          dim = 2;
      }

      // check for overflow:
      if ((step[dim] < 0 && current_key.k[dim] == 0) ||
          (step[dim] > 0 && current_key.k[dim] == 2 * kTreeMaxVal - 1)) {
        printf(
            "[CudaCastRay] Coordinate hit bounds in dim %d, aborting raycast\n",
            dim);
        // return border point nevertheless:
        *end = keyToCoord(resolution, current_key);
        find_end_pt[thread_id] = false;
        return;
      }

      // advance in direction "dim"
      current_key.k[dim] += step[dim];
      tMax[dim] += tDelta[dim];

      // generate world coords from key
      *end = keyToCoord(resolution, current_key);

      // check for maxrange:
      if (max_range_set) {
        // printf("max_range: %f\n", max_range);
        float dist_from_origin_sq = 0;
        for (unsigned int j = 0; j < 3; j++) {
          // printf("dist_from_origin_sq: %f\n", dist_from_origin_sq);
          float add_val = ((end->operator()(j) - origin(j)) *
          (end->operator()(j) - origin(j)));
          // printf("add %f\n", add_val);
          dist_from_origin_sq += add_val;
        }
        if (dist_from_origin_sq > maxrange_sq) {
          find_end_pt[thread_id] = false;
          // printf("[CudaRayCaster] reach max_range. max_range: %f, end_pt: %f, %f, %f, origin_pt: %f, %f, %f\n", 
          // max_range, end->operator()(0), end->operator()(1), end->operator()(2), origin(0), origin(1), origin(2));
          return;
        }
      }

      current_key_ind = keyToIndex(current_key);
      cu_uint64_t current_node_occ =
          CudaHashTable::gpuQuery(hash_table, current_key_ind);
      // printf("[cuda_ray_caster]current_key: %d, %d, %d, cuda_node_val : %lld\n", current_key.k[0], current_key.k[1], current_key.k[2], current_node_occ);
      if (current_node_occ != vEmpty) {
        if (current_node_occ == CudaRayCaster::OccStatus::Occupied) {
          done = true;
          // printf("[CudaRayCaster] hit occ in %d, %d, %d\n", current_key.k[0], current_key.k[1], current_key.k[2]);
          break;
        }
        // otherwise: node is free and valid, raycasting continues
      } else if (!ignore_unknown) {  // no node found, this usually means we are
                                     // in "unknown" areas
        find_end_pt[thread_id] = false;
        // printf("[CudaRayCaster] hit unknown in %d, %d, %d", current_key.k[0], current_key.k[1], current_key.k[2]);
        return;
      }
    }  // end while

    find_end_pt[thread_id] = true;
    return;
  }
}

/* CudaRayCaster methods. */
CudaRayCaster::CudaRayCaster(const OcTreeData &octree_data, bool print_info)
    : resolution_(octree_data.resolution) {
  int node_num = octree_data.keys.size();
  if (print_info) {
    std::cout << "[CudaRayCaster] octree node num: " << node_num << std::endl;
  }
  std::vector<KeyValue> kvs(node_num);
  for (int i = 0; i < node_num; ++i) {
    kvs[i].key = octree_data.keys[i];
    kvs[i].value = octree_data.occupancy[i];
  }
  cu_hash_table_.insert(kvs.data(), node_num);
}

bool*
CudaRayCaster::castRay(const std::vector<octomap::point3d> &origins,
                       const std::vector<octomap::point3d> &dirs,
                       std::vector<octomap::point3d> *end_pts,
                       bool ignore_unknown,
                       const std::vector<double> &max_range) {
  int cast_size = origins.size();
  CudaPoint3d *device_origins;
  cudaMalloc(&device_origins, sizeof(point3d) * cast_size);
  cudaMemcpy(device_origins, origins.data(), sizeof(point3d) * cast_size,
             cudaMemcpyHostToDevice);

  CudaPoint3d *device_dirs;
  cudaMalloc(&device_dirs, sizeof(point3d) * cast_size);
  cudaMemcpy(device_dirs, dirs.data(), sizeof(point3d) * dirs.size(),
             cudaMemcpyHostToDevice);

  CudaPoint3d *device_end_pts;
  cudaMalloc(&device_end_pts, sizeof(point3d) * cast_size);

  double *device_max_ranges;
  cudaMalloc(&device_max_ranges, sizeof(double) * cast_size);
  cudaMemcpy(device_max_ranges, max_range.data(),
             sizeof(double) * cast_size, cudaMemcpyHostToDevice);

  bool *device_find_end_pts;
  cudaMalloc(&device_find_end_pts, sizeof(bool) * cast_size);

  int mingridsize;
  int threadblocksize;
  cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize,
                                     kernelCastRay, 0, 0);

  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);

  cudaEventRecord(start);

  int gridsize =
      ((cu_uint64_t)origins.size() + threadblocksize - 1) / threadblocksize;
  kernelCastRay<<<gridsize, threadblocksize>>>(
      cu_hash_table_.data(), resolution_, cast_size, device_origins,
      device_dirs, device_end_pts, ignore_unknown, device_max_ranges,
      device_find_end_pts);

  bool* find_end_pts = new bool[cast_size];
  cudaMemcpy(find_end_pts, device_find_end_pts, sizeof(bool) * cast_size, cudaMemcpyDeviceToHost);

  cudaEventRecord(stop);
  cudaEventSynchronize(stop);

  cudaFree(device_origins);
  cudaFree(device_dirs);
  cudaFree(device_end_pts);
  cudaFree(device_max_ranges);

  return find_end_pts;
}

}  // namespace octomap
