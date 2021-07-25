/*
 * Created on Tue Jun 22 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#ifndef ACTIVE_MAPPING_WORLD_REPRESENTATION_CUDA_ACC_CUDA_HASHTABLE_H_
#define ACTIVE_MAPPING_WORLD_REPRESENTATION_CUDA_ACC_CUDA_HASHTABLE_H_

// #include <stdint.h>

typedef unsigned long long cu_uint64_t;

#ifdef __CUDACC__
#ifndef CUDA_CALLABLE
#define CUDA_CALLABLE __device__
#endif
#else
#define CUDA_CALLABLE
#endif

struct KeyValue {
  cu_uint64_t key;
  cu_uint64_t value;
};

class CudaHashTable {
 public:
  CudaHashTable() { createHashTable(); }

  ~CudaHashTable();

  void insert(const KeyValue *kvs, cu_uint64_t num_kvs);

  void query(KeyValue *kvs, cu_uint64_t num_kvs);

  CUDA_CALLABLE cu_uint64_t static gpuQuery(const KeyValue *hash_table,
                                         cu_uint64_t key);

  KeyValue * data() { return hash_table_; }
 private:
  /**
   * @brief Create a Hash Table. It is simply an array.
   *
   */
  void createHashTable();

  KeyValue *hash_table_;
};

const cu_uint64_t kHashTableCapacity = 128 * 1024 * 1024;
const cu_uint64_t kNumKeyValues = kHashTableCapacity / 2;
const cu_uint64_t kEmpty = 0xffffffffffffffff;
const cu_uint64_t vEmpty = kEmpty;

#endif  // ACTIVE_MAPPING_WORLD_REPRESENTATION_CUDA_ACC_CUDA_HASHTABLE_H_
