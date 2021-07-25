/*
 * Created on Tue Jun 22 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#include "active_mapping/world_representation/cuda_acc/cuda_hashtable.h"
#include <stdio.h>
#include <iostream>

CudaHashTable::~CudaHashTable() {
  cudaFree(hash_table_);
}

CUDA_CALLABLE cu_uint64_t hash(cu_uint64_t k) {
  k ^= k >> 16;
  k *= 0x85ebca6b;
  k ^= k >> 13;
  k *= 0xc2b2ae35;
  k ^= k >> 16;
  return k & (kHashTableCapacity - 1);
}

__global__ void gpu_hashtable_insert(KeyValue *hashtable, const KeyValue *kvs,
                                     const unsigned int numkvs) {
  unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
  if (threadid < numkvs) {
    cu_uint64_t key = kvs[threadid].key;
    cu_uint64_t value = kvs[threadid].value;
    cu_uint64_t slot = hash(key);

    while (true) {
      cu_uint64_t prev = atomicCAS(&hashtable[slot].key, kEmpty, key);
      if (prev == kEmpty || prev == key) {
        hashtable[slot].value = value;
        return;
      } 
      printf("key: %d, slot: %d, value: %d\n", key, slot, value);

      slot = (slot + 1) & (kHashTableCapacity - 1);
    }
  }
}

void CudaHashTable::insert(const KeyValue *kvs, cu_uint64_t num_kvs) {
  // Copy the keyvalues to the GPU
  KeyValue* device_kvs;
  cudaMalloc(&device_kvs, sizeof(KeyValue) * num_kvs);
  cudaMemcpy(device_kvs, kvs, sizeof(KeyValue) * num_kvs, cudaMemcpyHostToDevice);

  // Have CUDA calculate the thread block size
  int mingridsize;
  int threadblocksize;
  cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, gpu_hashtable_insert, 0, 0);

  // Create events for GPU timing
  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);

  cudaEventRecord(start);

  // Insert all the keys into the hash table
  int gridsize = ((cu_uint64_t)num_kvs + threadblocksize - 1) / threadblocksize;
  gpu_hashtable_insert<<<gridsize, threadblocksize>>>(hash_table_, device_kvs, (cu_uint64_t)num_kvs);

  cudaEventRecord(stop);

  cudaEventSynchronize(stop);

  float milliseconds = 0;
  cudaEventElapsedTime(&milliseconds, start, stop);
  float seconds = milliseconds / 1000.0f;
  // printf("    GPU inserted %lld items in %f ms (%f million keys/second)\n", 
      // num_kvs, milliseconds, num_kvs / (double)seconds / 1000000.0f);

  cudaFree(device_kvs);
}

CUDA_CALLABLE cu_uint64_t CudaHashTable::gpuQuery(const KeyValue* hash_table, cu_uint64_t key) {
  cu_uint64_t slot = hash(key);
  while (true) {
    cu_uint64_t prev_key = hash_table[slot].key;
    if (prev_key == kEmpty) {
      return vEmpty;
    } else if (prev_key == key) {
      // printf("query: key: %d, slot: %d, value: %d\n", key, slot, hash_table[slot].value);
      return hash_table[slot].value;
    }

    slot = (slot + 1) & (kHashTableCapacity - 1);
  }
}

__global__ void gpu_hashtable_query(const KeyValue * hash_table, KeyValue *kvs,
                                     const unsigned int numkvs) {
  unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
  if (threadid < numkvs) {
    cu_uint64_t key = kvs[threadid].key;
    cu_uint64_t slot = hash(key);
    kvs[threadid].value = CudaHashTable::gpuQuery(hash_table, key);
  }
}

void CudaHashTable::query(KeyValue *kvs, cu_uint64_t num_kvs) {
  KeyValue* device_kvs;
  cudaMalloc(&device_kvs, sizeof(KeyValue) * num_kvs);
  cudaMemcpy(device_kvs, kvs, sizeof(KeyValue) * num_kvs, cudaMemcpyHostToDevice);

  // Have CUDA calculate the thread block size
  int mingridsize;
  int threadblocksize;
  cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, gpu_hashtable_insert, 0, 0);

  // Create events for GPU timing
  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);

  cudaEventRecord(start);

  // Insert all the keys into the hash table
  int gridsize = ((cu_uint64_t)num_kvs + threadblocksize - 1) / threadblocksize;
  gpu_hashtable_query<<<gridsize, threadblocksize>>>(hash_table_, device_kvs, (cu_uint64_t)num_kvs);

  cudaEventRecord(stop);

  cudaMemcpy(kvs, device_kvs, sizeof(KeyValue) * num_kvs, cudaMemcpyDeviceToHost);
  cudaEventSynchronize(stop);

  float milliseconds = 0;
  cudaEventElapsedTime(&milliseconds, start, stop);
  float seconds = milliseconds / 1000.0f;
  printf("    GPU  %lld items in %f ms (%f million keys/second)\n", 
      num_kvs, milliseconds, num_kvs / (double)seconds / 1000000.0f);

  cudaFree(device_kvs);
}

void CudaHashTable::createHashTable() {
  // Allocate memory
  cudaMalloc(&hash_table_, sizeof(KeyValue) * kHashTableCapacity);

  // Initialize hash table to empty
  cudaMemset(hash_table_, 0xff, sizeof(KeyValue) * kHashTableCapacity);
}

// void CudaHashT
