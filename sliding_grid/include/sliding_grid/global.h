#ifndef SLIDING_GRID_GLOBAL_H
#define SLIDING_GRID_GLOBAL_H

#include <exception>
#ifdef STAND_ALONE_BUILD
#include <glog/logging.h>
#else
#include <mlog_core/mlog.h>
#endif

#define GL_EXPECT_TRUE(expr) \
  if (!(expr)) throw std::runtime_error(#expr " is false")
#define GL_STATIC_EXPECT_TRUE(expr) static_assert(expr, #expr " is false")
#ifdef STAND_ALONE_BUILD
#define GL_LOG(FMT, ...) printf(FMT"\n", ##__VA_ARGS__) 
#else
#define GL_LOG MLOG_INFO_F//
#endif

#endif  // SLIDING_GRID_GLOBAL_H
