/*
 * Created on Fri Sep 24 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#pragma once
#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <memory>
#include <opencv2/core.hpp>

class UniformBSpline {
 public:
  using Ptr = std::shared_ptr<UniformBSpline>;
  explicit UniformBSpline(const int order, const double dt);
  cv::Point2d get(double t);
  int order() { return k_; }
  double dt() { return dt_; }
  void addControlPoint(const cv::Point2d &pt);

 private:
  struct Vec2bHash {
    size_t operator()(const cv::Vec2b &i) const { return size_t(i(0)) + 1447 * size_t(i(1)); }
  };

  std::vector<cv::Point2d> control_points_;
  double B(int i, int j, double t);
  void createB(int i, int j);
  std::unordered_map<cv::Vec2b, std::function<double(double)>, Vec2bHash> b_;
  int k_;
  double dt_;
};
