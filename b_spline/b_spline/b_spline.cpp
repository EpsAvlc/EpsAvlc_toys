/*
 * Created on Fri Sep 24 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#include "b_spline/b_spline.h"

UniformBSpline::UniformBSpline(const int order, const double dt) : k_(order), dt_(dt) {}

cv::Point2d UniformBSpline::get(double t) {
  cv::Point2d res;
  res.x = 0;
  res.y = 0;
  for (int i = 0; i < control_points_.size(); ++i) {
    res += B(i, k_, t) * control_points_[i];
  }
  return res;
}

void UniformBSpline::addControlPoint(const cv::Point2d& pt) {
  control_points_.push_back(pt);
}

double UniformBSpline::B(int i, int j, double t) {
  if (b_.count(cv::Vec2b(i, j)) == 0) {
    createB(i, j);
  }
  return b_[cv::Vec2b(i, j)](t);
}

void UniformBSpline::createB(int i, int j) {
  if (j == 0) {
    b_[cv::Vec2b(i, j)] = [=](double t) {
      if (i * dt_ <= t && t < (i + 1) * dt_) {
        return 1;
      } else {
        return 0;
      }
    };
  } else {
    auto lhs_ind = cv::Vec2b(i, j - 1);
    auto rhs_ind = cv::Vec2b(i + 1, j - 1);
    if (b_.count(lhs_ind) == 0) {
      createB(i, j - 1);
    }
    if (b_.count(rhs_ind) == 0) {
      createB(i + 1, j - 1);
    }
    b_[cv::Vec2b(i, j)] = [=](double t) {
      double t0 = i * dt_;
      double t1 = (i + j + 1) * dt_;
      return (t - t0) / (j * dt_) * b_[lhs_ind](t) +
             (t1 - t) / (j * dt_) * b_[rhs_ind](t);
    };
  }
}
