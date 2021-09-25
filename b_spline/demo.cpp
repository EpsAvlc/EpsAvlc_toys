/*
 * Created on Fri Sep 24 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#include <ceres/ceres.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "b_spline/b_spline.h"

std::vector<cv::Point2d> dataToVector(const double *const data, int num) {
  std::vector<cv::Point2d> res;
  for (int i = 0; i < num / 2; ++i) {
    cv::Point2d pt;
    pt.x = data[i * 2];
    pt.y = data[i * 2 + 1];
    res.push_back(pt);
  }
  return res;
}

class BSplineCost {
 public:
  explicit BSplineCost(double t, const cv::Point2d gt_pt, const int cpt_num)
      : t_(t), gt_pt_(gt_pt), cpt_num_(cpt_num) {}

  bool operator()(const double *const cps, double *residual) const {
    std::vector<cv::Point2d> control_points = dataToVector(cps, cpt_num_ * 2);
    UniformBSpline b_spline(3, 2);
    for (int i = 0; i < b_spline.order(); ++i) {
      b_spline.addControlPoint(control_points[0]);
    }
    for (int i = 0; i < control_points.size(); ++i) {
      b_spline.addControlPoint(control_points[i]);
    }
    for (int i = 0; i < b_spline.order(); ++i) {
      b_spline.addControlPoint(control_points.back());
    }
    cv::Point2d meas_pt = b_spline.get(t_);
    residual[0] = meas_pt.x - gt_pt_.x;
    residual[1] = meas_pt.y - gt_pt_.y;
    return true;
  }

 private:
  double t_;
  cv::Point2d gt_pt_;
  int cpt_num_;
};

double pts[8] = {100, 300, 200, 300, 300, 300, 400, 300};

class BSplineCallback : public ceres::IterationCallback {
 public:
  virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) {
    for (int i = 0; i <8; ++i) {
      std::cout << pts[i] << ", ";
    }
    std::cout<< std::endl;
    return ceres::SOLVER_CONTINUE;
  }
};

int main(int argc, char const *argv[]) {
  /****************************example 2*******************************/
  cv::Mat paint2(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
  auto sin_func = [](double t) { return 50.0 * cos(2.0 * M_PI / 6.0 * t) + 300.0; };

  double pixel_per_t = 2.0 / 100;

  std::map<double, cv::Point2d> gt;
  for (int i = 100; i < 400; ++i) {
    double t = i * pixel_per_t + 8;
    int x = i;
    double y_val = sin_func(t);
    int y = static_cast<int>(sin_func(t));
    paint2.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 0);
    gt[t] = cv::Point2d(x, y);
  }

  ceres::Problem problem;
  double t_step = (2.0 * (8 + 3 - 1)) / 100.0;
  for (auto &pt_pair : gt) {
    double curr_time = pt_pair.first;
    cv::Point2d point = pt_pair.second;
    ceres::CostFunction *cost_function =
        new ceres::NumericDiffCostFunction<BSplineCost, ceres::CENTRAL, 2, 8>(new BSplineCost(curr_time, point, 4));
    problem.AddResidualBlock(cost_function, nullptr, pts);
    // problem.
  }

  ceres::Solver::Options options;
  BSplineCallback callback;
  options.callbacks.push_back(&callback);
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  options.update_state_every_iteration = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  UniformBSpline b_spline(3, 2);
  std::vector<cv::Point2d> cps = dataToVector(pts, 8);
  for (int i = 0; i < b_spline.order(); ++i) {
    b_spline.addControlPoint(cps[0]);
  }

  for (int i = 0; i < cps.size(); ++i) {
    b_spline.addControlPoint(cps[i]);
    cv::circle(paint2, cps[i], 5, cv::Scalar(255, 0, 0), 2);
    cv::putText(paint2, std::to_string(i), cps[i], cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 0, 0));
  }

  for (int i = 0; i < b_spline.order(); ++i) {
    b_spline.addControlPoint(cps.back());
  }

  for (auto &pt_pair : gt) {
    double curr_time = pt_pair.first;
    cv::Point2d point = b_spline.get(curr_time);
    static cv::Point2d last_point(-1, -1);
    if (last_point.x > 0) {
      cv::line(paint2, last_point, point, cv::Scalar(0, 0, 255));
    }
    last_point = point;
  }

  cv::imshow("paint2", paint2);
  cv::waitKey(0);
  return 0;
}
