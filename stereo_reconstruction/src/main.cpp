/*
 * Created on Mon Jun 14 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <chrono>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "stereo_reconstruction/stereo_reconstruction.h"

class TicToc {
 public:
  void tic() {
    last_time_ = std::chrono::system_clock::now();
  }

  double toc() {
    std::chrono::milliseconds dt =
      std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::system_clock::now() - last_time_);
    return dt.count();
  }
 private:
  std::chrono::system_clock::time_point last_time_;
};

int main(int argc, char const *argv[]) {
  cv::Mat left_img = cv::imread("/home/cm/Projects/EpsAvlc_toys/"
    "stereo_reconstruction/pics/left2.png");
  cv::Mat right_img = cv::imread("/home/cm/Projects/EpsAvlc_toys/"
    "stereo_reconstruction/pics/right2.png");
  cv::imshow("left2", left_img);
  cv::imshow("right2", right_img);
  cv::cvtColor(left_img, left_img, CV_BGR2GRAY);
  cv::cvtColor(right_img, right_img, CV_BGR2GRAY);
  cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create();

  bm->setPreFilterType(CV_STEREO_BM_NORMALIZED_RESPONSE);
  int numberOfDisparities = ((left_img.cols / 8) + 15) & -16;
  bm->setPreFilterSize(9);
  bm->setPreFilterCap(63);
  bm->setBlockSize(21);
  bm->setMinDisparity(-16);
  bm->setNumDisparities(numberOfDisparities);
  bm->setTextureThreshold(10);
  bm->setUniquenessRatio(5);
  bm->setSpeckleWindowSize(100);
  bm->setSpeckleRange(32);
  cv::Mat disp;
  TicToc tt;
  tt.tic();
  bm->compute(left_img, right_img, disp);
  std::cout << "bm: " << tt.toc() << std::endl;
  disp.convertTo(disp, CV_8U, 255 / (numberOfDisparities*16.));
  cv::imshow("disp", disp);
  cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,  // int minDisparity
                                    96,     // int numDisparities
                                    15,      // int SADWindowSize
                                    600,    // int P1 = 0
                                    2400,   // int P2 = 0
                                    10,     // int disp12MaxDiff = 0
                                    16,     // int preFilterCap = 0
                                    2,      // int uniquenessRatio = 0
                                    20,    // int speckleWindowSize = 0
                                    30,     // int speckleRange = 0
                                    true);  // bool fullDP = false
  tt.tic();
  cv::Mat disparity;
  sgbm->compute(left_img, right_img, disparity);
  std::cout << "sgbm: " << tt.toc() << std::endl;

  disparity.convertTo(disp, CV_8U, 255 / (numberOfDisparities*16.));
  cv::imshow("disp2", disp);
  cv::waitKey(0);


  cv::Mat cam_k_left = (cv::Mat_<double>(3, 3) << 458.654, 0, 367.215, 0,
    457.296, 248.375, 0, 0, 1);
  cv::Mat cam_distor_left = (cv::Mat_<double>(1, 4) <<
    -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
  cv::Mat cam_k_right = (cv::Mat_<double>(3, 3) << 457.587, 0, 379.999, 0,
    456.134, 255.238, 0, 0, 1);
  cv::Mat cam_distor_right = (cv::Mat_<double>(1, 4) <<
    -0.28368365,  0.07451284, -0.00010473, -3.55590700e-0);
  cv::Mat cam_extrins_left = (cv::Mat_<double>(4, 4)
    << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0);
  cv::Mat cam_extrins_right = (cv::Mat_<double>(4, 4)
        << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
         0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
        -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
         0.0, 0.0, 0.0, 1.0);
  cv::Mat stereo_extrins = cam_extrins_left.inv() * cam_extrins_right;
  cv::Mat R1, R2, P1, P2, Q;
  cv::stereoRectify(cam_k_left,
                    cam_distor_left,
                    cam_k_right,
                    cam_distor_right,
                    left_img.size(),
                    stereo_extrins(cv::Range(0, 3), cv::Range(0, 3)),
                    stereo_extrins(cv::Range(0, 3), cv::Range(3, 4)),
                    R1, R2, P1, P2,
                    Q);
  // std::cout << Q << std::endl;
  cv::Mat points_3d;
  disparity.convertTo(disparity, CV_16FC1, 1/16);
  // Have bug.
  cv::reprojectImageTo3D(disparity, points_3d, Q);

  pcl::PointCloud<pcl::PointXYZRGB> output_pc;
  for (int r = 0; r < left_img.rows; ++r) {
    for (int c = 0; c < left_img.cols; ++c) {
      pcl::PointXYZRGB pt;
      const cv::Vec3b& color = left_img.at<cv::Vec3b>(r, c);
      pt.r = color(2);
      pt.g = color(1);
      pt.b = color(0);
      const cv::Vec3d pt_cv = points_3d.at<cv::Vec3b>(r, c);
      pt.x = pt_cv(0);
      pt.y = pt_cv(1);
      pt.z = pt_cv(2);
      output_pc.points.push_back(pt);
    }
  }
  output_pc.width = output_pc.size();
  output_pc.height = 1;
  pcl::io::savePCDFile("/home/cm/Projects/EpsAvlc_toys/stereo_reconstruction/pics/out.pcd", output_pc);
  return 0;
}
