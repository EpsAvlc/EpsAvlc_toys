/*
 * Created on Mon Jun 14 2021
 *
 * Author: EpsAvlc
 */

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "stereo_reconstruction/stereo_reconstruction.h"

class TicToc {
 public:
  void tic() { last_time_ = std::chrono::system_clock::now(); }

  double toc() {
    std::chrono::milliseconds dt =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now() - last_time_);
    return dt.count();
  }

 private:
  std::chrono::system_clock::time_point last_time_;
};

int main(int argc, char const *argv[]) {
  cv::Mat left_img = cv::imread("/home/cm/Projects/EpsAvlc_toys/"
                                "stereo_reconstruction/pics/left3.png");
  cv::Mat left_img_ori = left_img.clone();
  cv::Mat right_img = cv::imread("/home/cm/Projects/EpsAvlc_toys/"
                                 "stereo_reconstruction/pics/right3.png");
  cv::cvtColor(left_img, left_img, CV_BGR2GRAY);
  cv::cvtColor(right_img, right_img, CV_BGR2GRAY);
  // cv::imshow("left2", left_img);
  // cv::imshow("right2", right_img);


  // std::cout << "bm: " << tt.toc() << std::endl;
  // disparity_bm.convertTo(disp, CV_8U, 255 / (numberOfDisparities * 16.));
  // cv::imshow("disp", disp);

  /************calibrate rectify*****************/
  cv::Mat cam_k_left = (cv::Mat_<double>(3, 3) << 458.654, 0, 367.215, 0,
                        457.296, 248.375, 0, 0, 1);
  cv::Mat cam_distor_left = (cv::Mat_<double>(1, 4) << -0.28340811, 0.07395907,
                             0.00019359, 1.76187114e-05);
  cv::Mat cam_k_right = (cv::Mat_<double>(3, 3) << 457.587, 0, 379.999, 0,
                         456.134, 255.238, 0, 0, 1);
  cv::Mat cam_distor_right = (cv::Mat_<double>(1, 4) << -0.28368365, 0.07451284,
                              -0.00010473, -3.55590700e-05);
  cv::Mat cam_extrins_left =
      (cv::Mat_<double>(4, 4) << 0.0148655429818, -0.999880929698,
       0.00414029679422, -0.0216401454975, 0.999557249008, 0.0149672133247,
       0.025715529948, -0.064676986768, -0.0257744366974, 0.00375618835797,
       0.999660727178, 0.00981073058949, 0.0, 0.0, 0.0, 1.0);
  cv::Mat cam_extrins_right =
      (cv::Mat_<double>(4, 4) << 0.0125552670891, -0.999755099723,
       0.0182237714554, -0.0198435579556, 0.999598781151, 0.0130119051815,
       0.0251588363115, 0.0453689425024, -0.0253898008918, 0.0179005838253,
       0.999517347078, 0.00786212447038, 0.0, 0.0, 0.0, 1.0);
  cv::Mat stereo_extrins = cam_extrins_left.inv() * cam_extrins_right;
  stereo_extrins = stereo_extrins.inv();
  std::cout << stereo_extrins << std::endl;
  cv::Mat R1, R2, P1, P2, Q, stereo_R, stereo_t;
  stereo_R = stereo_extrins(cv::Range(0, 3), cv::Range(0, 3));
  stereo_t = stereo_extrins(cv::Range(0, 3), cv::Range(3, 4));
  cv::stereoRectify(
      cam_k_left, cam_distor_left, cam_k_right, cam_distor_right,
      left_img.size(), stereo_R, stereo_t, R1, R2, P1, P2, Q, 1024, 0);

  cv::Mat map11, map12, map21, map22;
  cv::initUndistortRectifyMap(cam_k_left, cam_distor_left, R1, P1,
                              left_img.size(), CV_16SC2, map11, map12);
  cv::initUndistortRectifyMap(cam_k_right, cam_distor_right, R2, P2,
                              right_img.size(), CV_16SC2, map21, map22);
  cv::Mat left_img_remap, right_img_remap;
  cv::remap(left_img, left_img_remap, map11, map12, cv::INTER_LINEAR);
  cv::remap(right_img, right_img_remap, map21, map22, cv::INTER_LINEAR);
  cv::imshow("left", left_img_remap);
  cv::imshow("right", right_img_remap);

  cv::imwrite("/home/cm/left.png", left_img_remap);
  cv::imwrite("/home/cm/right.png", right_img_remap);

  /******************BM Match******************/
  cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create();
  bm->setPreFilterType(CV_STEREO_BM_NORMALIZED_RESPONSE);
  int numberOfDisparities = 64;
  bm->setPreFilterSize(41);
  bm->setPreFilterCap(28);
  bm->setBlockSize(21);
  bm->setMinDisparity(-30);
  bm->setNumDisparities(numberOfDisparities);
  bm->setTextureThreshold(0);
  bm->setUniquenessRatio(10);
  bm->setSpeckleWindowSize(0);
  bm->setSpeckleRange(0);
  cv::Mat disp, disparity_bm;
  TicToc tt;
  tt.tic();
  bm->compute(left_img_remap, right_img_remap, disparity_bm);
  disparity_bm.convertTo(disp, CV_8U, 255 / (numberOfDisparities * 16.));
  cv::imshow("disp", disp);
  cv::Mat points_3d;

  // cv::Ptr<cv::StereoSGBM> sgbm =
  //     cv::StereoSGBM::create(0,      // int minDisparity
  //                            96,     // int numDisparities
  //                            5,     // int SADWindowSize
  //                            600,    // int P1 = 0
  //                            2400,   // int P2 = 0
  //                            20,     // int disp12MaxDiff = 0
  //                            16,     // int preFilterCap = 0
  //                            15,      // int uniquenessRatio = 0
  //                            100,     // int speckleWindowSize = 0
  //                            2,     // int speckleRange = 0
  //                            true);  // bool fullDP = false
  // cv::Mat disparity_sgbm;
  // sgbm->compute(left_img_remap, right_img_remap, disparity_sgbm);



  // disparity_sgbm.convertTo(disp, CV_8U, 255 / (numberOfDisparities * 16.));
  // cv::imshow("disp2", disp);

  disparity_bm = disparity_bm / 16;
  // 这里有个地方需要注意，如果获得视差图像是CV_16S类型的，这样的视差图的每个像素值由一个16bit表示，其中低位的4位存储的是视差值得小数部分，所以真实视差值应该是该值除以16。在进行映射后应该乘以16，以获得毫米级真实位置。
  cv::reprojectImageTo3D(disparity_bm, points_3d, Q, false);
  pcl::PointCloud<pcl::PointXYZRGB> pts_3d_pc, disparity_pc;
  cv::Mat disp_depth = cv::Mat(left_img.size(), CV_8UC1, cv::Scalar(0));
  for (int r = 0; r < left_img.rows; ++r) {
    for (int c = 0; c < left_img.cols; ++c) {
      pcl::PointXYZRGB pt;
      const uchar &color = left_img_remap.at<uchar>(r, c);
      pt.r = color;
      pt.g = color;
      pt.b = color;
      const cv::Vec3f pt_cv = points_3d.at<cv::Vec3f>(r, c);
      pt.x = pt_cv[0];
      pt.y = pt_cv[1];
      pt.z = pt_cv[2];
      if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) &&
        std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
        if (pt.z < 0.1f)
          continue;

        pts_3d_pc.push_back(pt);
        pt.x = left_img_remap.cols - c;
        pt.y = left_img_remap.rows - r;
        pt.z = disparity_bm.at<int16_t>(r, c);

          disparity_pc.push_back(pt);
      }
    }
  }
  // // cv::imshow("disp_depth", disp_depth);
  // std::cout << pts_3d_pc.size() << std::endl;
  // cv::waitKey(0);
  pcl::io::savePLYFile(
      "/home/cm/Projects/EpsAvlc_toys/stereo_reconstruction/pics/3d_pts.ply",
      pts_3d_pc);
  pcl::io::savePLYFile(
      "/home/cm/Projects/EpsAvlc_toys/stereo_reconstruction/pics/disparity.ply",
      disparity_pc);
  pcl::io::savePCDFile(
      "/home/cm/Projects/EpsAvlc_toys/stereo_reconstruction/pics/out.pcd",
      pts_3d_pc);
  cv::waitKey(0);
  return 0;
}
