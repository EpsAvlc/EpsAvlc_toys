/*
 * Created on Thu Jul 01 2021
 *
 * Copyright EpsAvlc
 *
 * Author: EpsAvlc
 */

#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

class StereoReconstruction {
 public:
  explicit StereoReconstruction(cv::Mat cam_k_left, cv::Mat cam_k_right,
                                cv::Mat cam_distor_left,
                                cv::Mat cam_distor_right, cv::Mat R, cv::Mat t,
                                cv::Size image_size)
      : cam_k_left_(cam_k_left)
      , cam_k_right_(cam_k_right)
      , cam_distor_left_(cam_distor_left)
      , cam_distor_right_(cam_distor_right) {
    cv::stereoRectify(cam_k_left, cam_distor_left, cam_k_right,
                      cam_distor_right, image_size, R, t, R1_, R2_, P1_, P2_,
                      Q_, 1024, 0);
    bm_ = cv::StereoBM::create();
    bm_->setPreFilterType(CV_STEREO_BM_NORMALIZED_RESPONSE);
    bm_->setPreFilterSize(41);
    bm_->setPreFilterCap(28);
    bm_->setBlockSize(21);
    bm_->setMinDisparity(-30);
    bm_->setNumDisparities(64);
    bm_->setTextureThreshold(0);
    bm_->setUniquenessRatio(10);
    bm_->setSpeckleWindowSize(0);
    bm_->setSpeckleRange(0);
  }

  cv::Mat reconstruction(const cv::Mat &left_img, const cv::Mat &right_img) {
    cv::Mat map11, map12, map21, map22;
    cv::initUndistortRectifyMap(cam_k_left_, cam_distor_left_, R1_, P1_,
                                left_img.size(), CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(cam_k_right_, cam_distor_right_, R2_, P2_,
                                right_img.size(), CV_16SC2, map21, map22);
    cv::Mat left_img_remap, right_img_remap;
    cv::remap(left_img, left_img_remap, map11, map12, cv::INTER_LINEAR);
    cv::remap(right_img, right_img_remap, map21, map22, cv::INTER_LINEAR);
    cv::Mat disparity;
    bm_->compute(left_img_remap, right_img_remap, disparity);
    cv::Mat points_3d;
    disparity /= 16;
    cv::reprojectImageTo3D(disparity, points_3d, Q_, false);
    return std::move(points_3d);
  }

 private:
  cv::Mat cam_k_left_, cam_k_right_, cam_distor_left_, cam_distor_right_;
  cv::Mat R1_, R2_, P1_, P2_, Q_;
  cv::Ptr<cv::StereoBM> bm_;
};

int main(int argc, char const *argv[]) {
  /* code */
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_truth_point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPLYFile(
      "/home/cm/Projects/EpsAvlc_toys/ndt_registration/map/map.ply",
      *ground_truth_point_cloud);

  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(ground_truth_point_cloud);

  while (!viewer.wasStopped()) {
  }
  return 0;
}
