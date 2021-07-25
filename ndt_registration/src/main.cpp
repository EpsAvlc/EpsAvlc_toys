/*
 * Created on Thu Jul 01 2021
 *
 * Copyright EpsAvlc
 *
 * Author: EpsAvlc
 */

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_ground_truth_point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud(ground_truth_point_cloud);
  approximate_voxel_filter.filter(*filtered_ground_truth_point_cloud);

  // pcl::visualization::CloudViewer viewer("Cloud Viewer");
  // viewer.showCloud(ground_truth_point_cloud);

  cv::Mat cam_k_left       = (cv::Mat_<double>(3, 3) << 458.654, 0, 367.215, 0,
                        457.296, 248.375, 0, 0, 1);
  cv::Mat cam_distor_left  = (cv::Mat_<double>(1, 4) << -0.28340811, 0.07395907,
                             0.00019359, 1.76187114e-05);
  cv::Mat cam_k_right      = (cv::Mat_<double>(3, 3) << 457.587, 0, 379.999, 0,
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
  stereo_extrins         = stereo_extrins.inv();
  cv::Mat R1, R2, P1, P2, Q, stereo_R, stereo_t;
  stereo_R = stereo_extrins(cv::Range(0, 3), cv::Range(0, 3));
  stereo_t = stereo_extrins(cv::Range(0, 3), cv::Range(3, 4));

  cv::Mat left_img     = cv::imread("/home/cm/Projects/EpsAvlc_toys/"
                                "stereo_reconstruction/pics/left3.png");
  cv::Mat left_img_ori = left_img.clone();
  cv::Mat right_img    = cv::imread("/home/cm/Projects/EpsAvlc_toys/"
                                 "stereo_reconstruction/pics/right3.png");
  cv::cvtColor(left_img, left_img, CV_BGR2GRAY);
  cv::cvtColor(right_img, right_img, CV_BGR2GRAY);
  StereoReconstruction stereo_reconstruction(
      cam_k_left, cam_k_right, cam_distor_left, cam_distor_right, stereo_R,
      stereo_t, left_img.size());
  cv::Mat depth = stereo_reconstruction.reconstruction(left_img, right_img);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pts_3d_pc(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int r = 0; r < left_img.rows; ++r) {
    for (int c = 0; c < left_img.cols; ++c) {
      pcl::PointXYZ   pt;
      const cv::Vec3f pt_cv = depth.at<cv::Vec3f>(r, c);
      pt.x                  = pt_cv[ 0 ];
      pt.y                  = pt_cv[ 1 ];
      pt.z                  = pt_cv[ 2 ];
      if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) &&
          std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
        if (pt.z < 0.1f) continue;

        pts_3d_pc->push_back(pt);
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_depth_pts(
      new pcl::PointCloud<pcl::PointXYZ>());
  approximate_voxel_filter.setInputCloud(pts_3d_pc);
  approximate_voxel_filter.filter(*filtered_depth_pts);

  std::cout << "input size: " << filtered_ground_truth_point_cloud->size()
            << std::endl;
  std::cout << "target size: " << filtered_depth_pts->size() << std::endl;
  // std::cout << pts_3d_pc->points.size() << std::endl;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setTransformationEpsilon(0.01);
  ndt.setStepSize(0.1);
  ndt.setResolution(1.0);
  ndt.setMaximumIterations(35);
  ndt.setInputSource(filtered_ground_truth_point_cloud);
  ndt.setInputTarget(filtered_depth_pts);

  // Eigen::AngleAxisf    init_rotation(0.6931, Eigen::Vector3f::UnitZ());
  // Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
  // Eigen::Matrix4f      init_guess = (init_translation *
  // init_rotation).matrix();

  Eigen::Matrix4f init_guess;
  cv::cv2eigen(cam_extrins_left, init_guess);
  // std::cout << init_guess << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "prepare for ndt" << std::endl;
  ndt.align(*output_cloud, init_guess);

  std::cout << "output size: " << output_cloud->size() << std::endl;
  // viewer.showCloud(pts_3d_pc);

  // // cv::imshow("depth", depth);
  // // cv::waitKey(0);

  // while (!viewer.wasStopped()) {
  // }
  return 0;
}
