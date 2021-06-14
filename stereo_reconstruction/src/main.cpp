/*
 * Created on Mon Jun 14 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#include "stereo_reconstruction/stereo_reconstruction.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char const *argv[]) {
  cv::Mat left_img = cv::imread("/home/cm/Projects/EpsAvlc_toys/"
    "stereo_reconstruction/pics/left2.png");
  cv::Mat right_img = cv::imread("/home/cm/Projects/EpsAvlc_toys/"
    "stereo_reconstruction/pics/right2.png");
  cv::imshow("left2", left_img);
  cv::imshow("right2", right_img);
  cv::cvtColor(left_img, left_img, CV_BGR2GRAY);
  cv::cvtColor(right_img, right_img, CV_BGR2GRAY);
  cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16, 9);

  bm->setPreFilterType(CV_STEREO_BM_NORMALIZED_RESPONSE);
  bm->setPreFilterSize(9);
  bm->setPreFilterCap(31);
  bm->setBlockSize(21);
  bm->setMinDisparity(-16);
  bm->setNumDisparities(64);
  bm->setTextureThreshold(10);
  bm->setUniquenessRatio(5);
  bm->setSpeckleWindowSize(100);
  bm->setSpeckleRange(32);
  cv::Mat disp;
  bm->compute(left_img, right_img, disp);
  disp.convertTo(disp, CV_32F, 1.0/16);
  cv::imshow("disp", disp);
  cv::waitKey(0);
  return 0;
}
