/*
 * Created on Mon Sep 23 2019
 * 
 * Author: EpsAvlc
 */

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    Mat img_left = imread("./imgs/office1.jpg");
    Mat img_right = imread("./imgs/office2.jpg");
    resize(img_left, img_left, Size(), 0.4, 0.4);
    resize(img_right, img_right, Size(), 0.4, 0.4);

    vector<KeyPoint> keypoints_left, keypoints_right;
    Mat descriptor_left, descriptor_right;    
    Ptr<ORB> orb = ORB::create();
    orb->detect(img_left, keypoints_left);
    orb->detect(img_right, keypoints_right);
    orb->compute(img_left, keypoints_left, descriptor_left);
    orb->compute(img_right, keypoints_right, descriptor_right);
    vector<DMatch> matches;
    BFMatcher matcher(NORM_HAMMING);
    matcher.match(descriptor_left, descriptor_right, matches);

    double min_dist = 10000, max_dist = 0;
    for(int i = 0; i < descriptor_left.rows; i++)
    {
        double dist = matches[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }

    vector<DMatch> good_matches;
    vector<Point2f> points_left, points_right; 

    for(int i = 0; i < descriptor_left.rows; i++)
    {
        if(matches[i].distance <= max(2*min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
            points_left.push_back(keypoints_left[matches[i].queryIdx].pt);
            points_right.push_back(keypoints_right[matches[i].trainIdx].pt);
        }
    }

    // Mat img_match;
    // drawMatches(img_left, keypoints_left, img_right, keypoints_right, good_matches, img_match);
    // imshow("img_match", img_match);

    Mat H;
    H = findHomography(points_left, points_right);
    // getAffineTransform(img_left, img_right)
    // cout << "Homography is: " << endl << H << endl;
    Mat img_stitching;
    warpPerspective(img_left, img_stitching, H, Size(img_left.cols *2, img_left.rows));
    // imshow("img_left", img_left);
    imshow("before", img_stitching);
    img_right.copyTo(img_stitching(Rect(0, 0, img_right.cols, img_right.rows)));
    imshow("img_stitching", img_stitching);
    imwrite("./imgs/stitching.jpg", img_stitching);
    waitKey(0);    
}