#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

bool isSaved = false;

void callback(const ImageConstPtr& image1, const ImageConstPtr& image2)
{
    cv_bridge::CvImagePtr img0 = cv_bridge::toCvCopy(image1);
    imwrite("cam0_" + to_string(img0->header.stamp.toSec()) + ".png", img0->image);
    cv_bridge::CvImagePtr img1 = cv_bridge::toCvCopy(image2);
    imwrite("cam1_" + to_string(img0->header.stamp.toSec()) + ".png", img1->image);
    isSaved = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_img");
    ros::NodeHandle nh;


    message_filters::Subscriber<Image> image1_sub;
    message_filters::Subscriber<Image> image2_sub;
    image1_sub.subscribe(nh, "/fisheye_to_pinhole/cam0/right_img", 1);
    image2_sub.subscribe(nh, "/fisheye_to_pinhole/cam1/left_img", 1);

    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        if(isSaved)
            break;
    }
}