#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>

void msgCallback(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImagePtr Dest = cv_bridge::toCvCopy(msg);

    ROS_INFO("Value: %f", Dest->image.at<double>(msg->width/2, msg->height/2)); 
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "realsense_depth_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/depth/image_rect_raw", 100, msgCallback);
   
    ros::spin();

    return 0;
}