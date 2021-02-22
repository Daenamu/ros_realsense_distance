#include "ros/ros.h"
#include "sensor_msgs/Image.h"


void msgCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO("Width: %i",msg->width);
    ROS_INFO("Height: %i",msg->height);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "realsense_depth_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/depth/image_rect_raw", 100, msgCallback);
   
    ros::spin();
    
    return 0;
}