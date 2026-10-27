#include <ros/ros.h>
#include "circle_stereo_detector.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "circle_stereo_detector");
    ros::NodeHandle nh;

    CircleStereoDetector detector(nh);

    ros::spin();
    return 0;
}
