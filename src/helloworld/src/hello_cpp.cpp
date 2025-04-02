#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hello_cpp_node"); // 初始化节点
    ros::NodeHandle nh;
    ROS_INFO("Hello World (C++ Version)!");
    return 0;
}