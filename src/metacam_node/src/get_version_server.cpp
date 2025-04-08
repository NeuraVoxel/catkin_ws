#include <ros/ros.h>
#include "metacam_node/Base.h" // 替换为你的包名

// 服务回调函数
bool handleVersionRequest(
    metacam_node::Base::Request &req,
    metacam_node::Base::Response &res)
{
    res.version = "1.0.1"; // 设置响应版本号
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "version_server");
    ros::NodeHandle nh;

    // 创建服务对象
    ros::ServiceServer service = nh.advertiseService(
        "/get_version",
        handleVersionRequest);

    ROS_INFO("Service /get_version is ready.");
    ros::spin();

    return 0;
}