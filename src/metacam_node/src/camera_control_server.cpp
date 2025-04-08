#include <ros/ros.h>
#include "metacam_node/CameraControl.h" // 替换为你的包名

// 服务回调函数
bool handleCameraControl(
    metacam_node::CameraControl::Request &req,
    metacam_node::CameraControl::Response &res)
{
    // 解析请求参数（示例：1/10/auto/0）
    ROS_INFO("Received request: mode=%d, value=%d, option=%s, flags=%d",
             req.mode, req.value, req.option.c_str(), req.flags);

    // 在此实现相机控制逻辑（如设置相机参数）
    // 例如：调用相机SDK接口设置模式、曝光值等

    return true; // 返回操作成功状态
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_control_server");
    ros::NodeHandle nh;

    // 注册服务
    ros::ServiceServer service = nh.advertiseService(
        "/camera_control",
        handleCameraControl);

    ROS_INFO("Service /camera_control is ready.");
    ros::spin();

    return 0;
}