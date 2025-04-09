#include <ros/ros.h>
#include <string>
#include "metacam_node/ProjectControl.h" // 替换为你的包名

// 服务回调函数：处理项目启停
bool handleProjectControl(
    metacam_node::ProjectControl::Request &req,
    metacam_node::ProjectControl::Response &res)
{
    if (req.command == "project_control")
    {
        std::string action = req.action;
        if (action == "start")
        {
            // 在此实现项目启动逻辑（如加载配置文件、启动子进程）
            res.success = true;
            res.message = "Project started successfully.";
            ROS_INFO("Project started.");
        }
        else if (action == "stop")
        {
            // 在此实现项目停止逻辑（如释放资源、终止进程）
            res.success = true;
            res.message = "Project stopped.";
            ROS_INFO("Project stopped.");
        }
        else
        {
            res.success = false;
            res.message = "Invalid action. Use 'start' or 'stop'.";
            ROS_ERROR("Invalid action: %s", action.c_str());
        }
        return true;
    }
    return false; // 非project_control请求由其他回调处理
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "project_control_server");
    ros::NodeHandle nh;

    // 注册服务（复用Base.srv）
    ros::ServiceServer service = nh.advertiseService(
        "/project_control",
        handleProjectControl);

    ROS_INFO("Service /project_control is ready.");
    ros::spin();

    return 0;
}