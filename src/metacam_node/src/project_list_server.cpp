#include <ros/ros.h>
#include <vector>
#include "metacam_node/ProjectList.h" // 替换为你的包名

// 服务回调函数：返回预设任务列表
bool handleProjectList(
    metacam_node::ProjectList::Request &req,
    metacam_node::ProjectList::Response &res)
{
    if (req.command == "project_list")
    {
        // 示例任务列表（可扩展为动态获取）
        std::vector<std::string> task_list = {"task1", "task2", "task3"};
        res.tasks = task_list;
        res.success = true;
        res.message = "Project list retrieved successfully.";
        ROS_INFO("Project list request handled. Tasks: %zu", task_list.size());
        return true;
    }
    return false; // 非project_list请求由其他回调处理
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "project_list_server");
    ros::NodeHandle nh;

    // 注册服务（复用Base.srv）
    ros::ServiceServer service = nh.advertiseService(
        "/project_list",
        handleProjectList);

    ROS_INFO("Service /project_list is ready.");
    ros::spin();

    return 0;
}