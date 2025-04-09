#include <ros/ros.h>
#include <fstream>
#include <vector>
#include "metacam_node/ProjectCloud.h" // 替换为你的包名
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// 读取本地PCD文件为二进制流
bool loadPCDData(const std::string &path, std::vector<uint8_t> &data)
{
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open PCD file: %s", path.c_str());
        return false;
    }
    file.seekg(0, std::ios::end);
    data.resize(file.tellg());
    file.seekg(0, std::ios::beg);
    file.read(reinterpret_cast<char *>(data.data()), data.size());
    return true;
}

// 服务回调函数
bool handleProjectCloud(
    metacam_node::ProjectCloud::Request &req,
    metacam_node::ProjectCloud::Response &res)
{
    // 任务名映射到PCD路径（示例逻辑）
    std::string pcd_path;
    if (req.task_name == "task1")
    {
        pcd_path = "/path/to/task1_cloud.pcd";
    }
    else
    {
        res.success = false;
        res.message = "Invalid task name.";
        ROS_WARN("Invalid task name: %s", req.task_name.c_str());
        return true;
    }

    // 加载PCD二进制数据
    std::vector<uint8_t> pcd_data;
    if (!loadPCDData(pcd_path, pcd_data))
    {
        res.success = false;
        res.message = "PCD file read failed.";
        return true;
    }

    // 填充响应
    res.pcd_data = pcd_data;
    res.success = true;
    res.message = "PCD data sent successfully.";
    ROS_INFO("Sent PCD data for task: %s (Size: %zu bytes)",
             req.task_name.c_str(), pcd_data.size());
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "project_cloud_server");
    ros::NodeHandle nh;

    // 注册服务
    ros::ServiceServer service = nh.advertiseService(
        "/project_cloud",
        handleProjectCloud);

    ROS_INFO("Service /project_cloud is ready.");
    ros::spin();

    return 0;
}