#include <ros/ros.h>
#include <fstream>
#include <vector>
#include "metacam_node/ProjectImage.h" // 替换为你的包名

// 读取本地JPEG文件为二进制流
bool loadImageData(const std::string &path, std::vector<uint8_t> &data)
{
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open image file: %s", path.c_str());
        return false;
    }
    file.seekg(0, std::ios::end);
    data.resize(file.tellg());
    file.seekg(0, std::ios::beg);
    file.read(reinterpret_cast<char *>(data.data()), data.size());
    return true;
}

// 服务回调函数
bool handleProjectImage(
    metacam_node::ProjectImage::Request &req,
    metacam_node::ProjectImage::Response &res)
{
    // 根据任务名映射到图片路径（示例逻辑）
    std::string image_path;
    if (req.task_name == "task1")
    {
        image_path = "/mnt/ubuntu/NeuraVoxel/MetaCam/catkin_ws/src/metacam_node/images/1744208620331.png";
    }
    else
    {
        res.success = false;
        res.message = "Invalid task name.";
        ROS_WARN("Invalid task name: %s", req.task_name.c_str());
        return true;
    }

    // 加载图片数据
    std::vector<uint8_t> image_data;
    if (!loadImageData(image_path, image_data))
    {
        res.success = false;
        res.message = "Image file read failed.";
        return true;
    }

    // 填充响应
    res.image_data = image_data;
    res.success = true;
    res.message = "Image data sent successfully.";
    ROS_INFO("Sent image data for task: %s (Size: %zu bytes)",
             req.task_name.c_str(), image_data.size());
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "project_image_server");
    ros::NodeHandle nh;

    // 注册服务
    ros::ServiceServer service = nh.advertiseService(
        "/project_image",
        handleProjectImage);

    ROS_INFO("Service /project_image is ready.");
    ros::spin();

    return 0;
}