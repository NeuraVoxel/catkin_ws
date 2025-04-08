#include <ros/ros.h>
#include <string>
#include "metacam_node/IPConfig.h" // 替换为你的包名

// 服务回调函数：解析IP配置并返回网关
bool handleIPConfig(
    metacam_node::IPConfig::Request &req,
    metacam_node::IPConfig::Response &res)
{
    std::string ip_subnet = req.ip_subnet;
    // 解析IP和子网（示例：从 "192.168.0.33/24" 中提取IP和掩码）
    size_t slash_pos = ip_subnet.find('/');
    if (slash_pos == std::string::npos)
    {
        ROS_ERROR("Invalid IP format: %s", ip_subnet.c_str());
        return false;
    }
    std::string ip = ip_subnet.substr(0, slash_pos);
    std::string subnet = ip_subnet.substr(slash_pos + 1);

    // 假设根据IP和子网生成网关（实际需根据网络逻辑实现）
    res.gateway = "192.168.0.1"; // 示例固定网关
    ROS_INFO("IP: %s/%s → Gateway: %s", ip.c_str(), subnet.c_str(), res.gateway.c_str());
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ip_config_server");
    ros::NodeHandle nh;

    // 注册服务
    ros::ServiceServer service = nh.advertiseService(
        "/ip_config",
        handleIPConfig);

    ROS_INFO("Service /ip_config is ready.");
    ros::spin();

    return 0;
}