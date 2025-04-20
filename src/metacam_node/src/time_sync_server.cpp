#include <ros/ros.h>
#include <metacam_node/TimeSync.h>
#include <ctime>
#include <sstream>

bool handleTimeRequest(
    metacam_node::TimeSync::Request &req,
    metacam_node::TimeSync::Response &res) 
{
    try {
        // 解析输入时间戳
        std::tm client_time = {};
        std::istringstream ss(req.timestamp);
        ss >> std::get_time(&client_time, "%Y-%m-%d %H:%M:%S");
        
        if (ss.fail()) {
            throw std::runtime_error("Invalid timestamp format");
        }

        // 获取当前系统时间
        std::time_t sys_time = std::time(nullptr);
        std::tm* sys_tm = std::localtime(&sys_time);

        // 计算时间差（秒）
        double offset = std::difftime(std::mktime(sys_tm), std::mktime(&client_time));

        // 构造响应
        std::ostringstream oss;
        oss << "Time offset: " << offset << " seconds";
        res.result = oss.str();

        ROS_INFO("Processed timestamp: %s", req.timestamp.c_str());
        return true;
    } 
    catch (const std::exception &e) {
        res.result = "Error: " + std::string(e.what());
        return false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "time_sync_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService(
        "set_system_time", 
        handleTimeRequest
    );

    ROS_INFO("Time Sync Service Ready");
    ros::spin();
    return 0;
}