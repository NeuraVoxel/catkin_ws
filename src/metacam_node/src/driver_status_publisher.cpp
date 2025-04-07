#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <random> // 用于生成随机状态码（可选）

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "driver_status_publisher");
    ros::NodeHandle nh;

    // 创建发布者，话题名 "/driver_status"，消息类型 std_msgs::UInt8
    ros::Publisher pub = nh.advertise<std_msgs::UInt8>("/driver_status", 10);

    // 设置发布频率（示例：2Hz）
    ros::Rate loop_rate(2);

    // 示例状态码：0=关闭, 1=运行, 2=错误
    // 随机生成器配置（可选）
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint8_t> dist(0, 2);

    while (ros::ok())
    {
        std_msgs::UInt8 msg;

        // 生成状态码（两种模式任选其一）
        // --- 模式1：固定状态码 ---
        msg.data = 1; // 固定为运行状态

        // --- 模式2：随机状态码 ---
        // msg.data = dist(gen);

        // 发布消息
        pub.publish(msg);

        // 输出日志
        ROS_INFO("Published driver status: %u", msg.data);

        // 处理回调并等待
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}