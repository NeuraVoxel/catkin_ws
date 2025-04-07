#include <ros/ros.h>
#include <std_msgs/String.h>
#include <random> // 用于生成随机数

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "storage_publisher");
    ros::NodeHandle nh;

    // 创建发布者，话题名 /storage，消息类型 String
    ros::Publisher pub = nh.advertise<std_msgs::String>("/storage", 10);

    // 随机数生成器配置
    std::random_device rd;                           // 随机设备种子
    std::mt19937 gen(rd());                          // 使用 Mersenne Twister 引擎
    std::uniform_int_distribution<int> dist(0, 512); // 生成 [0, 512] 的整数

    // 固定最大值
    const int MAX_VALUE = 512;

    // 设置发布频率（例如 1Hz）
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        // 生成随机当前值
        int current = dist(gen);

        // 创建消息并填充数据
        std_msgs::String msg;
        msg.data = std::to_string(current) + "/" + std::to_string(MAX_VALUE);

        // 发布消息
        pub.publish(msg);

        // 打印日志（可选）
        ROS_INFO("Publishing: %s", msg.data.c_str());

        // 处理回调并等待
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}