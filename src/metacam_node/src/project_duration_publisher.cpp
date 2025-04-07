#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "project_duration_publisher");
    ros::NodeHandle nh;

    // 创建发布者，话题名为 "/project_duration"，消息类型为 std_msgs::Float32
    ros::Publisher pub = nh.advertise<std_msgs::Float32>("/project_duration", 10);

    // 设置发布频率（示例：1Hz）
    ros::Rate loop_rate(1);

    // 模拟项目持续时间（从0开始递增，或自定义数据）
    float duration = 0.0;

    while (ros::ok())
    {
        std_msgs::Float32 msg;

        // 生成数据（示例：时间递增 + 随机波动）
        duration += 0.1;
        msg.data = duration;

        // 发布消息
        pub.publish(msg);

        // 输出日志
        ROS_INFO("Published duration: %.2f", msg.data);

        // 处理回调并等待
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}