#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

int main(int argc, char **argv)
{
    // 初始化ROS节点，节点名为"battery_publisher"
    ros::init(argc, argv, "battery_publisher");
    ros::NodeHandle nh;

    // 创建发布者，发布到"/battery"话题，消息类型为sensor_msgs::BatteryState，队列大小设为10
    ros::Publisher battery_pub = nh.advertise<sensor_msgs::BatteryState>("/battery", 10);

    // 设置循环频率（例如10Hz）
    ros::Rate loop_rate(10);

    // 持续运行直到ROS关闭
    while (ros::ok())
    {
        sensor_msgs::BatteryState msg;

        // 设置消息头的时间戳
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "battery"; // 可选的frame标识

        // 模拟电池数据（根据实际情况修改）
        msg.voltage = 12.0;        // 电压（伏特）
        msg.current = -0.5;        // 电流（负值表示放电）
        msg.charge = 5.0;          // 当前电量（Ah）
        msg.capacity = 5.0;        // 最大容量（Ah）
        msg.design_capacity = 5.0; // 设计容量（Ah）
        msg.percentage = 0.8;      // 剩余电量百分比（0.0~1.0）

        // 电池状态枚举（根据传感器实际情况设置）
        msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

        // 是否已充满电？需要根据实际情况设置
        msg.present = (msg.percentage >= 1.0) ? false : true;

        // 发布消息
        battery_pub.publish(msg);

        // 处理ROS事件并等待循环频率
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}