#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <bitset> // 用于比特位操作（调试输出）

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "driver_status_publisher");
    ros::NodeHandle nh;

    // 创建发布者，话题名 /driver_status，消息类型 std_msgs::UInt8
    ros::Publisher pub = nh.advertise<std_msgs::UInt8>("/driver_status", 10);

    // 设置发布频率（例如 1Hz）
    ros::Rate loop_rate(1);

    // 设备状态掩码定义（根据你的需求调整）
    const uint8_t SD_MASK = 0x08;    // 第4位: 0b00001000
    const uint8_t SLAM_MASK = 0x04;  // 第3位: 0b00000100
    const uint8_t CAM_MASK = 0x02;   // 第2位: 0b00000010
    const uint8_t LIDAR_MASK = 0x01; // 第1位: 0b00000001

    while (ros::ok())
    {
        std_msgs::UInt8 msg;

        // --- 模拟生成设备状态（随机示例） ---
        bool sd_enabled = (rand() % 2 == 0);    // SD 卡状态（随机）
        bool slam_enabled = (rand() % 2 == 0);  // SLAM 状态
        bool cam_enabled = (rand() % 2 == 0);   // 摄像头状态
        bool lidar_enabled = (rand() % 2 == 0); // 激光雷达状态

        // --- 组合状态到 8 位数据 ---
        msg.data = 0; // 初始化为全0
        if (sd_enabled)
            msg.data |= SD_MASK;
        if (slam_enabled)
            msg.data |= SLAM_MASK;
        if (cam_enabled)
            msg.data |= CAM_MASK;
        if (lidar_enabled)
            msg.data |= LIDAR_MASK;

        // 发布消息
        pub.publish(msg);

        // 调试输出（可选）
        ROS_INFO("Published status: 0x%02X (%s)", msg.data,
                 std::bitset<8>(msg.data).to_string().c_str());

        // 处理回调并等待
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}