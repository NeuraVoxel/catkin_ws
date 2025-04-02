#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <random>

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "pointcloud_publisher");
    ros::NodeHandle nh;

    // 创建发布者
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1);

    // 设置发布频率
    ros::Rate loop_rate(10); // 1Hz

    // 创建随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-5.0, 5.0);

    // 创建点云消息
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "base_link";
    cloud_msg.height = 1;
    cloud_msg.width = 10000;  // 10000个点

    // 设置点云字段
    cloud_msg.fields.resize(4);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;
    
    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;
    
    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;

    cloud_msg.fields[3].name = "rgb";
    cloud_msg.fields[3].offset = 12;
    cloud_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[3].count = 1;

    cloud_msg.point_step = 16;  // 每个点占用的字节数
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.is_dense = true;

    // 分配内存
    cloud_msg.data.resize(cloud_msg.row_step);

    while (ros::ok())
    {
        cloud_msg.header.stamp = ros::Time::now();

        // 生成随机点云数据
        for (size_t i = 0; i < cloud_msg.width; ++i)
        {
            float x = dis(gen);
            float y = dis(gen);
            float z = dis(gen);
            uint8_t r = rand() % 255;
            uint8_t g = rand() % 255;
            uint8_t b = rand() % 255;
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

            memcpy(&cloud_msg.data[i * cloud_msg.point_step + 0], &x, sizeof(float));
            memcpy(&cloud_msg.data[i * cloud_msg.point_step + 4], &y, sizeof(float));
            memcpy(&cloud_msg.data[i * cloud_msg.point_step + 8], &z, sizeof(float));
            memcpy(&cloud_msg.data[i * cloud_msg.point_step + 12], &rgb, sizeof(float));
        }

        // 发布点云消息
        cloud_pub.publish(cloud_msg);
        ROS_INFO("Published PointCloud2 with %zu points", cloud_msg.width);

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
