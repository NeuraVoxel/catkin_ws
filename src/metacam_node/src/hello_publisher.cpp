#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "hello_publisher");
    
    // 创建节点句柄
    ros::NodeHandle n;
    
    // 创建一个Publisher，发布名为/hello_topic的topic，消息类型为std_msgs::String
    ros::Publisher hello_pub = n.advertise<std_msgs::String>("/hello_topic", 10);
    
    // 设置循环的频率
    ros::Rate loop_rate(10);  // 1Hz

    int count = 0;
    
    while (ros::ok())
    {
        // 初始化std_msgs::String类型的消息
        std_msgs::String msg;
        msg.data = "helloworld" + std::to_string(count++);
        
        // 发布消息
        hello_pub.publish(msg);
        
        ROS_INFO("Published: %s", msg.data.c_str());
        
        // 按照循环频率延时
        loop_rate.sleep();
    }
    
    return 0;
} 