#include <ros/ros.h>
#include <string>
#include <sys/statvfs.h>               // 用于获取USB挂载信息
#include "metacam_node/USBOperation.h" // 替换为你的包名

// USB挂载路径（根据实际配置修改）
const std::string USB_MOUNT_PATH = "/media/usb";

// 服务回调函数：处理USB挂载/卸载操作
bool handleUSBOperation(
    metacam_node::USBOperation::Request &req,
    metacam_node::USBOperation::Response &res)
{
    if (req.command == "usb_operation")
    {
        // 示例逻辑：检查USB挂载状态并返回信息
        struct statvfs stat;
        if (statvfs(USB_MOUNT_PATH.c_str(), &stat) == 0)
        {
            res.success = true;
            res.message = "USB is mounted. Free space: " +
                          std::to_string(stat.f_bfree * stat.f_bsize / 1024 / 1024) + " MB";
            ROS_INFO("USB operation: Mount check successful.");
        }
        else
        {
            res.success = false;
            res.message = "USB not mounted or path invalid.";
            ROS_ERROR("USB operation failed: %s", strerror(errno));
        }
        return true;
    }
    return false; // 非USB操作请求由其他回调处理
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "usb_operation_server");
    ros::NodeHandle nh;

    // 注册服务（复用Base.srv）
    ros::ServiceServer service = nh.advertiseService(
        "/usb_operation",
        handleUSBOperation);

    ROS_INFO("Service /usb_operation is ready.");
    ros::spin();

    return 0;
}