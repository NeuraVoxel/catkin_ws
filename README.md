# catkin_ws
catkin_ws/
├── src/
│   ├── package1/        # 功能包1
│   │   ├── src/
│   │   │   ├── node1.cpp  # 节点1
│   │   │   └── node2.cpp  # 节点2
│   │   ├── CMakeLists.txt  # 编译配置，生成两个可执行文件
│   │   └── package.xml
│   ├── package2/        # 功能包2
│   │   └── scripts/
│   │       ├── node3.py    # 节点3（Python）
│   │       └── node4.py    # 节点4（Python）
└── ...


以下是 **ROS1 (Noetic/Melodic)** 下创建 Hello World 示例的详细步骤及代码：

---

### 一、创建工作空间
```bash
# 创建并初始化工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash  # 激活环境变量
```

---

### 二、创建功能包
```bash
cd src
catkin_create_pkg helloworld roscpp rospy std_msgs  # 包含C++、Python和消息依赖
```

---

### 三、编写示例代码
#### **C++ 版本**
1. **代码文件**：在 `~/catkin_ws/src/helloworld/src` 下新建 `hello_cpp.cpp`，内容如下：
    ```cpp
    #include "ros/ros.h"
    int main(int argc, char **argv) {
        ros::init(argc, argv, "hello_cpp_node");  // 初始化节点
        ros::NodeHandle nh;
        ROS_INFO("Hello World (C++ Version)!");
        return 0;
    }
    ```

2. **编译配置**：修改 `CMakeLists.txt`，在 `Build` 部分添加：
    ```cmake
    add_executable(hello_cpp src/hello_cpp.cpp)
    target_link_libraries(hello_cpp ${catkin_LIBRARIES})
    ```

#### **Python 版本**
1. **代码文件**：在 `~/catkin_ws/src/helloworld/scripts` 下新建 `hello_py.py`，内容如下：
    ```python
    #!/usr/bin/env python
    import rospy
    if __name__ == "__main__":
        rospy.init_node("hello_py_node")  # 初始化节点
        rospy.loginfo("Hello World (Python Version)!")
    ```

2. **设置权限**：
    ```bash
    chmod +x ~/catkin_ws/src/helloworld/scripts/hello_py.py
    ```

---

### 四、编译与运行
```bash
# 编译工作空间
cd ~/catkin_ws/
catkin_make

# 启动 ROS 核心
roscore

# 新终端运行 C++ 节点
source devel/setup.bash  # 确保环境变量生效
rosrun helloworld hello_cpp

# 新终端运行 Python 节点
rosrun helloworld hello_py.py
```

---

### 关键注意事项
1. **环境变量**：每次新开终端需执行 `source devel/setup.bash`，或将其添加到 `~/.bashrc` 永久生效。
2. **文件路径**：
   • C++ 源码需放在 `src` 目录下；
   • Python 源码需放在 `scripts` 目录，并确保可执行权限。
3. **依赖检查**：若报错缺失依赖，可通过 `rosdep install` 安装。

---

### 常见问题
• **节点未运行**：检查 `roscore` 是否已启动，且节点名称无重复。
• **Python脚本报错**：确认 `#!/usr/bin/env python` 的 Python 版本与系统一致（如 `python3` 需修改为 `#!/usr/bin/env python3`）。

通过上述步骤，可快速验证 ROS1 环境及基础功能。如需更复杂的逻辑（如发布消息），可参考 ROS 官方文档扩展代码


# Service
## get_version_server

```
source devel/setup.bash 
rosrun metacam_node get_version_server
rosservice call /get_version
```

## camera_control_server
```
source devel/setup.bash 
rosrun metacam_node camera_control_server

rosservice call /camera_control "mode: 1
value: 10
option: 'auto'
flags: 0"
```

## ip_config_server
```
source devel/setup.bash 
rosrun metacam_node ip_config_server

rosservice call /ip_config "ip_subnet: '192.168.0.33/24'"
```


## project_control_server
```
source devel/setup.bash 
rosrun metacam_node project_control_server
# 启动项目
rosservice call /project_control "command: 'project_control'
action: 'start'"

# 停止项目
rosservice call /project_control "command: 'project_control'
action: 'stop'"
```

## usb_operation_server
```
source devel/setup.bash 
rosrun metacam_node usb_operation_server
# 触发USB状态检查
rosservice call /usb_operation "command: 'usb_operation'"
```

## project_list_server
```
source devel/setup.bash 
rosrun metacam_node project_list_server
# 触发任务列表查询
rosservice call /project_list "command: 'project_list'"
```


## project_image_server
```
source devel/setup.bash 
rosrun metacam_node project_image_server
# 请求task1的图片数据
rosservice call /project_image "task_name: 'task1'"
```


## project_cloud_server
```
source devel/setup.bash 
rosrun metacam_node project_cloud_server
# 请求task1的点云数据
rosservice call /project_cloud "task_name: 'task1'"
```