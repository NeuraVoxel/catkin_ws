# ROS Publisher Examples

这个包包含了两个ROS发布者节点示例：

## 1. Hello Publisher Node

这是一个简单的ROS发布者节点示例，用于周期性地发布"helloworld"字符串消息。

### 功能
- 创建一个名为`hello_publisher`的ROS节点
- 以1Hz的频率在`/hello_topic`话题上发布std_msgs/String类型的消息
- 消息内容为"helloworld"

### 运行方法
1. 首先启动ROS核心：
```bash
roscore
```

2. 在新的终端中运行节点：
```bash
rosrun hello_pub hello_publisher
```

### 查看发布的消息
可以使用以下命令查看发布的消息：
```bash
rostopic echo /hello_topic
```

## 2. PointCloud Publisher Node

这是一个点云发布者节点示例，用于周期性地发布包含10000个点的随机点云数据。

### 功能
- 创建一个名为`pointcloud_publisher`的ROS节点
- 以1Hz的频率在`/point_cloud`话题上发布sensor_msgs/PointCloud2类型的消息
- 每个点云包含10000个随机生成的点
- 每个点包含XYZ坐标和RGB颜色信息
- 点的坐标范围在[-5.0, 5.0]之间
- 点的颜色为随机RGB值

### 实现细节
- 使用C++11标准随机数生成器生成点的坐标
- 直接操作PointCloud2消息的底层数据结构
- 点云数据采用紧凑的内存布局，每个点占用16字节
- 支持XYZ+RGB格式的点云数据

### 运行方法
1. 首先启动ROS核心：
```bash
roscore
```

2. 在新的终端中运行节点：
```bash
rosrun hello_pub pointcloud_publisher
```

### 查看点云数据
可以使用以下方法之一查看点云数据：

1. 使用RViz：
```bash
rviz
```
然后添加PointCloud2显示，设置话题为`/point_cloud`

2. 使用命令行查看点云消息信息：
```bash
rostopic echo /point_cloud
```

### 编译方法
在工作空间根目录下执行：
```bash
catkin_make
source devel/setup.bash
``` 