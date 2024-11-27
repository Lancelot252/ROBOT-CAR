# 数据中心巡检小车demo

## 本项目将开发一款数据中心巡检小车，包含手动和自动两部分

## CarProgram
小车的相关程序，主要为遥控模式下的小车程序（目前）

## ros_connect
小车ROS系统与手机端通信程序，其作用相当于一个中介，将手机所发出的JSON数据转换为ros话题发布，使车辆控制程序可以接收到相应数据。

### udp_server.cpp
`udp_server.cpp` 文件包含以下主要功能：

1. **获取CPU序列号**:
   - 函数: `getCpuSerialNumber()`
   - 描述: 从 `/proc/cpuinfo` 文件中读取CPU序列号，并进行格式化处理。读取到的序列号会被反转并转换为大写字母，最终返回16位的序列号字符串。
   - 参数: 无

2. **UDP设备发现**:
   - 函数: `udpDeviceDiscovery(int port, const std::string& robot_type, const std::string& serial_number)`
   - 描述: 创建一个UDP服务器，监听指定端口，等待设备发现请求。当接收到特定消息（如 "LOBOT_NET_DISCOVER"）时，服务器会响应包含机器人类型和序列号的消息。
   - 参数:
     - `port`: 监听的UDP端口号。
     - `robot_type`: 机器人的类型（例如 "SPIDER"）。
     - `serial_number`: 机器人的序列号。

3. **处理JSON-RPC请求**:
   - 函数: `handleJsonRpcRequests(int tcp_port, ros::Publisher& publisher)`
   - 描述: 创建一个TCP服务器，监听指定端口，处理来自客户端的JSON-RPC请求。解析请求中的JSON数据，根据请求的方法名发布相应的ROS话题（水平移动或旋转信息，使用字符串类型进行传输），并返回处理结果。
   - 参数:
     - `tcp_port`: 监听的TCP端口号。
     - `publisher`: ROS发布者，用于发布处理后的消息。

4. **主函数**:
   - 函数: `main(int argc, char** argv)`
   - 描述: 初始化ROS节点，创建ROS发布者，获取CPU序列号，启动UDP设备发现线程，并在主线程中处理JSON-RPC请求。主函数首先初始化ROS节点和发布者，然后获取CPU序列号，启动一个独立的线程来处理UDP设备发现，最后在主线程中处理JSON-RPC请求，直到程序结束。
   - 参数:
     - `argc`: 命令行参数的数量。
     - `argv`: 命令行参数的数组。

#### 使用的函数库
- **ROS**: 用于机器人操作系统的通信和日志记录。
- **std_msgs**: 用于ROS标准消息类型。
- **iostream**: 用于标准输入输出流操作。
- **fstream**: 用于文件输入输出流操作。
- **string**: 用于字符串操作。
- **thread**: 用于多线程操作。
- **boost::asio**: 用于网络编程（TCP和UDP）。
- **jsoncpp**: 用于JSON数据解析和生成。
- **arpa/inet.h**: 用于网络地址操作（如socket编程）。

### CMakeLists.txt
`CMakeLists.txt` 文件包含以下主要配置：

1. **项目初始化**:
   - 设置CMake的最低版本要求，并定义项目名称。
   - 使用C++11标准进行编译，并添加jsoncpp库。

2. **查找catkin包和系统依赖**:
   - 查找ROS所需的catkin包（如roscpp、rospy、std_msgs）。
   - 查找jsoncpp库。

3. **包含目录**:
   - 指定头文件的搜索路径，包括catkin和jsoncpp的头文件目录。

4. **生成可执行文件**:
   - 定义可执行文件`udp_server_node`和`udp_server`，并链接所需的库（如catkin和jsoncpp）。

