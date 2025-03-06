# 数据中心巡检小车demo

## 目录
1. [项目简介](#本项目将开发一款数据中心巡检小车，包含手动和自动两部分)
2. [motorControl](#motorcontrol)
   - [RobotControl.h](#robotcontrolh)
   - [RobotControl.cpp](#robotcontrolcpp)
   - [MotorControl.h](#motorcontrolh)
   - [MotorControl.cpp](#motorcontrolcpp)
   - [MotorCan.h](#motorcanh)
   - [MotorCan.cpp](#motorcancpp)
   - [CarControlNode.cpp](#carcontrolnodecpp)
   - [timer.h](#timerh)
   - [RobotCarMode.h](#robotcarmodeh)
3. [ros_connect](#ros_connect)
   - [udp_server.cpp](#udp_servercpp)
   - [CMakeLists.txt](#cmakelists.txt)

## 本项目将开发一款数据中心巡检小车，包含手动和自动两部分

## motorControl
小车遥控程序，负责接收遥控信号并控制电机。

### RobotControl.h
`RobotControl.h` 文件包含以下主要内容：

1. **成员变量**:
   - `int32_t angle`: 电机角度。
   - `int16_t ntorque`: 电机扭矩。
   - `uint16_t motor_speed`: 电机速度。
   - `uint16_t motor_max_speed`: 电机最大速度。
   - `MotorCan usbCan`: MotorCan 对象，用于与电机通信。

2. **构造函数**:
   - `RobotControl()`: 默认构造函数，初始化成员变量。
   - `RobotControl(MotorCan *usbCan)`: 带参数的构造函数，使用 `MotorCan` 对象初始化 `usbCan`。
   - `RobotControl(int canID)`: 带参数的构造函数，使用 CAN ID 初始化 `usbCan`。

3. **析构函数**:
   - `~RobotControl()`: 析构函数。

4. **主要方法**:
   - `int motorFoward(uint8_t motorID)`: 控制指定电机前进。
   - `int motorback(uint8_t motorID)`: 控制指定电机后退。
   - `int motorStop()`: 停止所有电机。
   - `int motorSetSpeed(uint16_t speed)`: 设置电机的速度。
   - `long long motorReadAngle(uint8_t motorID)`: 读取指定电机的角度。
   - `int readErrorState(uint8_t motorID)`: 读取电机错误状态。
   - `int clearErrorState(uint8_t motorID)`: 清除电机错误状态。

### RobotControl.cpp
`RobotControl.cpp` 文件包含以下主要功能：

1. **电机前进**:
   - 函数: `motorFoward(uint8_t motorID)`
   - 描述: 控制指定电机前进。
   - 参数: `motorID` - 电机ID。

2. **电机后退**:
   - 函数: `motorback(uint8_t motorID)`
   - 描述: 控制指定电机后退。
   - 参数: `motorID` - 电机ID。

3. **电机停止**:
   - 函数: `motorStop()`
   - 描述: 停止所有电机。

4. **设置电机速度**:
   - 函数: `motorSetSpeed(uint16_t speed)`
   - 描述: 设置电机的速度。
   - 参数: `speed` - 电机速度。

5. **读取电机角度**:
   - 函数: `motorReadAngle(uint8_t motorID)`
   - 描述: 读取指定电机的角度。
   - 参数: `motorID` - 电机ID。

### MotorControl.h
`MotorControl.h` 文件包含以下主要内容：

1. **成员变量**:
   - `MotorCan usbCan`: MotorCan 对象，用于与电机通信。
   - `int32_t angle`: 电机角度。
   - `int16_t ntorque`: 电机扭矩。
   - `float now_speed`: 当前速度。
   - `float max_speed`: 最大速度。

2. **构造函数**:
   - `MotorControl()`: 默认构造函数，初始化成员变量。
   - `MotorControl(MotorCan *Can)`: 带参数的构造函数，使用 `MotorCan` 对象初始化 `usbCan`。

3. **析构函数**:
   - `~MotorControl()`: 析构函数。

4. **主要方法**:
   - `int robotForward()`: 控制机器人前进。
   - `int robotBack()`: 控制机器人后退。
   - `int robotStop()`: 停止机器人。
   - `int robotLeft()`: 控制机器人左转。
   - `int robotRight()`: 控制机器人右转。
   - `int robotChangeSpeed(float speed)`: 改变机器人的速度。

### MotorControl.cpp
`MotorControl.cpp` 文件包含以下主要功能：

1. **机器人前进**:
   - 函数: `robotForward()`
   - 描述: 控制机器人前进。

2. **机器人后退**:
   - 函数: `robotBack()`
   - 描述: 控制机器人后退。

3. **机器人停止**:
   - 函数: `robotStop()`
   - 描述: 停止机器人。

4. **机器人左转**:
   - 函数: `robotLeft()`
   - 描述: 控制机器人左转。

5. **机器人右转**:
   - 函数: `robotRight()`
   - 描述: 控制机器人右转。

### MotorCan.h
`MotorCan.h` 文件包含以下主要内容：

1. **成员变量**:
   - `long long _current_angle_count[8]`: 当前角度计数。
   - `int16_t _current_state[8][8]`: 当前状态。
   - `VCI_CAN_OBJ vco_send[8]`: 发送数据对象。
   - `VCI_CAN_OBJ vco_receive[64]`: 接收数据对象。
   - `int nDeviceType`: 设备类型。
   - `int nDeviceInd`: 设备索引。
   - `int nCANInd`: CAN 通道索引。
   - `uint16_t nMotorspeed`: 电机速度。

2. **构造函数**:
   - `MotorCan()`: 默认构造函数，初始化成员变量。
   - `MotorCan(int canID)`: 带参数的构造函数，使用 CAN ID 初始化。

3. **析构函数**:
   - `~MotorCan()`: 析构函数。

4. **主要方法**:
   - `int StartUSBCan()`: 启动 USB CAN。
   - `int StartDevice()`: 启动设备。
   - `int CloseDevice()`: 关闭设备。
   - `int SendIncrementCommand(uint16_t sendspeed, int32_t sendangle, uint8_t motorID)`: 发送增量控制指令。
   - `int readMotorState(uint8_t motorID)`: 读取电机状态。
   - `int SetSpeed(int32_t sendspeed, uint8_t motorID)`: 设置电机速度。

### MotorCan.cpp
`MotorCan.cpp` 文件包含以下主要功能：

1. **发送增量控制指令**:
   - 函数: `SendIncrementCommand(uint16_t sendspeed, int32_t sendangle, uint8_t motorID)`
   - 描述: 发送增量控制指令以控制电机。
   - 参数: `sendspeed` - 速度, `sendangle` - 角度, `motorID` - 电机ID。

2. **读取电机状态**:
   - 函数: `readMotorState(uint8_t motorID)`
   - 描述: 读取指定电机的状态。
   - 参数: `motorID` - 电机ID。

3. **设置电机速度**:
   - 函数: `SetSpeed(int32_t sendspeed, uint8_t motorID)`
   - 描述: 设置电机的速度。
   - 参数: `sendspeed` - 速度, `motorID` - 电机ID。

### CarControlNode.cpp
`CarControlNode.cpp` 文件包含以下主要功能：

1. **主函数**:
   - 函数: `main(int argc, char **argv)`
   - 描述: 初始化ROS节点，订阅话题，并控制机器人运行。
   - 参数: `argc` - 命令行参数数量, `argv` - 命令行参数数组。

### timer.h
`timer.h` 文件包含以下主要内容：

1. **TimerServer 类**:
   - 定义了一个定时器服务器类 `TimerServer`，用于管理多个定时器任务。
   - 提供了启动、停止定时器和设置定时器的方法。
   - 使用 `std::priority_queue` 管理定时器队列，使用 `std::thread` 实现定时器线程。

### RobotCarMode.h
`RobotCarMode.h` 文件包含以下主要内容：

1. **成员变量**:
   - `MotorControl m_robotCar`: 用于控制小车的马达。
   - `int prev_command`: 上一个命令。
   - `int next_command`: 下一个命令。
   - `double prev_Vx, Vx`: 上一个和当前的 x 方向速度。
   - `double prev_Vy, Vy`: 上一个和当前的 y 方向速度。
   - `double prev_w, w`: 上一个和当前的角速度。
   - `double speed`: 当前速度。

2. **构造函数**:
   - `RobotCarMode()`: 默认构造函数，初始化成员变量。
   - `RobotCarMode(MotorCan *CarCan)`: 带参数的构造函数，使用 `MotorCan` 对象初始化 `m_robotCar`。

3. **析构函数**:
   - `~RobotCarMode()`: 析构函数。

4. **主要函数**

   - `void joyNodeHandler(const std_msgs::String::ConstPtr& msg)`: 处理手柄输入的回调函数，根据输入设置小车的运动命令。
   
    * @brief 处理手柄输入的回调函数，根据输入设置小车的运动命令。
    * 
    * 该函数接收一个 `std_msgs::String::ConstPtr` 类型的参数 `msg`，表示手柄输入的消息。
    * 函数内部会解析该消息，并根据解析结果设置小车的运动命令。
    * 
    * @param msg 手柄输入的消息，类型为 `std_msgs::String::ConstPtr`。
    * 
    * 内部实现：
    * 1. 从 `msg` 中提取手柄输入的字符串。
    * 2. 根据提取的字符串判断手柄的操作类型（如前进、后退、左转、右转等）。
    * 3. 根据操作类型设置相应的小车运动命令。
    * 4. 将运动命令发送给小车的控制模块。
    
   
   - `int run()`: 控制小车执行命令的方法，根据当前命令控制小车的运动，并将命令置为 `RESET`。

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

## 启动指南
### 启动server

1. 打开终端并进入项目目录：
   ```bash
   cd /home/pi/ROBOT-CAR
   ```

2. 进入 `ros_connect` 目录并设置环境变量：
   ```bash
   cd ros_connect
   source devel/setup.bash
   ```

3. 启动 `mjpeg_server_node`：
   ```bash
   rosrun mjpeg_server_node mjpeg_server_node
   ```

4. 启动 `udp_server_node`：
   ```bash
   rosrun udp_server udp_server_node
   ```

### 启动 motorControl

   1. 打开终端并进入项目目录：
      ```bash
      cd /home/pi/ROBOT-CAR
      ```

   2. 进入 `motorControl` 目录并设置环境变量：
      ```bash
      cd motorControl
      source devel/setup.bash
      ```

   3. 启动 `carControlNode`：
      ```bash
      rosrun carControlNode carControlNode
      ```