#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <sstream>
#include <string>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <fstream>




// UDP接收端口
#define PORT 9027
std::string robot_type = "SMART_CAR";

// 获取 CPU 序列号
std::string getCpuSerialNumber() {
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string line;
    std::string serial_num;
    while (std::getline(cpuinfo, line)) {
        if (line.find("Serial") == 0) {  // 查找包含 'Serial' 的行
            serial_num = line.substr(line.find(":") + 2);  // 获取序列号
            break;
        }
    }
    if (!serial_num.empty()) {
        std::reverse(serial_num.begin(), serial_num.end());
        serial_num = serial_num.substr(0, 16);  // 取反并只保留16位
        std::transform(serial_num.begin(), serial_num.end(), serial_num.begin(), ::toupper);
    }
    return serial_num;
}
std::string sn = getCpuSerialNumber();
// 处理接收到的指令
void processCommand(const std::string &command,int sockfd,struct sockaddr_in client_addr,socklen_t addr_len)
{
    ROS_INFO("Received command: %s", command.c_str());
    
    // 根据指令类型进行处理
    if (command == "SetMovementAngle") {
        ROS_INFO("Processing SetMovementAngle command");
    } else if (command == "SetPWMServo") {
        ROS_INFO("Processing SetPWMServo command");
    } else if (command == "Heartbeat") {
        ROS_INFO("Processing Heartbeat command");
    } else if (command == "LOBOT_NET_DISCOVER"){
            std::string response = robot_type + ":" + sn + "\n";
            ssize_t send_len = sendto(sockfd, response.c_str(), response.size(), 0, (struct sockaddr *)&client_addr, addr_len);
            if (send_len < 0) {
                ROS_ERROR("Error sending response!");
            } else {
                
                ROS_INFO("Sent response: %s", response.c_str());
        }
    }
    else{
        ROS_WARN("Unknown command received: %s", command.c_str());
    }
}

 
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "control_listener");
    ros::NodeHandle nh;

    // 创建UDP套接字
    int sockfd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        return 1;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    
    // 设置服务器信息
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY; // 监听所有网卡
    server_addr.sin_port = htons(PORT); // 设置监听端口

    // 绑定套接字
    if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Bind failed");
        close(sockfd);
        return 1;
    }

    ROS_INFO("Listening for commands on UDP port %d", PORT);

    char buffer[1024];
    while (ros::ok())
    {
        // 接收来自Android的消息
        int n = recvfrom(sockfd, (char *)buffer, sizeof(buffer), MSG_WAITALL, (struct sockaddr *)&client_addr, &client_addr_len);
        buffer[n] = '\0'; // 确保接收到的字符串是以'\0'结束的

        std::string received_data(buffer);
        ROS_INFO("Received data: %s", received_data.c_str());

        // 处理接收到的指令
        processCommand(received_data,sockfd,client_addr,client_addr);
        // 保证ROS节点在运行
        ros::spinOnce();
    }

    close(sockfd); // 关闭UDP套接字
    return 0;
}

