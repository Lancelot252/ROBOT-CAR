#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

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

int main(int argc, char** argv) {
    // 初始化 ROS
    ros::init(argc, argv, "udp_server");
    ros::NodeHandle nh;

    // 初始化 UDP 套接字
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        ROS_ERROR("Failed to create socket!");
        return 1;
    }

    std::string host = "0.0.0.0";
    int port = 9027;
    std::string robot_type = "SPIDER";
    std::string sn = getCpuSerialNumber();

    struct sockaddr_in server_addr, client_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;  // 绑定到所有网络接口
    server_addr.sin_port = htons(port);

    // 绑定套接字
    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ROS_ERROR("Failed to bind socket!");
        close(sockfd);
        return 1;
    }

    char buffer[1024];
    socklen_t addr_len = sizeof(client_addr);

    ROS_INFO("Server listening on %s:%d", host.c_str(), port);

    while (ros::ok()) {
        ssize_t recv_len = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0, (struct sockaddr *)&client_addr, &addr_len);
        if (recv_len < 0) {
            ROS_ERROR("Error receiving data!");
            continue;
        }

        buffer[recv_len] = '\0';  // Null-terminate the received data
        std::string msg(buffer);

        ROS_INFO("Received message: %s", msg.c_str());

        // 如果接收到 "LOBOT_NET_DISCOVER" 消息，则发送响应
        if (msg == "LOBOT_NET_DISCOVER") {
            std::string response = robot_type + ":" + sn + "\n";
            ssize_t send_len = sendto(sockfd, response.c_str(), response.size(), 0, (struct sockaddr *)&client_addr, addr_len);
            if (send_len < 0) {
                ROS_ERROR("Error sending response!");
            } else {
                ROS_INFO("Sent response: %s", response.c_str());
            }
        }
    }

    // 关闭套接字
    close(sockfd);
    return 0;
}
