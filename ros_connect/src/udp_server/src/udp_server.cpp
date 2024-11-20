#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <httplib.h>
#include <nlohmann/json.hpp>

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

// 处理接收到的命令并发布 ROS 话题
void handleCommand(const std::string& requestBody, ros::Publisher& publisher) {
    try {
        nlohmann::json jsonObj = nlohmann::json::parse(requestBody);
        std::string method = jsonObj["method"];
        int id = jsonObj["id"];
        nlohmann::json params = jsonObj["params"];

        std_msgs::String msg;
        //msg.data = jsonObj.dump();  // 将 JSON 对象转换为字符串

        if (method == "SetMovementAngle")
        {
            switch (params[0])
            {
            case 90://前进
                msg.data = "forward";
                ROS_INFO("forward");
                break;
            
            case 270://后退
                msg.data = "backward";
                ROS_INFO("backward");
                break;

            case 180://左转
                msg.data = "left";
                ROS_INFO("left");
                break;

            case 360://右转
                msg.data = "right";
                ROS_INFO("right");
                break;

            case -1://停止
                msg.data = "stop";
                ROS_INFO("stop");
                break;
            
            default:
                break;
            }
        }
        

        publisher.publish(msg);  // 发布 ROS 话题
    } catch (const std::exception& e) {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
    }
}

int main(int argc, char** argv) {
    // 初始化 ROS
    ros::init(argc, argv, "udp_server");
    ros::NodeHandle nh;

    // 创建 ROS 发布者
    ros::Publisher publisher = nh.advertise<std_msgs::String>("phonejoy", 1000);

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
        /*if (msg == "LOBOT_NET_DISCOVER") {
            std::string response = robot_type + ":" + sn + "\n";
            ssize_t send_len = sendto(sockfd, response.c_str(), response.size(), 0, (struct sockaddr *)&client_addr, addr_len);
            if (send_len < 0) {
                ROS_ERROR("Error sending response!");
            } else {
                ROS_INFO("Sent response: %s", response.c_str());
            }
        }*/
        // 处理接收到的 JSON 数据并发布 ROS 话题
        handleCommand(msg, publisher);
    }

    // 关闭套接字
    close(sockfd);
    return 0;
}
