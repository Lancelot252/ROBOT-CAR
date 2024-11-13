#include "ros/ros.h"
#include "std_msgs/String.h"
#include <arpa/inet.h>      // 网络连接
#include <sys/socket.h>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <cstdlib>

#define PORT 9026           // 信号监听端口号
#define BUFFER_SIZE 1024    // 缓冲区大小
#define DISCOVERY_PORT 9027 // 设备发现广播端口
#define WIFI_CONFIG_FILE "/home/pi/SmartCar_Dir/SmartCar_1.1_beta/phoneconnect/src/phoneconnect/src/wpa_supplicant.conf"  // WiFi配置文件路径


// 连接WiFi函数
bool connectWiFi(const std::string& ssid, const std::string& password) {
    std::string command = "nmcli dev wifi connect " + ssid + " password " + password;
    int result = system(command.c_str());
    return result == 0;
}

// 读取WiFi配置文件
bool readWiFiConfig(std::string& ssid, std::string& password) {
    std::ifstream file(WIFI_CONFIG_FILE);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open WiFi config file.");
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.find("ssid") != std::string::npos) {
            ssid = line.substr(line.find('=') + 1);
        } else if (line.find("psk") != std::string::npos) {
            password = line.substr(line.find('=') + 1);
        }
    }
    file.close();
    return true;
}

// 设备发现函数
void deviceDiscovery(const std::string& deviceType, const std::string& serialNumber) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        ROS_ERROR("Socket creation failed for device discovery.");
        return;
    }

    struct sockaddr_in broadcast_addr;
    memset(&broadcast_addr, 0, sizeof(broadcast_addr));
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_addr.s_addr = inet_addr("192.168.149.1");  // 广播地址
    broadcast_addr.sin_port = htons(DISCOVERY_PORT);

    std::string discovery_msg = "Device Type: " + deviceType + ", Serial Number: " + serialNumber;
    sendto(sockfd, discovery_msg.c_str(), discovery_msg.size(), 0,
           (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));

    ROS_INFO("Device discovery message sent: %s", discovery_msg.c_str());
    close(sockfd);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "network_listener");
    ros::NodeHandle nh;

    ros::Publisher signal_pub = nh.advertise<std_msgs::String>("Smartphone", 1000);
    ROS_INFO("init success\n");
    // 连接WiFi
    std::string ssid, password;
    if (readWiFiConfig(ssid, password)) {
        ROS_INFO("read success");
        if (connectWiFi(ssid, password)) {
            ROS_INFO("Connected to WiFi SSID: %s", ssid.c_str());
        } else {
            ROS_ERROR("Failed to connect to WiFi.");
            return -1;
        }
    } else {
        ROS_ERROR("No WiFi configuration found.");
        return -1;
    }

    // 设备发现
    std::string deviceType = "SPIDER";  // 可以自定义设备类型
    std::string serialNumber = "12345678";  // 示例序列号，可以从硬件或配置获取
    deviceDiscovery(deviceType, serialNumber);

    // 创建网络监听socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        ROS_ERROR("Socket creation failed.");
        return -1;
    }

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ROS_ERROR("Socket binding failed.");
        close(sockfd);
        return -1;
    }

    ROS_INFO("Network listener is running, waiting for signals...");

    // 接收并发布信号
    while (ros::ok()) {
        char buffer[BUFFER_SIZE];
        struct sockaddr_in client_addr;
        socklen_t len = sizeof(client_addr);

        int n = recvfrom(sockfd, buffer, BUFFER_SIZE, MSG_WAITALL, (struct sockaddr *)&client_addr, &len);
        if (n < 0) {
            ROS_ERROR("Failed to receive data.");
            continue;
        }

        buffer[n] = '\0';  // 确保字符串以null结尾

        // 将接收到的信号封装成ROS消息并发布
        std_msgs::String msg;
        msg.data = buffer;
        signal_pub.publish(msg);

        ROS_INFO("Received signal: %s", buffer);

        ros::spinOnce();  // 确保ROS正常运行
    }

    close(sockfd);
    return 0;
}
