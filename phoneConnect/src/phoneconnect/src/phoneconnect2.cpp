#include "ros/ros.h"
#include "std_msgs/String.h"
#include <arpa/inet.h>      // 网络连接
#include <sys/socket.h>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <cstdlib>
#include <microhttpd.h>  // 添加 libmicrohttpd 库用于 HTTP 处理

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
        } else if (line.find("password") != std::string::npos) {
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

// 处理 HTTP 请求的回调函数
static int answer_to_connection(void *cls, struct MHD_Connection *connection, const char *url, const char *method,
                                const char *version, const char *upload_data, size_t *upload_data_size, void **con_cls) {
    // 只处理 POST 请求
    if (strcmp(method, "POST") != 0) {
        return MHD_NO; // 我们只处理 POST 请求
    }

    // 从请求中获取参数（假设参数为 JSON 格式或者简单的数值参数）
    const char *data = upload_data;  // 获取传入的数据

    // 在此处解析 Android 发来的数据并传递给 postRpc 函数处理
    // 假设收到的数据是一个控制指令，我们将它传递给 postRpc
    int command = atoi(data); // 简化为直接解析整数数据
    int cmd_data[] = {command}; // 将数据封装成数组传递

    postRpc("SetMovementAngle", 1, cmd_data, nullptr, nullptr);  // 调用 postRpc 方法处理

    // 构建响应
    const char *response = "OK"; // 响应内容
    struct MHD_Response *response_obj = MHD_create_response_from_buffer(strlen(response), (void *)response,
                                                                         MHD_RESPMEM_PERSISTENT);
    int ret = MHD_queue_response(connection, MHD_HTTP_OK, response_obj);
    MHD_destroy_response(response_obj);
    return ret;
}

// 启动 HTTP 服务器
void startHttpServer() {
    struct MHD_Daemon *daemon = MHD_start_daemon(MHD_USE_THREAD_PER_CONNECTION, 9030, nullptr, nullptr, &answer_to_connection,
                                                 nullptr, MHD_OPTION_END);
    if (daemon == nullptr) {
        ROS_ERROR("Failed to start HTTP server.");
    }
    else {
        ROS_INFO("HTTP server is running on port 9030");
    }
}

// Post 发送JSON-PRC指令
void postRpc(const std::string& method, int type, int* i, const char** strings, const float* floats) {
    try {
        std::string urll = "http://";
        urll += "127.0.0.1";  // 本机 IP 或者设备的 IP 地址
        urll += ":9030/";

        // 这里可以添加与 Android 端的 HTTP 请求逻辑
        ROS_INFO("Sending RPC command: %s", method.c_str());
    }
    catch (const std::exception &e) {
        ROS_ERROR("Error in postRpc: %s", e.what());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "network_listener");
    ros::NodeHandle nh;

    // 启动 HTTP 服务器，监听来自 Android 应用的 HTTP 请求
    startHttpServer();

    // 连接WiFi
    std::string ssid, password;
    if (readWiFiConfig(ssid, password)) {
        ROS_INFO("WiFi configuration loaded successfully.");
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
