#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include <jsoncpp/json/json.h>  // JsonCpp库
#include <arpa/inet.h>

using boost::asio::ip::tcp;

// 获取CPU序列号
std::string getCpuSerialNumber() {
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string line, serial_num;
    while (std::getline(cpuinfo, line)) {
        if (line.find("Serial") == 0) {
            serial_num = line.substr(line.find(":") + 2);
            break;
        }
    }
    if (!serial_num.empty()) {
        std::reverse(serial_num.begin(), serial_num.end());
        serial_num = serial_num.substr(0, 16);
        std::transform(serial_num.begin(), serial_num.end(), serial_num.begin(), ::toupper);
    }
    return serial_num;
}

// UDP设备发现线程函数
void udpDeviceDiscovery(int port, const std::string& robot_type, const std::string& serial_number) {
    int udp_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sockfd < 0) {
        ROS_ERROR("Failed to create UDP socket!");
        return;
    }

    struct sockaddr_in server_addr, client_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(udp_sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        ROS_ERROR("Failed to bind UDP socket!");
        close(udp_sockfd);
        return;
    }

    ROS_INFO("UDP Device Discovery listening on port %d", port);

    char buffer[1024];
    socklen_t addr_len = sizeof(client_addr);
    while (ros::ok()) {
        ssize_t recv_len = recvfrom(udp_sockfd, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&client_addr, &addr_len);
        if (recv_len > 0) {
            buffer[recv_len] = '\0';
            std::string msg(buffer);

            if (msg == "LOBOT_NET_DISCOVER") {
                std::string response = robot_type + ":" + serial_number + "\n";
                sendto(udp_sockfd, response.c_str(), response.size(), 0, (struct sockaddr*)&client_addr, addr_len);
                ROS_INFO("Sent UDP response: %s", response.c_str());
            }
        }
    }

    close(udp_sockfd);
}


// 处理 JSON-RPC 请求
void handleJsonRpcRequests(int tcp_port, ros::Publisher& publisher) {
    try {
        boost::asio::io_service io_service;
        tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), tcp_port));

        ROS_INFO("JSON-RPC Server listening on TCP port %d", tcp_port);

        while (ros::ok()) {
            tcp::socket socket(io_service);
            acceptor.accept(socket);

            ROS_INFO("TCP Client connected!");

            try {
                // 接收数据
                boost::asio::streambuf buffer;
                size_t bytes_transferred = boost::asio::read_until(socket, buffer, "\n");

                if (bytes_transferred == 0) {
                    ROS_WARN("Received empty message from client.");
                    continue;
                }

                std::istream input(&buffer);
                std::string request((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());

                // 打印接收到的原始数据
                ROS_INFO_STREAM("Raw data received: " << request);

                // 查找JSON部分的开始和结束位置
                size_t jsonStartPos = request.find("{");
                size_t jsonEndPos = request.rfind("}");

                if (jsonStartPos != std::string::npos && jsonEndPos != std::string::npos) {
                    // 截取 JSON 字符串
                    std::string jsonStr = request.substr(jsonStartPos, jsonEndPos - jsonStartPos + 1);
                    ROS_INFO("Extracted JSON: %s", jsonStr.c_str());
                    
                    // 解析 JSON
                    Json::Value root;
                    Json::CharReaderBuilder readerBuilder;
                    std::string errs;
                    std::istringstream requestStream(jsonStr);
                    if (!Json::parseFromStream(readerBuilder, requestStream, &root, &errs)) {
                        ROS_WARN_STREAM("Failed to parse JSON: " << errs);//错误信息：无法解析 JSON
                    } else {
                        // 输出解析后的 JSON
                        ROS_INFO_STREAM("Parsed JSON: " << root.toStyledString());

                        // 根据请求发布ROS话题
                        if (root["method"] == "SetMovementAngle") {//水平移动
                            int angle = root["params"][0].asInt();
                            std_msgs::String msg;
                            switch (angle) {
                                case 90:
                                    msg.data = "forward";
                                    break;
                                case 270:
                                    msg.data = "backward";
                                    break;
                                case 180:
                                    msg.data = "left";
                                    break;
                                case 360:
                                    msg.data = "right";
                                    break;
                                case -1:
                                    msg.data = "stop";
                                    break;
                                default:
                                    msg.data = "unknown";
                            }
                        
                            publisher.publish(msg);
                            ROS_INFO("Published movement command: %s", msg.data);
                        }
                        if(root["method"]=="SetBrushMotor"){//水平旋转
                            std_msgs::String msg;
                            int angle = root["params"][1].asInt();
                            ROS_INFO("%d",angle);
                            switch(angle){
                                case -100:
                                    msg.data = "turnleft";
                                    break;
                                case 100:
                                    msg.data = "turnright";
                                    break;
                                default:
                                    msg.data = "unknown";
                            }
                            publisher.publish(msg);
                            ROS_INFO("Published movement command: %s", msg.data);
                        }

                        // 返回 JSON-RPC 响应
                        Json::Value response;
                        response["jsonrpc"] = "2.0";
                        response["id"] = root["id"];
                        response["result"] = "Success";

                        std::string responseStr = response.toStyledString();
                        boost::asio::write(socket, boost::asio::buffer(responseStr + "\n"));
                    }
                } else {
                    ROS_WARN_STREAM("No valid JSON found in the request.");
                }

            } catch (std::exception& e) {
                ROS_ERROR_STREAM("Error handling JSON-RPC request: " << e.what());
            }
        }
    } catch (std::exception& e) {
        ROS_ERROR_STREAM("Error starting JSON-RPC server: " << e.what());//输出错误信息
    }
}

int main(int argc, char** argv) {
    // 初始化 ROS
    ros::init(argc, argv, "json_rpc_server");
    ros::NodeHandle nh;

    // 创建 ROS 发布者
    ros::Publisher publisher = nh.advertise<std_msgs::String>("phonejoy", 1000);

    // 获取 CPU 序列号
    std::string serial_number = getCpuSerialNumber();
    std::string robot_type = "SPIDER";

    // 启动 UDP 设备发现线程
    std::thread udp_thread(udpDeviceDiscovery, 9027, robot_type, serial_number);

    // 主线程处理 JSON-RPC 请求
    handleJsonRpcRequests(9030, publisher);

    // 等待 UDP 线程结束
    udp_thread.join();

    return 0;
}
