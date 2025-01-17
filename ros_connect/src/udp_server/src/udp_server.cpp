#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include <jsoncpp/json/json.h>  // JsonCpp库
#include <arpa/inet.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
// 移除 #include <http_server_cpp/server.hpp>
using boost::asio::ip::tcp;
cv::Mat current_image;  // 存储当前图像帧
std::mutex image_mutex;
// 图像回调函数
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(image_mutex);
    try
    {
        
        // 将ROS图像消息转换为OpenCV图像
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_image = cv_ptr->image;  // 更新���局图像帧
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert image: %s", e.what());
    }
}

// 添加HTTP服务器处理函数
void httpServer(int port) {
    try {
        boost::asio::io_service io_service;

        // 创建接受器，监听指定端口
        boost::asio::ip::tcp::acceptor acceptor(io_service,
        boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port));
        ROS_INFO("MJPEG Server started on port %d", port);
        while (ros::ok()) {
            boost::asio::ip::tcp::socket socket(io_service);
            acceptor.accept(socket);

            // 创建HTTP响应头
            std::string header = "HTTP/1.1 200 OK\r\n"
                                 "Content-Type: multipart/x-mixed-replace; boundary=--boundarydonotcross\r\n"
                                 "\r\n";

            boost::asio::write(socket, boost::asio::buffer(header));

            while (ros::ok()) {
                std::cout<<"-------while success----------"<<std::endl;
                if (!current_image.empty()) {
                    std::cout<<"-------if success----------"<<std::endl;
                    std::vector<uchar> buffer;
                    cv::imencode(".jpg", current_image, buffer);

                    std::ostringstream oss;
                    oss << "--boundarydonotcross\r\n"
                        << "Content-Type: image/jpeg\r\n"
                        << "Content-Length: " << buffer.size() << "\r\n\r\n";

                    boost::asio::write(socket, boost::asio::buffer(oss.str()));
                    boost::asio::write(socket, boost::asio::buffer(buffer));

                    
                    // 等待一段时间再发送下一帧
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }
        }
    } catch (std::exception& e) {
        ROS_ERROR_STREAM("HTTP server error: " << e.what());
    }
}

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

                // 打���接收到的原始数据
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
                        if(root["method"]=="SetBrushMotor"){//水平旋��
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
                        if(root["method"]=="SetPWMServo"){
                            std_msgs::String msg;
                            double speed_percent = (root["params"][3].asInt()+90)/180.0;
                            double speed = speed_percent*15;
                            ROS_INFO("%.2lf",speed);
                            if(speed==0){
                                msg.data = "stop";
                            }else if(speed>0&&speed<=5){
                                msg.data = "slow";
                            }else if(speed>5&&speed<=10){
                                msg.data = "medium";
                            }else if(speed>10&&speed<=15){
                                msg.data = "fast";
                            }
                            publisher.publish(msg);
                            ROS_INFO("Published movement command: %s", msg.data.c_str());
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

   // 创建图像订阅者
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub = it.subscribe("/rgb", 1, imageCallback);

    // 启动HTTP服务器线程
    std::thread http_thread(httpServer, 8080);

    // 主线程处理 JSON-RPC 请求
    handleJsonRpcRequests(9030, publisher);
    
    ros::spin();
    // 等待 UDP 线程结束
    udp_thread.join();

    // 等待HTTP线程结束
    http_thread.join();

    return 0;
}