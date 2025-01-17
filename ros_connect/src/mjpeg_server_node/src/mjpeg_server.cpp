#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <chrono>
#include <iostream>

using boost::asio::ip::tcp;

cv::Mat current_image;  // 当前图像

// 图像回调函数
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // 将ROS图像消息转换为OpenCV图像
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_image = cv_ptr->image;  // 更新当前图像帧
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert image: %s", e.what());
    }
}

// MJPEG HTTP服务器处理函数
void startMjpegServer(int port) {
    try {
        boost::asio::io_service io_service;
        tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), port));

        ROS_INFO("MJPEG Server started on port %d", port);

        while (ros::ok()) {
            tcp::socket socket(io_service);
            acceptor.accept(socket);

            std::string header = "HTTP/1.1 200 OK\r\n"
                                 "Content-Type: multipart/x-mixed-replace; boundary=--boundarydonotcross\r\n"
                                 "\r\n";
            boost::asio::write(socket, boost::asio::buffer(header));

            while (ros::ok()) {
                std::cout<<"-----------------while success---------------"<<std::endl;
                if (!current_image.empty()) {
                    
                std::cout<<"-----------------if success---------------"<<std::endl;
                    // 将图像编码为JPEG
                    std::vector<uchar> buffer;
                    cv::imencode(".jpg", current_image, buffer);
                    std::ostringstream oss;
                    oss << "--boundarydonotcross\r\n"
                        << "Content-Type: image/jpeg\r\n"
                        << "Content-Length: " << buffer.size() << "\r\n\r\n";

                    boost::asio::write(socket, boost::asio::buffer(oss.str()));
                    boost::asio::write(socket, boost::asio::buffer(buffer));

                    // 控制帧间隔
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }
        }
    } catch (std::exception& e) {
        ROS_ERROR_STREAM("MJPEG server error: " << e.what());
    }
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "mjpeg_server");
    ros::NodeHandle nh;

    // 创建图像订阅者
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub = it.subscribe("/rgb", 1, imageCallback);

    // 启动MJPEG服务器线程
    std::thread mjpeg_thread(startMjpegServer, 8080);

    // 启动ROS的消息处理循环
    ros::spin();

    // 等待MJPEG服务器线程结束
    mjpeg_thread.join();

    return 0;
}
