// filepath: /home/pi/ROBOT-CAR/sipeed-tofv075-ros/src/image_streamer/src/image_streamer.cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include <thread>

using boost::asio::ip::tcp;

cv::Mat current_image;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        current_image = cv_ptr->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert image: %s", e.what());
    }
}

void httpServer(int port) {
    boost::asio::io_service io_service;
    tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), port));

    while (true) {
        tcp::socket socket(io_service);
        acceptor.accept(socket);

        std::vector<uchar> buf;
        cv::imencode(".jpg", current_image, buf);
        std::string header = "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: " + std::to_string(buf.size()) + "\r\n\r\n";
        boost::system::error_code ignored_error;
        boost::asio::write(socket, boost::asio::buffer(header), ignored_error);
        boost::asio::write(socket, boost::asio::buffer(buf), ignored_error);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_streamer");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera/image", 1, imageCallback);

    std::thread server_thread(httpServer, 8080);
    ros::spin();
    server_thread.join();

    return 0;
}