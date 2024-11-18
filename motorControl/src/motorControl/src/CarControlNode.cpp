#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include "RobotCarMode.h"
#include "MotoControl.h"
#include "RobotControl.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "CarControlNode");
    ros::NodeHandle n;
    
    //初始化小车运动相关物理参数和Can
    RobotControl Car{0};

    //
    RobotCarMode robot{Car.getUsbCan()};

    ros::Subscriber subJoy = n.subscribe("phonejoy", 1, &RobotCarMode::joyNodeHandler,&robot);

    //  做测试使用
    //ros::Subscriber subCmdVel = n.subscribe("cmd_vel", 1, &RobotCarMode::cmdVelHandler,&robot);
    
    ros::Rate rate(10);
	
    while(ros::ok())
    {
        robot.run();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
