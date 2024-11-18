// 成员：
// Car 运动相关命令，速度，角速度，马达(m_robotCar)

// 重要函数：
// joyhandler：对手柄按钮命令的处理(回调)
// run：控制motor，执行命令，并将命令置为RESET(主函数循环内)

// 功能：
// 实现读取手柄移动命令和控制马达执行命令

// 关联：
// m_robotCar 是MotorControl类型的，
// motorcontrol用于控制四个轮子

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include "ButtonMaps.h"
#include "MotoControl.h"
#include "RobotControl.h"

class RobotCarMode{
    enum COMMAND{
        RESET = 0,
        STOP = 1,
        SLOW_STOP,
        GO,
        SPEED_MODE_1,
        SPEED_MODE_5,
        SPEED_MODE_15,

        SPEED_UP,
        SPEED_DOWN,

	    LEFT_1CM,
	    RIGHT_1CM,
	    FORWARD_1CM,
	    BACKWARD_1CM,
	    ROTATE_180
    };
    private: 
    //马达控制
        MotorControl m_robotCar;
        int prev_command;
        int next_command;
        double prev_Vx;
        double Vx;
        double prev_Vy;
        double Vy;
        double prev_w;
        double w;

        double speed;

    public:
        RobotCarMode():
            m_robotCar{},prev_command{-1},next_command{0},
            prev_Vx{-1.0},
            Vx{0.0},
            prev_Vy{-1.0},
            Vy{0.0},
            prev_w{-1.0},
            w{0.0},
            speed{0.0}
        {

        };
        RobotCarMode(MotorCan *CarCan):
            m_robotCar{CarCan},next_command{0},
            prev_Vx{-1.0},
            Vx{0.0},
            prev_Vy{-1.0},
            Vy{0.0},
            prev_w{-1.0},
            w{0.0},
            speed{0.0}
        {

        };
        ~RobotCarMode(){};

        // 路径规划->小车
//        void cmdVelHandler(const geometry_msgs::Twist& vel_msg){
        //     ROS_INFO("Received a /cmd_vel message!");
    	//     ROS_INFO("Linear Components:[%f,%f,%f]",vel_msg.linear.x,vel_msg.linear.y,vel_msg.linear.z);
	    //     ROS_INFO("Angular Components:[%f,%f,%f]",vel_msg.angular.x,vel_msg.angular.y,vel_msg.angular.z);
//
        //     Vx = vel_msg.linear.y; // left rocker, horizontal, from left to rihgt [1 -1]
        //     Vy = vel_msg.linear.x; // left rocker, vertial, form top to bottom [1 -1]
        //     w = vel_msg.angular.z; // right rocker, horizontal, from left to right [1 -1]
        //     if (Vx || Vy || w){
        //         if (!(Vx<=0.01 && Vx>=-0.01 && Vy<=0.01 && Vy>=-0.01 && w<=0.01 && w>=-0.01))
        //             {   
        //                 next_command = COMMAND::GO;
        //             }
        //         else
        //             {}
        //     }
        //     //no input (loose all button)
        //     //no input (loose all button)
        //     else{
        //         if (next_command != COMMAND::RESET){
        //             std::cout <<"change to STOP" << "\n";
        //             next_command = COMMAND::STOP;
        //         }
        //         // stop status,only send stop command once
        //         else if (next_command == COMMAND::STOP){
        //             next_command = COMMAND::RESET;
        //         }
        //         else{
        //             //std::cout <<"keep reset" <<"\n";
        //         }
        //     }
        // }

        void joyNodeHandler(const sensor_msgs::Joy::ConstPtr& msg){
	
            std::cout << "carmode::joyhandler" << "\n";
            //Vx = msg->axes[0]; // left rocker, horizontal, from left to rihgt [1 -1]
            //Vy = msg->axes[1]; // left rocker, vertial, form top to bottom [1 -1]
            //w = msg->axes[2]; // right rocker, horizontal, from left to right [1 -1]

	    //start
	        if(msg.data == "forward"){
                Vx=0.5;
            }else if(msg.data == "backward"){
                Vx=-0.5;
            }else{
                Vx=0;
            }

            if(msg.data == "left"){
                Vy=-0.5;
            }else if(msg.data == "right"){
                Vy=0.5;
            }else{
		        Vy=0;
	        }

            if(BUTTON_L3){
                w=-0.5;
            }else if(BUTTON_R3){
                w=0.5;
            }else{
                w=0;
            }
	    //end（此处将控制改为按键）
	    
            if (msg.data == "stop"){
                next_command = COMMAND::STOP;
            }
            else if (BUTTON_Y){
                next_command = COMMAND::SPEED_MODE_1;
            }
            else if (BUTTON_A){
                //next_command = COMMAND::SPEED_MODE_5;
                next_command = COMMAND::SPEED_UP;//new
            }
            else if (BUTTON_B){
                //next_command = COMMAND::SPEED_MODE_15;
                next_command = COMMAND::SPEED_DOWN;//new
            }
	    /*(else if (BUTTON_L1){
		std::cout<<"向左1cm"<<std::endl;
                m_robotCar.xyw2wheel_1(1,0,0);
                m_robotCar.left_1cm();
	    }
	    else if (BUTTON_L2){
		std::cout<<"向后1cm"<<std::endl;
                m_robotCar.xyw2wheel_1(1,0,0);
                m_robotCar.backward_1cm();
            }
	    else if (BUTTON_R1){
		std::cout<<"向右1cm"<<std::endl;
                m_robotCar.xyw2wheel_1(1,0,0);
                m_robotCar.right_1cm();
            }
	    else if (BUTTON_R2){
		std::cout<<"向前1cm"<<std::endl;
                m_robotCar.xyw2wheel_1(1,0,0);
                m_robotCar.foward_1cm();
            }
	    else if (BUTTON_L3){
		std::cout<<"掉头"<<std::endl;
                m_robotCar.xyw2wheel_1(1,0,0);
                m_robotCar.rotate180();
            }
*/
            else if (Vx || Vy || w){
                if (!(Vx<=0.01 && Vx>=-0.01 && Vy<=0.01 && Vy>=-0.01 && w<=0.01 && w>=-0.01)){   
                        next_command = COMMAND::GO;
                }else{} 
            }
            //no input (loose all button)
            //no input (loose all button)
            else{
                if (next_command != COMMAND::RESET){
                    std::cout <<"change to STOP" << "\n";
                    next_command = COMMAND::STOP;
                }
                // stop status,only send stop command once
                else if (next_command == COMMAND::STOP){
                    next_command = COMMAND::RESET;
                }
                else{
                    //std::cout <<"keep reset" <<"\n";
                }
            }
        };
        
        int run(){		
            //same command execute only once
             //if (next_command == prev_command && prev_Vx == Vx && prev_Vy == Vy && prev_w == w){
                 //std::cout << "Command: " << next_command << " Direction: " << next_direction << "\n";
            //    // std::cout << "Same Command and Direction. not execute" << "\n";
            //     return 0;
            // }
            
            prev_command = next_command;
            prev_Vx = Vx;
            prev_Vy = Vy;
            prev_w = w;
            //std::cout << "RobotCar run" << "\n";
            //std::cout << "next_command: " << next_command << "\n";
            if (next_command == COMMAND::STOP && prev_command != COMMAND::RESET){
                m_robotCar.robotStop();
                std::cout << "car stop....." << "\n";
                next_command = COMMAND::RESET;
            }
            else if(next_command == COMMAND::GO){
                m_robotCar.xyw2wheel_1(Vx,Vy,w);//改变速度、方向
                m_robotCar.robotGearRun();
                std::cout << "car go, speed: " << m_robotCar.getRobotSpeed() << "\n";
            }
            else if(next_command == COMMAND::SPEED_MODE_1)
            {
                m_robotCar.robotChangeSpeed(1.8);
                std::cout << "change speed 1" << "\n";
                    
            }
            else if(next_command == COMMAND::SPEED_MODE_5)
            {
                m_robotCar.robotChangeSpeed(3.6);
                std::cout << "change speed 3.6" << "\n";
            }
            else if(next_command == COMMAND::SPEED_MODE_15)
            {
                m_robotCar.robotChangeSpeed(15);
                std::cout << "change speed 15" << "\n";
            }
            else if(next_command == COMMAND::SPEED_UP)//new
            {
                if(speed < 15){
                    speed++;
                    m_robotCar.robotChangeSpeed(speed);
                    std::cout<<"change speed "<<speed<< "\n";
                }
            }
            else if(next_command == COMMAND::SPEED_DOWN)//new
            {
                if(speed > 1){
                    speed--;
                    m_robotCar.robotChangeSpeed(speed);
                    std::cout<<"change speed "<<speed<< "\n";
                }
            }
            else if(next_command == COMMAND::RESET){
	    	    std::cout << "run: RESET" << "\n";
	        }
            else{
	    	    std::cout << "run: else";
	        }
        };
};
