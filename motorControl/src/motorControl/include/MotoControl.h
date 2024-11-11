#ifndef _MOTORCONTROL_H_
#define _MOTORCONTROL_H_

#include "MotorCan.h"



class MotorControl
{
private:

    MotorCan usbCan;
    int32_t angle;
    int16_t ntorque;

    int32_t speedMode = 1;
    float timeCount = 0;
    float AcStartSpeed = 0;
    int16_t ifStop = 1;

    const uint8_t wheel_1 = 1;
    const uint8_t wheel_2 = 2;
    const uint8_t wheel_3 = 3;
    const uint8_t wheel_4 = 4;

    const int slowchangelength = 10;
    uint16_t init_speed = 1;

    float now_speed = 4;
    float max_speed = 20;
     

    const float a_PARAMETER = 0.16;
    const float b_PARAMETER = 0.16;
    float gearScale[8]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
public:
    MotorControl();
    MotorControl(MotorCan *Can);
    ~MotorControl();
    //角度控制
    int leftFrontForward();//左前轮 wheel 1
    int leftForntBackward();

    int rightFrontFoward();//右前轮 wheel 2
    int rightFrontBackward();

    int leftRearForward();//左后轮 wheel 3
    int leftRearBackward();

    int rightRearFoward();//右后轮 wheel 4
    int rightRearBackward();

    int robotForward();
    int robotBack();
    int robotLeft();
    int robotRight();
    int robotLeftForward();
    int robotRightForward();
    int robotLeftBackward();
    int robotRightBackward();
    int robotClockwise();
    int robotClockReverse();
    int robotStop();

    int robotAngleReset();

    int robotSlowStart();
    int robotSlowStop();

    //电流控制
    // int robotTorGo();
    // int robotTorStop();
    int robotChangeSpeed(float speed);
    int robotChangeMode(uint16_t the_speedMode)
    {
       speedMode = the_speedMode;
    }
    // int robotSetSpeed(uint16_t speed);
    int speedIncreaseCurve();
    int keepTorque();//不保持力矩的原因猜测应该是电机stop.

    int getRobotSpeed();
    float getTimeCount(); // lihp ADD, not used
    int WriteAngleTest(); // lihp 1001 test, can be deleted

    int xyw2wheel_1(float x,float y,float w);//手柄
    int xyw2wheel_2(float x, float y, float w);//遥控
    int rotate180(); //原地旋转180
    int foward_1cm(); // 前进1cm
    int backward_1cm(); // 后退1cm
    int left_1cm(); // 左移1cm
    int right_1cm();// 右移1cm

    int robotGearRun();
    int robotGearRun_test(int _angle1, int _angle2, int _angle3, int _angle4); // 测试前进后退左右移动确定距离

    int testPoseCode(int speed3, int speed5, int speed6, int angle3, int angle5, int angle6);
    int poseA2B_1();
    int poseA2B_2();
    int poseB2A();
    int poseB2C_1();
    int poseB2C_2();
    int poseC2B();

};

#endif
