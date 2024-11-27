#include "MotoControl.h"
#include "MotorCan.h"
#include <cmath>
#include "stdlib.h"
#include "unistd.h"
#include <time.h>
MotorControl::MotorControl()
{
    usbCan.StartUSBCan();
    usbCan.StartDevice();
    usbCan.MotorClose(1);
    usbCan.MotorClose(2);
    usbCan.MotorClose(3);
    usbCan.MotorClose(4);
    angle = 6000;
    ntorque = 50;
}

MotorControl::MotorControl(MotorCan *Can)
{
    usbCan = *Can;
    usbCan.MotorClose(1);
    usbCan.MotorClose(2);
    usbCan.MotorClose(3);
    usbCan.MotorClose(4);
    angle = 6000;
    ntorque = 50;
}

int MotorControl::speedIncreaseCurve()
{
    if(init_speed < now_speed-1)
    {
        init_speed += 1;
        // init_speed += 1;
        // init_speed = log(init_speed+1);
    }
    else{
        init_speed = now_speed;
    }
}
MotorControl::~MotorControl()
{
    usbCan.CloseDevice();
}

int MotorControl::leftFrontForward()//左前轮 wheel 1
{


    if(usbCan.GetMode() == 1)
    {
        usbCan.SendTorqueCommand(-ntorque,1);
    }
    else{
    // usbCan.SendIncrementCommand(init_speed,0,1);
        usbCan.SendIncrementCommand(init_speed,-angle,1);
        // std::cout << "left front forward" << std::endl;
    }
}
int MotorControl::leftForntBackward()
{
    if(usbCan.GetMode() == 1)
    {
        usbCan.SendTorqueCommand(ntorque,1);
    }
    else{
    // usbCan.SendIncrementCommand(init_speed,0,1);
    usbCan.SendIncrementCommand(init_speed,angle,1);
    }
}

int MotorControl::rightFrontFoward()//右前轮 wheel 2
{
    if(usbCan.GetMode() == 1)
    {
        usbCan.SendTorqueCommand(ntorque,2);

    }
    else{
    // usbCan.SendIncrementCommand(init_speed,0,2);
        usbCan.SendIncrementCommand(init_speed,angle,2);
        // std::cout << "right front forward" << std::endl;
    }
}
int MotorControl::rightFrontBackward()
{
    if(usbCan.GetMode() == 1)
    {
        usbCan.SendTorqueCommand(-ntorque,2);
    }
    else{
    // usbCan.SendIncrementCommand(init_speed,0,2);
    usbCan.SendIncrementCommand(init_speed,-angle,2);
    }
}

int MotorControl::leftRearForward()//左后轮 wheel 3
{
    if(usbCan.GetMode() == 1)
    {
        usbCan.SendTorqueCommand(-ntorque,3);
    }
    else{
    // usbCan.SendIncrementCommand(init_speed,0,3);
        usbCan.SendIncrementCommand(init_speed,-angle,3);
        // std::cout << "left rear forward" << std::endl;
    }
}
int MotorControl::leftRearBackward()
{
    if(usbCan.GetMode() == 1)
    {
        usbCan.SendTorqueCommand(ntorque,3);
    }
    else{
    // usbCan.SendIncrementCommand(init_speed,0,3);
    usbCan.SendIncrementCommand(init_speed,angle,3);
    }
}

int MotorControl::rightRearFoward()//右后轮 wheel 4
{
    // int16_t wheel_4_tor = (int16_t)ntorque*1.3;
    if(usbCan.GetMode() == 1)
    {
        usbCan.SendTorqueCommand(ntorque,4);
    }
    else{
    // usbCan.SendIncrementCommand(init_speed,0,4);
        usbCan.SendIncrementCommand(init_speed,angle,4);
        // std::cout << "right rear forward" << std::endl;
    }
}
int MotorControl::rightRearBackward()
{
    // int16_t wheel_4_tor = (int16_t)ntorque*1.3;
    if(usbCan.GetMode() == 1)
    {
        usbCan.SendTorqueCommand(-ntorque,4);
    }
    else{
    // usbCan.SendIncrementCommand(init_speed,0,4);
    usbCan.SendIncrementCommand(init_speed,-angle,4);
    }
}

int MotorControl::robotForward()
{
    speedIncreaseCurve();
    leftFrontForward();
    rightFrontFoward();
    leftRearForward();
    rightRearFoward();
}
int MotorControl::robotBack()
{
    speedIncreaseCurve();
    leftForntBackward();
    rightFrontBackward();
    leftRearBackward();
    rightRearBackward();
}
int MotorControl::robotLeft()
{
    speedIncreaseCurve();
    leftForntBackward();
    rightFrontFoward();
    leftRearForward();
    rightRearBackward();
}
int MotorControl::robotRight()
{
    speedIncreaseCurve();
    leftFrontForward();
    rightFrontBackward();
    leftRearBackward();
    rightRearFoward();
}

//////////////////////////////////
int MotorControl::robotLeftForward()
{
    rightFrontFoward();
    leftRearForward();
}
int MotorControl::robotRightForward()
{
    leftFrontForward();
    rightRearFoward();
}
int MotorControl::robotLeftBackward()
{
    leftForntBackward();
    rightRearBackward();
}
int MotorControl::robotRightBackward()
{
    rightFrontBackward();
    leftRearBackward();
}
//////////////////////////////////////////////
int MotorControl::robotClockwise()
{
    speedIncreaseCurve();
    leftForntBackward();
    rightFrontFoward();
    leftRearBackward();
    rightRearFoward();
}
int MotorControl::robotClockReverse()
{
    speedIncreaseCurve();
    leftFrontForward();
    rightFrontBackward();
    leftRearForward();
    rightRearBackward();
}

//////////////////////////////////////////////////////////////
// int MotorControl::robotTorGo()
// {
//     uint16_t ii = 50;
//     // usbCan.sendMultiTorqueCommand(ii,ii,ii,ii);
//     // usbCan.SendTorqueCommand(0,1);
//     // usbCan.SendTorqueCommand(ii,1);
//     // usbCan.SendTorqueCommand(ii,2);
//     // usbCan.SendTorqueCommand(ii,3);
//     usbCan.SendTorqueCommand(ii,4);

// }
// int MotorControl::robotTorStop()
// {
//     uint16_t ii = 0;
//     // usbCan.sendMultiTorqueCommand(ii,ii,ii,ii);
//     usbCan.SendTorqueCommand(0,1);
//     usbCan.SendTorqueCommand(ii,1);
// }
////////////////////////////////////////////////////////////
int MotorControl::robotStop()
{
    init_speed = 1;
    ifStop = 1;
    if(usbCan.GetMode() == 1)
    {
        // usbCan.MotorStop(1);
        // usbCan.MotorStop(2);
        // usbCan.MotorStop(3);
        // usbCan.MotorStop(4);

        usbCan.MotorClose(1);
        usbCan.MotorClose(2);
        usbCan.MotorClose(3);
        usbCan.MotorClose(4);
        usbCan.MotorClose(5);
        usbCan.MotorClose(6);
        usbCan.MotorClose(7);
        usbCan.MotorClose(8);

        std::cout << "!" << std::endl ;

        // usbCan.MotorRun(1);
        // usbCan.MotorRun(2);
        // usbCan.MotorRun(3);
        // usbCan.MotorRun(4);
        usbCan.ReadMultiCircleCommand(1);
        int32_t cangle = usbCan.ReceiveMultiCircleAngle(1);

    }
    else{
    // usbCan.SendIncrementCommand(init_speed,0,1);
    // usbCan.SendIncrementCommand(init_speed,0,2);
    // usbCan.SendIncrementCommand(init_speed,0,3);
    // usbCan.SendIncrementCommand(init_speed,0,4);

        usbCan.MotorStop(1);
        usbCan.MotorStop(2);
        usbCan.MotorStop(3);
        usbCan.MotorStop(4);
        usbCan.MotorStop(5);
        usbCan.MotorStop(6);
        usbCan.MotorStop(7);
        usbCan.MotorStop(8);

        // std::cout << "!" << std::endl ;


        // usbCan.ClearUSBCanBuffer();
        usbCan.UpdateAllMotorAngle();

        // std::cout << "!" << std::endl ;


        // int dw;
        // dw = usbCan.ReadMultiCircleCommand(1);
        // // std::cout << dw << " dw" << std::endl;
        // int32_t cangle = 0;
        // if(dw == 1)
        // {
        //         cangle = usbCan.ReceiveMultiCircleAngle(1);        
        // }        
        // usbCan.UpdateMotorCurrentAngle(cangle,1);

        // dw = usbCan.ReadMultiCircleCommand(2);
        // // std::cout << dw << " dw" << std::endl;
        
        // if(dw == 1)
        // {
        //         cangle = usbCan.ReceiveMultiCircleAngle(2);        
        // }        
        // usbCan.UpdateMotorCurrentAngle(cangle,2);
        // dw = usbCan.ReadMultiCircleCommand(3);
        // // std::cout << dw << " dw" << std::endl;
        
        // if(dw == 1)
        // {
        //         cangle = usbCan.ReceiveMultiCircleAngle(3);        
        // }        
        // usbCan.UpdateMotorCurrentAngle(cangle,3);
        // dw = usbCan.ReadMultiCircleCommand(4);
        // // std::cout << dw << " dw" << std::endl;
        
        // if(dw == 1)
        // {
        //         cangle = usbCan.ReceiveMultiCircleAngle(4);        
        // }        
        // usbCan.UpdateMotorCurrentAngle(cangle,4);



    // std::cout << cangle << "motor1 angle update" << std::endl;

        usbCan.MotorClose(1);
        usbCan.MotorClose(2);
        usbCan.MotorClose(3);
        usbCan.MotorClose(4);
        usbCan.MotorClose(5);
        usbCan.MotorClose(6);
        usbCan.MotorClose(7);
        usbCan.MotorClose(8);
    // usbCan.MotorRun(1);
    // usbCan.MotorRun(2);
    // usbCan.MotorRun(3);
    // usbCan.MotorRun(4);


    }
}

int MotorControl::robotAngleReset()
{
    usbCan.ResetMotorAngle(1);
    usbCan.ResetMotorAngle(2);
    usbCan.ResetMotorAngle(3);
    usbCan.ResetMotorAngle(4);
        
}

 int MotorControl::robotSlowStop()
{
     timeCount = 0;
    if(ifStop == 1) return 0;
    if(now_speed <= 1)
    {
        robotStop();
    }
    else if (now_speed > 1)
    {
        
        now_speed -= 1;
       // timeCount = -(log(1 - (now_speed - 1) / 4)); // lihp ADD, calculate timeCount
        if(now_speed < 1) 
        {
            now_speed = 1;
        }
        robotGearRun();
        
    }
   
}

int MotorControl::robotSlowStart()
{
    if(speedMode == 5 && now_speed < max_speed)
    {
        if(timeCount < 0.01) AcStartSpeed = now_speed;
        now_speed = AcStartSpeed+(max_speed - AcStartSpeed)*(1-exp(-timeCount));
        timeCount += 0.1;
        if(now_speed > max_speed - 0.1) 
        {
            now_speed = max_speed;
            timeCount = 0;
        }
    }
    if(speedMode == 1 && now_speed >1)
    {
        now_speed -= 0.2;
        if(now_speed < 1) now_speed = 1;
    }
}


int MotorControl::robotChangeSpeed(float speed)
{
    now_speed = speed;
    usbCan.SetSpeed(speed,wheel_1);
    usbCan.SetSpeed(speed,wheel_2);
    usbCan.SetSpeed(speed,wheel_3);
    usbCan.SetSpeed(speed,wheel_4);

}
int MotorControl::getRobotSpeed()
{
    return now_speed;
}
int MotorControl::keepTorque()
{
    // usbCan.MotorRun(wheel_1);
    // usbCan.MotorRun(wheel_2);
    // usbCan.MotorRun(wheel_3);
    // usbCan.MotorRun(wheel_4);
    usbCan.UpdateAllMotorAngle();

    usbCan.SendIncrementCommand(0,0,wheel_2);
    usbCan.SendIncrementCommand(0,0,wheel_3);
    usbCan.SendIncrementCommand(0,0,wheel_4);
}

int MotorControl::xyw2wheel_1(float x, float y, float w)
{
    /*
    float w1scale = x - y - (a_PARAMETER+b_PARAMETER)*w;
    float w2scale = x + y - (a_PARAMETER+b_PARAMETER)*w;
    float w3scale = -x - y - (a_PARAMETER+b_PARAMETER)*w;
    float w4scale = -x + y - (a_PARAMETER+b_PARAMETER)*w;
    */
   std::cout<<"x: "<<x<<"   y: "<<y<<"    w :"<<w<<std::endl;

    float w1scale =x - y - (a_PARAMETER+b_PARAMETER)*w;   
    float w2scale =x + y - (a_PARAMETER+b_PARAMETER)*w;  
    float w3scale = -x - y - (a_PARAMETER+b_PARAMETER)*w;
    float w4scale = -x + y - (a_PARAMETER+b_PARAMETER)*w;

    std::cout<<"w1 scale:  "<<w1scale<<std::endl;
    std::cout<<"w2 scale:  "<<w2scale<<std::endl;
    std::cout<<"w3 scale:  "<<w3scale<<std::endl;
    std::cout<<"w4 scale:  "<<w4scale<<std::endl;

    // float Max = max(w1scale,max(w2scale,max(w3scale,w4scale)));
    // if(Max > 1) slowSetup(speed1/Max*maxSpeed, speed2/Max*maxSpeed, speed3/Max*maxSpeed, speed4/Max*maxSpeed, flag);
    // else slowSetup(speed1*maxSpeed, speed2*maxSpeed, speed3*maxSpeed, speed4*maxSpeed, flag);
    
    
    // std::cout << w1scale << " " << w2scale << " " << w3scale << " " << w4scale << std::endl;
    // float Max = std::max(w1scale,std::max(w2scale,std::max(w3scale,w4scale)));
    // if(Max > 1.0)
    // {
    //     w1scale = w1scale/Max;
    //     w2scale = w2scale/Max;
    //     w3scale = w3scale/Max;
    //     w4scale = w4scale/Max;
    // }

    gearScale[wheel_1-1] = w1scale;
    gearScale[wheel_2-1] = w2scale;
    gearScale[wheel_3-1] = w3scale;
    gearScale[wheel_4-1] = w4scale;
    return 1;
}



//不用
int MotorControl::xyw2wheel_2(float x, float y, float w)
{
    /*
    float w1scale = x - y - (a_PARAMETER+b_PARAMETER)*w;
    float w2scale = x + y - (a_PARAMETER+b_PARAMETER)*w;
    float w3scale = -x - y - (a_PARAMETER+b_PARAMETER)*w;
    float w4scale = -x + y - (a_PARAMETER+b_PARAMETER)*w;
    */
    float w1scale = x + y + (a_PARAMETER+b_PARAMETER)*w;
    float w2scale = x - y + (a_PARAMETER+b_PARAMETER)*w;
    float w3scale = -x + y + (a_PARAMETER+b_PARAMETER)*w;
    float w4scale = -x - y + (a_PARAMETER+b_PARAMETER)*w;

    // float Max = std::max(w1scale,std::max(w2scale,std::max(w3scale,w4scale)));
    // if(Max > 1.0)
    // {
    //     w1scale = w1scale/Max;
    //     w2scale = w2scale/Max;
    //     w3scale = w3scale/Max;
    //     w4scale = w4scale/Max;
    // }
    // if(Max > 1) slowSetup(speed1/Max*maxSpeed, speed2/Max*maxSpeed, speed3/Max*maxSpeed, speed4/Max*maxSpeed, flag);
    // else slowSetup(speed1*maxSpeed, speed2*maxSpeed, speed3*maxSpeed, speed4*maxSpeed, flag);
    
    
    // std::cout << w1scale << " " << w2scale << " " << w3scale << " " << w4scale << std::endl;

    gearScale[wheel_1-1] = w1scale;
    gearScale[wheel_2-1] = w2scale;
    gearScale[wheel_3-1] = w3scale;
    gearScale[wheel_4-1] = w4scale;
    return 1;
}

int MotorControl::robotGearRun()
{
    ifStop = 0;
    float fw1 = gearScale[wheel_1-1]*100.0;
    float fw2 = gearScale[wheel_2-1]*100.0;
    float fw3 = gearScale[wheel_3-1]*100.0;
    float fw4 = gearScale[wheel_4-1]*100.0;
    //std::cout << abs(fw1*now_speed) << " now speed " << std::endl;
    //std::cout << usbCan.UpdateSingleMotorState(1) << " re speed " << std::endl;
    //std::cout << fw1*now_speed << " fw1 * max" << fw2*now_speed << " " << fw3*now_speed << " " << fw4*now_speed << std::endl;
    // usbCan.testangle += fw1*angle;
    // std::cout << usbCan.getAngle(1) << "angle in list " << std::endl;
    // std::cout << usbCan.UpdateSingleMotorAngle(1) << " now angle " << std::endl;
    // std::cout << usbCan.testangle << " real angle " << std::endl;
    usbCan.SendGearIncrementCommand(abs(fw1*now_speed),fw1*angle,wheel_1);
    usbCan.SendGearIncrementCommand(abs(fw2*now_speed),fw2*angle,wheel_2);
    usbCan.SendGearIncrementCommand(abs(fw3*now_speed),fw3*angle,wheel_3);
    usbCan.SendGearIncrementCommand(abs(fw4*now_speed),fw4*angle,wheel_4);
     
    
    // std::cout << "fw1 " << fw1 << std::endl;
    // std::cout << "fw2 " << fw2 << std::endl;
    // std::cout << "fw3 " << fw3 << std::endl;
    // std::cout << "fw4 " << fw4 << std::endl;
    /*std::cout << "angle " << angle << std::endl;
    int nowi1 = usbCan.UpdateSingleMotorState(1);
    if(abs(nowi1) > maxi1)
    {
       maxi1 = nowi1;
    }
    
    
    int nowi2 = usbCan.UpdateSingleMotorState(2);
    if(abs(nowi2) > maxi1)
    {
       maxi1 = nowi2;
    }
    
    int nowi3 = usbCan.UpdateSingleMotorState(3);
    if(abs(nowi3) > maxi1)
    {
       maxi1 = nowi3;
    }
    
    int nowi4 = usbCan.UpdateSingleMotorState(4);
    if(abs(nowi4) > maxi1)
    {
       maxi1 = nowi4;
    }
    
    

    float i1 = ((float) maxi1 * 33)/2048;*/
    //std::cout << i1 << " i1 " << std::endl;
    

    return 1;
}

int MotorControl::rotate180(){
    ifStop = 0;
    int  _angle = 450000;
    float fw1 = gearScale[wheel_1-1]*100.0;
    float fw2 = gearScale[wheel_2-1]*100.0;
    float fw3 = gearScale[wheel_3-1]*100.0;
    float fw4 = gearScale[wheel_4-1]*100.0;
    std::cout<<"原地旋转180°, angle: "<<_angle<<std::endl;
    usbCan.SendGearIncrementCommand(abs(fw1*now_speed),_angle,wheel_1);
    usbCan.SendGearIncrementCommand(abs(fw2*now_speed),_angle,wheel_2);
    usbCan.SendGearIncrementCommand(abs(fw3*now_speed),_angle,wheel_3);
    usbCan.SendGearIncrementCommand(abs(fw4*now_speed),_angle,wheel_4);
    return 1;
}

int MotorControl::foward_1cm(){
    ifStop = 0;
    int  _angle = 5000;
    float fw1 = gearScale[wheel_1-1]*100.0;
    float fw2 = gearScale[wheel_2-1]*100.0;
    float fw3 = gearScale[wheel_3-1]*100.0;
    float fw4 = gearScale[wheel_4-1]*100.0;
    std::cout<<"前进1cm, angle: "<<_angle<<std::endl;
    usbCan.SendGearIncrementCommand(abs(fw1*now_speed),_angle*(-1),wheel_1);
    usbCan.SendGearIncrementCommand(abs(fw2*now_speed),_angle,wheel_2);
    usbCan.SendGearIncrementCommand(abs(fw3*now_speed),_angle*(-1),wheel_3);
    usbCan.SendGearIncrementCommand(abs(fw4*now_speed),_angle,wheel_4);
    return 1;
}

int MotorControl::backward_1cm(){
    ifStop = 0;
    int  _angle = 5000;
    float fw1 = gearScale[wheel_1-1]*100.0;
    float fw2 = gearScale[wheel_2-1]*100.0;
    float fw3 = gearScale[wheel_3-1]*100.0;
    float fw4 = gearScale[wheel_4-1]*100.0;
    std::cout<<"后退1cm, angle: "<<_angle<<std::endl;
    usbCan.SendGearIncrementCommand(abs(fw1*now_speed),_angle,wheel_1);
    usbCan.SendGearIncrementCommand(abs(fw2*now_speed),_angle*(-1),wheel_2);
    usbCan.SendGearIncrementCommand(abs(fw3*now_speed),_angle,wheel_3);
    usbCan.SendGearIncrementCommand(abs(fw4*now_speed),_angle*(-1),wheel_4);
    return 1;
}

int MotorControl::left_1cm(){
    ifStop = 0;
    int  _angle = 5000;
    float fw1 = gearScale[wheel_1-1]*100.0;
    float fw2 = gearScale[wheel_2-1]*100.0;
    float fw3 = gearScale[wheel_3-1]*100.0;
    float fw4 = gearScale[wheel_4-1]*100.0;
    std::cout<<"向左1cm, angle: "<<_angle<<std::endl;
    usbCan.SendGearIncrementCommand(abs(fw1*now_speed),_angle,wheel_1);
    usbCan.SendGearIncrementCommand(abs(fw2*now_speed),_angle,wheel_2);
    usbCan.SendGearIncrementCommand(abs(fw3*now_speed),_angle*(-1),wheel_3);
    usbCan.SendGearIncrementCommand(abs(fw4*now_speed),_angle*(-1),wheel_4);
    return 1;
}

int MotorControl::right_1cm(){
    ifStop = 0;
    int  _angle = 5000;
    float fw1 = gearScale[wheel_1-1]*100.0;
    float fw2 = gearScale[wheel_2-1]*100.0;
    float fw3 = gearScale[wheel_3-1]*100.0;
    float fw4 = gearScale[wheel_4-1]*100.0;
    std::cout<<"向右1cm, angle: "<<_angle<<std::endl;
    usbCan.SendGearIncrementCommand(abs(fw1*now_speed),_angle*(-1),wheel_1);
    usbCan.SendGearIncrementCommand(abs(fw2*now_speed),_angle*(-1),wheel_2);
    usbCan.SendGearIncrementCommand(abs(fw3*now_speed),_angle,wheel_3);
    usbCan.SendGearIncrementCommand(abs(fw4*now_speed),_angle,wheel_4);
    return 1;
}



int MotorControl::robotGearRun_test(int _angle1, int _angle2, int _angle3, int _angle4){
    ifStop = 0;
    float fw1 = gearScale[wheel_1-1]*100.0;
    float fw2 = gearScale[wheel_2-1]*100.0;
    float fw3 = gearScale[wheel_3-1]*100.0;
    float fw4 = gearScale[wheel_4-1]*100.0;
    std::cout<<"轮子1 angle: "<<_angle1<<std::endl;
    std::cout<<"轮子2 angle: "<<_angle2<<std::endl;
    std::cout<<"轮子3 angle: "<<_angle3<<std::endl;
    std::cout<<"轮子4 angle: "<<_angle4<<std::endl;
    usbCan.SendGearIncrementCommand(abs(fw1*now_speed),_angle1,wheel_1);
    usbCan.SendGearIncrementCommand(abs(fw2*now_speed),_angle2,wheel_2);
    usbCan.SendGearIncrementCommand(abs(fw3*now_speed),_angle3,wheel_3);
    usbCan.SendGearIncrementCommand(abs(fw4*now_speed),_angle4,wheel_4);
    return 1;

}


int MotorControl::WriteAngleTest()
{
    usbCan.WriteAngle();
    return 0;
}

int MotorControl::testPoseCode(int speed3, int speed5, int speed6, int angle3, int angle5, int angle6)
{
    
    speed3 = 2000;
    speed5 = 1000;
    speed6 = 1605;
    angle3 = 29500;
    angle5 = 29700;
    angle6 = 0;

    usbCan.SendGearIncrementCommand(speed3,angle3*100,3);
    usbCan.SendGearIncrementCommand(speed3,angle3*100,4);
    usbCan.SendGearIncrementCommand(speed5,angle5*100,5);
    usbCan.SendGearIncrementCommand(speed6,angle6*100,6);
}
int MotorControl::poseA2B_1()
{
    int speed3 = 500;
    int speed5 = 500;
    int speed6 = 500;
    int angle3 = 6000;
    int angle5 = 6000;
    int angle6 = 6000;
       usbCan.SendGearIncrementCommand(speed3,angle3*100,3);
       usbCan.SendGearIncrementCommand(speed3,angle3*100,4);
       usbCan.SendGearIncrementCommand(speed5,angle5*100,5);
       usbCan.SendGearIncrementCommand(speed6,angle6*100,6);
}

int MotorControl::poseA2B_2()
{
       int speed3 = 2315;
       int speed5 = 500;
       int speed6 = 470;
       int angle3 = 47220;
       int angle5 = 10200;
       int angle6 = 9600;
       usbCan.SendGearIncrementCommand(speed3,angle3*100,3);
       usbCan.SendGearIncrementCommand(speed3,angle3*100,4);
       usbCan.SendGearIncrementCommand(speed5,angle5*100,5);
       usbCan.SendGearIncrementCommand(speed6,angle6*100,6);
}

int MotorControl::poseB2A()
{
       int speed3 = 1500;
       int speed5 = 500;
       int speed6 = 480;
       int angle3 = -53220;
       int angle5 = -16200;
       int angle6 = -15600;
       usbCan.SendGearIncrementCommand(speed3,angle3*100,3);
       usbCan.SendGearIncrementCommand(speed3,angle3*100,4);
       usbCan.SendGearIncrementCommand(speed5,angle5*100,5);
       usbCan.SendGearIncrementCommand(speed6,angle6*100,6);
}

int MotorControl::poseB2C_1()
{
    int speed3 = 800;
    int speed5 = 1000;
    int speed6 = 1605;
    int angle3 = 13548;
    int angle5 = -18684;
    int angle6 = 30000;

    usbCan.SendGearIncrementCommand(speed3,angle3*100,3);
    usbCan.SendGearIncrementCommand(speed3,angle3*100,4);
    usbCan.SendGearIncrementCommand(speed5,angle5*100,5);
    usbCan.SendGearIncrementCommand(speed6,angle6*100,6);
}

int MotorControl::poseB2C_2()
{
    int speed3 = 2000;
    int speed5 = 1000;
    int speed6 = 1605;
    int angle3 = 29500;
    int angle5 = 29700;
    int angle6 = 0;

    usbCan.SendGearIncrementCommand(speed3,angle3*100,3);
    usbCan.SendGearIncrementCommand(speed3,angle3*100,4);
    usbCan.SendGearIncrementCommand(speed5,angle5*100,5);
    usbCan.SendGearIncrementCommand(speed6,angle6*100,6);
}
