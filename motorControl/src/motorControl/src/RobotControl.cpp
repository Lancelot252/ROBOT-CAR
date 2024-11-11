// 

#include "RobotControl.h"
#include "MotorCan.h"
#include "unistd.h"

RobotControl::RobotControl()
:usbCan(0)
{
    angle = 100000000;
    ntorque = 900;
    motor_speed = 15;
    motor_max_speed = 1;
    usbCan.StartDevice();
    for(uint8_t i = 0;i < 8;i++)
    {
        usbCan.MotorClose(i+1);
    }

}


RobotControl::RobotControl(int canID)
:usbCan(canID)
{
    //初始化物理参数
    angle = 100000000;
    ntorque = 900;
    motor_speed = 15;
    motor_max_speed = 1;
    if(canID == 0)
    {
        angle = 100000000;
        motor_speed = 15;
        usbCan.StartUSBCan();
    }
    usbCan.StartDevice();
    for(uint8_t i = 0;i < 8;i++)
    {
        usbCan.MotorClose(i+1);
    }

}

RobotControl::RobotControl(MotorCan *Can)
{
    usbCan = *Can;
    angle = 100000000;
    ntorque = 900;
    motor_speed = 15;
    motor_max_speed = 1;
    for(uint8_t i = 0;i < 8;i++)
    {
        usbCan.MotorClose(i+1);
    }

}

RobotControl::~RobotControl()
{
    usbCan.CloseDevice();
}
int RobotControl::motorFoward(uint8_t motorID)
{
    if(usbCan.GetMode() == 1)
    {
        usbCan.SendTorqueCommand(ntorque,motorID);
    }
    else{
     usbCan.SendIncrementCommand(motor_speed,angle,motorID);
    }
    //std::cout << "track speed " << motor_speed << std::endl;
}

int RobotControl::motorFoward(uint8_t motorID,uint16_t speedOrTorque)
{
    //std::cout << "motorFoward speed: " << speedOrTorque << "\n";
    if(usbCan.GetMode() == 1)
    {
        usbCan.SendTorqueCommand(speedOrTorque,motorID);
    }
    else{
     usbCan.SendIncrementCommand(speedOrTorque,angle,motorID);
    }
    //std::cout << "track speed " << motor_speed << std::endl;
}

int RobotControl::motorFoward(uint8_t motorID,uint16_t speedOrTorque,int32_t cangle)
{
    if(usbCan.GetMode() == 1)
    {
        usbCan.SendTorqueCommand(speedOrTorque,motorID);
    }
    else{
        usbCan.SendGearIncrementCommand(speedOrTorque,cangle,motorID);
    }
}
int RobotControl::motorback(uint8_t motorID)
{
    if(usbCan.GetMode() == 1)
    {
        usbCan.SendTorqueCommand(-ntorque,motorID);
    }
    else{
        usbCan.SendIncrementCommand(motor_speed,-angle,motorID);
    }
    //std::cout << "track speed " << motor_speed << std::endl;
}

int RobotControl::motorback(uint8_t motorID,uint16_t speedOrTorque)
{
    if(usbCan.GetMode() == 1)
    {
        usbCan.SendTorqueCommand(-speedOrTorque,motorID);
    }
    else{
        usbCan.SendIncrementCommand(speedOrTorque,-angle,motorID);
    }
    //std::cout << "track speed " << motor_speed << std::endl;
}

int RobotControl::motorback(uint8_t motorID,uint16_t speedOrTorque,int32_t cangle)
{
    if(usbCan.GetMode() == 1)
    {
        usbCan.SendTorqueCommand(-speedOrTorque,motorID);
    }
    else{
        usbCan.SendGearIncrementCommand(speedOrTorque,-cangle,motorID);
    }
}
int RobotControl::motorSetSpeed(uint16_t speed)
{
    if(usbCan.GetMode() == 1)
    {
        ntorque = speed;
        std::cout << "setntorque:" << ntorque <<"  ---RobotControl::motorSetSpeed"<< std::endl; 
    }
    else
    {
        motor_speed = speed;
        std::cout << "setspeed:" << motor_speed <<"  ---RobotControl::motorSetSpeed"<< std::endl;   
    }
}
int RobotControl::motorSetAngle(int32_t cangle)
{
    angle = cangle;
    std::cout << "setAngle:" << cangle << std::endl;
}

long long RobotControl::motorReadAngle(uint8_t motorID){
    return usbCan.getAngle(motorID);
}

int RobotControl::readErrorState(uint8_t motorID){
    int dwRel{-1};
    int error_state{-1};
    dwRel = usbCan.SendReadErrorState(motorID);
    if (1 == dwRel){
        error_state = usbCan.ReceiveReadErrorState(motorID);
        return error_state;
    }
    else{
        std::cout << "Send readErrorState Wrong, dwRel: " << dwRel << "\n";
    }
    return dwRel;
}

int RobotControl::clearErrorState(uint8_t motorID){
    int dwRel{-1};
    int error_state{-1};
    dwRel = usbCan.SendClearErrorState(motorID);
    if(1 == dwRel){
        error_state = usbCan.ReceiveClearErrorState(motorID);
        return error_state;
    }
    else{
        std::cout << "Send clearErrorState Wrong, dwRel: " << dwRel << "\n"; 
    }
    return dwRel;
}

uint16_t RobotControl::getMotrSpeed()
{
    return motor_speed;
}
int RobotControl::motorStop()
{
    usbCan.MotorStop(motor_1);
    usbCan.MotorStop(motor_2);
    usbCan.MotorStop(motor_3);
    usbCan.MotorStop(motor_4);
    usbCan.MotorStop(motor_5);
    usbCan.MotorStop(motor_6);
    usbCan.MotorStop(motor_7);
    usbCan.MotorStop(motor_8);

    usbCan.UpdateAllMotorAngle();

    usbCan.MotorClose(motor_1);
    usbCan.MotorClose(motor_2);
    usbCan.MotorClose(motor_3);
    usbCan.MotorClose(motor_4);
    usbCan.MotorClose(motor_5);
    usbCan.MotorClose(motor_6);
    usbCan.MotorClose(motor_7);
    usbCan.MotorClose(motor_8);
}
int RobotControl::motorStop(uint8_t motorID)
{
    usbCan.MotorStop(motorID);
}
int RobotControl::motorClose(uint8_t motorID)
{
    usbCan.MotorClose(motorID);
}
int RobotControl::motorUpdateAll()
{
    usbCan.UpdateAllMotorAngle();
}
int RobotControl::motorUpdateSingle(uint8_t motorID)
{
    usbCan.UpdateSingleMotorAngle(motorID);
}

int RobotControl::motorLock(uint8_t motorID){
    usbCan.SendIncrementCommand(0,0,motorID);
}

int RobotControl::motorUnlock(uint8_t motorID){
      usbCan.MotorStop(motorID);
      usbCan.UpdateSingleMotorAngle(motorID);
      usbCan.MotorClose(motorID);
}

int RobotControl::motorAllLock()
{
    usbCan.SendIncrementCommand(0,0,motor_1);
    usbCan.SendIncrementCommand(0,0,motor_2);
    usbCan.SendIncrementCommand(0,0,motor_3);
    usbCan.SendIncrementCommand(0,0,motor_4);
    usbCan.SendIncrementCommand(0,0,motor_5);
    usbCan.SendIncrementCommand(0,0,motor_6);
    usbCan.SendIncrementCommand(0,0,motor_7);
    usbCan.SendIncrementCommand(0,0,motor_8);
}

int RobotControl::motorCarUnlock()
{
    usbCan.MotorStop(motor_1);
    usbCan.MotorStop(motor_2);
    usbCan.MotorStop(motor_3);
    usbCan.MotorStop(motor_4);
    usbCan.UpdateAllMotorAngle();
    usbCan.MotorClose(motor_1);
    usbCan.MotorClose(motor_2);
    usbCan.MotorClose(motor_3);
    usbCan.MotorClose(motor_4);
}
int RobotControl::motorCarLock()
{
    usbCan.UpdateAllMotorAngle();
    usbCan.SendIncrementCommand(0,0,motor_1);
    usbCan.SendIncrementCommand(0,0,motor_2);
    usbCan.SendIncrementCommand(0,0,motor_3);
    usbCan.SendIncrementCommand(0,0,motor_4);

}
int RobotControl::motorCar5Lock()
{
    usbCan.SendIncrementCommand(0,0,motor_5);
}

int RobotControl::motorReadAllAngle()
{

}

// int RobotControl::motorSetAngle(int32_t cangle)
// {

// }

MotorCan* RobotControl::getUsbCan()
{
    return &usbCan;
}

