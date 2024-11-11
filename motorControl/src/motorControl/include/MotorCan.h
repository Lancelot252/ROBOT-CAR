#ifndef _MOTORCAN_H_
#define _MOTORCAN_H_

#include "controlcan.h"
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string>
#include <iosfwd>
#include <jsoncpp/json/json.h>
#include "timer.h"
#include <thread>

class MotorCan
{
protected:
    /* data */
    long long _current_angle_count[8] = {0,0,0,0,0,0,0,0};
    int16_t _current_state[8][8] = {
        {0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,0,0}
    };

    VCI_CAN_OBJ vco_send[8]; //8 bytes data for send
    VCI_CAN_OBJ vco_receive[64];//64
    VCI_CAN_OBJ vco_receiveCar[64];

    int nDeviceType;//3->usbcan-I,4->usbcan-II
    int nDeviceInd; //(can232中, 0:com1, 1:com2)
    int nCANInd; // can channel index (0,1)

    uint16_t nMotorspeed;
    // int nDeviceTypeCar;
    // int nDeviceIndCar;
    
    int mode = 0;//1: torque ,0: angle
    // const int slowchangelength = 5;

    VCI_INIT_CONFIG vic; // 初始化参数

public:
    
    static helper::TimerServer timer_;
    int testangle = 0;
    MotorCan();
    MotorCan(int canID);
    ~MotorCan();
    /*can init*/
    int StartUSBCan();
    int StartDevice();
    int CloseDevice();

    int SlowStart();
    int SlowStop();

    int ChangeMode(int m);
    int GetMode();
    int UpdateSingleMotorState(uint8_t motorID);
    int UpdateMotorCurrentAngle(int32_t cangle,uint8_t motorID);
    int UpdateAllMotorAngle();
    int UpdateSingleMotorAngle(uint8_t motorID);

    /*command*/
    int SendMultCircleCommand(int32_t sendangle,uint8_t motorID);//位置指令
    int SendIncrementCommand(uint16_t sendspeed, int32_t sendangle,uint8_t motorID);//增量控制指令

    int SendTorqueCommand(int16_t sendiqControl,uint8_t motorID);
    int sendMultiTorqueCommand(int16_t sendiqControl_1,int16_t sendiqControl_2,int16_t sendiqControl_3,int16_t sendiqControl_4);

    int SetSpeed(int32_t sendspeed,uint8_t motorID);
    int GetSpeed();
    int ReadMultiCircleCommand(uint8_t motorID);
    int ReceiveMultiCircleAngle(uint8_t motorID);//每次receive之前要先read发送查询指令
    /*read error state*/
    int SendReadErrorState(uint8_t motorID);
    int ReceiveReadErrorState(uint8_t motorID);
    /*clear error state*/
    int SendClearErrorState(uint8_t motorID);
    int ReceiveClearErrorState(uint8_t motorID);

    int MotorStop(uint8_t motorID);
    int MotorClose(uint8_t motorID);
    int MotorRun(uint8_t motorID);
    int ResetMotorAngle(uint8_t motorID);

    
    int ClearUSBCanBuffer();

    int SendGearIncrementCommand(uint16_t sendspeed,int32_t sendangle,uint8_t motorID);
    int16_t ReceiveMotorState(uint8_t motorID);

    int WriteAngle();
    int ReadAngle();
    int readMotorState(uint8_t motorID);
    long long getAngle(uint8_t motorID);
    int ResetAngle(uint8_t motorID);
    int getState(uint8_t motorID);
    void thread_getState(uint8_t motorID);
    void setTorqueMode();
    float getStateIncurrent(uint8_t motorID);
    int getStateSpeed(uint8_t motorID);
    int getStateEncode(uint8_t motorID);
};





#endif
