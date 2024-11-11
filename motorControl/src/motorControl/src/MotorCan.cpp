#include "MotorCan.h"
#include <iostream>
#include <fstream>
#include <string>
#include <iosfwd>
#include <jsoncpp/json/json.h>
#include <thread>
#include <unistd.h>
#include <string>
using namespace std;

helper::TimerServer MotorCan::timer_;

void ReadMotorStatePeriod(MotorCan* ptr, uint8_t motorID){
    ptr->readMotorState(motorID);
}


MotorCan::MotorCan()
{
    // _current_angle_count ;
    nDeviceType = 4;
    nDeviceInd = 0;
    nCANInd = 1;
    nMotorspeed = 10*100;


    /*can初始化*/
    // vic.AccCode = 0x80000008;
    vic.AccCode = 0x00000000;
    vic.AccMask = 0xFFFFFFFF;
    vic.Filter = 0;  //接受所有帧
    vic.Timing0 = 0x00;
    vic.Timing1 = 0x14;//波特率1000Kbps
    vic.Mode = 0;     //正常模式
    //timer_.Start();

}
MotorCan::MotorCan(int canID)
{
    // _current_angle_count ;
    nDeviceType = 4;
    nDeviceInd = 0;
    nCANInd = canID;
    nMotorspeed = 10*100;
    /*can初始化*/
    // vic.AccCode = 0x80000008;
    vic.AccCode = 0x00000000;
    vic.AccMask = 0xFFFFFFFF;
    vic.Filter = 0;  //接受所有帧
    vic.Timing0 = 0x00;
    vic.Timing1 = 0x14;//波特率1000Kbps
    vic.Mode = 0;     //正常模式
    //timer_.Start();
    //timer_.SetTimer(10*1000,std::bind(ReadMotorStatePeriod,this,canID));
}



MotorCan::~MotorCan()
{
    //timer_.Stop();
    CloseDevice();
}

int MotorCan::ChangeMode(int m )
{
    mode = m;
}
int MotorCan::GetMode()
{
    return mode;
}

int MotorCan::StartUSBCan()
{
    int dwRelOpenDevice;
    int dwRelVCI_InitCAN;
    dwRelOpenDevice = VCI_OpenDevice(nDeviceType, nDeviceInd, 0);
    if (dwRelOpenDevice != 1) {
        cout << "VCI_OpenDevice fail! " << endl;
        std::cout << "dwRelOpenDevice: " << dwRelOpenDevice << "\n";
        return -1;
    }
}
int MotorCan::StartDevice()
{
    int dwRelOpenDevice;
    int dwRelVCI_InitCAN;

    dwRelVCI_InitCAN = VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd, &vic);
    std::cout << "dwRelVCI_InitCAN: "<< dwRelVCI_InitCAN  << std::endl;
    if (dwRelVCI_InitCAN != 1) {
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        cout << "dwRelVCI_InitCAN fail!" << endl;
        std::cout << "dwRelVCI_InitCAN: "<< dwRelVCI_InitCAN  << std::endl;
        return -1;
    }

    if (VCI_StartCAN(nDeviceType, nDeviceInd, nCANInd) != 1) {
        VCI_CloseDevice(nDeviceType, nDeviceInd);
        cout << "VCI_StartCAN 1 fail!!" << endl;
        return -1;
    }

    cout << "Device_Open" << endl;
    VCI_ClearBuffer(nDeviceType, nDeviceInd, nCANInd);
    return 1;
}

int MotorCan::CloseDevice() {
    int dwRel;
    dwRel = VCI_CloseDevice(nDeviceType, nDeviceInd);
    if (dwRel != 1) {
        cerr << "CAN Close Errors! " << endl;
        return -1;
    }
    cout << "Device_Close" << endl;
    return 1;
}

int MotorCan::SendMultCircleCommand(int32_t sendangle=0,uint8_t motorID=1)
{
    uint16_t speed = nMotorspeed;
    UpdateSingleMotorAngle(motorID);
    int32_t angle = sendangle*100;
    int dwRel;

        /*send指令初始化*/
    vco_send[0].ID = (UINT)(0x00000140+motorID);
    vco_send[0].RemoteFlag = 0;
    vco_send[0].ExternFlag = 0; //扩展帧
    vco_send[0].DataLen = 8;

    BYTE b0 = vco_send[0].Data[0] = 0xA4;
    BYTE b1 = vco_send[0].Data[1] = 0x00;
    BYTE b2 = vco_send[0].Data[2] = *(uint8_t*)(&speed);
    BYTE b3 = vco_send[0].Data[3] = *((uint8_t*)(&speed)+1);

    BYTE b4 = vco_send[0].Data[4] = *(uint8_t*)(&angle);
    BYTE b5 = vco_send[0].Data[5] = *((uint8_t*)(&angle)+1);
    BYTE b6 = vco_send[0].Data[6] = *((uint8_t*)(&angle)+2);
    BYTE b7 = vco_send[0].Data[7] = *((uint8_t*)(&angle)+3);

    dwRel = VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco_send, 1);
    //cout << "SendSteeringCommand" << endl;
    //_current_angle_count[motorID-1] = angle;//todo:发送成功再更新
    return 1;
}

int MotorCan::SendIncrementCommand(uint16_t sendspeed,int32_t sendangle,uint8_t motorID)
{
    // cout << _current_angle_count[motorID-1] << " : " << int(motorID) << " ";
    UpdateSingleMotorAngle(motorID);
    uint16_t speed = sendspeed*100;
    // ReadMultiCircleCommand(motorID);
    // int32_t angle= ReceiveMultiCircleAngle(motorID)+sendangle*100;
    int32_t angle = _current_angle_count[motorID-1] + sendangle*100;
    // cout << angle << endl;
    int dwRel;

   // std::cout<<"轮子"<<motorID<<"的angle: "<<angle<<std::endl;


    /*send指令初始化*/
    vco_send[0].ID = (UINT)(0x00000140+motorID);
    vco_send[0].RemoteFlag = 0;
    vco_send[0].ExternFlag = 0; //扩展帧
    vco_send[0].DataLen = 8;
    BYTE b0 = vco_send[0].Data[0] = 0xA4;
    BYTE b1 = vco_send[0].Data[1] = 0x00;
    BYTE b2 = vco_send[0].Data[2] = *(uint8_t*)(&speed);
    BYTE b3 = vco_send[0].Data[3] = *((uint8_t*)(&speed)+1);

    BYTE b4 = vco_send[0].Data[4] = *(uint8_t*)(&angle);
    BYTE b5 = vco_send[0].Data[5] = *((uint8_t*)(&angle)+1);
    BYTE b6 = vco_send[0].Data[6] = *((uint8_t*)(&angle)+2);
    BYTE b7 = vco_send[0].Data[7] = *((uint8_t*)(&angle)+3);

    dwRel = VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco_send, 1);
    cout<<"speed:  "<<speed<<"   ----MotorCan::SendIncrementCommand"<<endl;
    //getState(motorID);
    //cout << "SendSteeringCommand" << endl;
    //cout<<(int)motorID<<endl;
    //_current_angle_count[motorID-1] = angle;//todo:发送成功再更新
    return 1;
}

int MotorCan::SetSpeed(int32_t sendspeed,uint8_t motorID)
//协议为uint32
{
    //std::cout  << "MOTORCAN:: SetSpeed" << "\n";
    int32_t speed = sendspeed;

    std::cout << "speed: " << speed << " motorID: " << (int)motorID << "\n";
    int dwRel;
    /*send指令初始化*/
    vco_send[0].ID = (UINT)(0x00000140+motorID);
    vco_send[0].RemoteFlag = 0;
    vco_send[0].ExternFlag = 0; //扩展帧
    vco_send[0].DataLen = 8;

    BYTE b0 = vco_send[0].Data[0] = 0xA2;
    BYTE b1 = vco_send[0].Data[1] = 0x00;
    BYTE b2 = vco_send[0].Data[2] = 0x00;
    BYTE b3 = vco_send[0].Data[3] = 0x00;

    BYTE b4 = vco_send[0].Data[4] = *(uint8_t*)(&speed);
    BYTE b5 = vco_send[0].Data[5] = *((uint8_t*)(&speed)+1);
    BYTE b6 = vco_send[0].Data[6] = *((uint8_t*)(&speed)+2);
    BYTE b7 = vco_send[0].Data[7] = *((uint8_t*)(&speed)+3);

    dwRel = VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco_send, 1);
    std::cout << "dwRel: " << dwRel << "\n";
    nMotorspeed = speed;
    return 1;
}
int MotorCan::GetSpeed()
{
    return nMotorspeed;
}

int MotorCan::SendTorqueCommand(int16_t sendiqControl,uint8_t motorID)
{
    int dwRel;


    /*send指令初始化*/
    vco_send[0].ID = (UINT)(0x00000140+motorID);
    vco_send[0].RemoteFlag = 0;
    vco_send[0].ExternFlag = 0; //扩展帧
    vco_send[0].DataLen = 8;
    BYTE b0 = vco_send[0].Data[0] = 0xA1;
    BYTE b1 = vco_send[0].Data[1] = 0x00;
    BYTE b2 = vco_send[0].Data[2] = 0x00;
    BYTE b3 = vco_send[0].Data[3] = 0x00;

    BYTE b4 = vco_send[0].Data[4] = *(uint8_t*)(&sendiqControl);
    BYTE b5 = vco_send[0].Data[5] = *((uint8_t*)(&sendiqControl)+1);
    BYTE b6 = vco_send[0].Data[6] = 0x00;
    BYTE b7 = vco_send[0].Data[7] = 0x00;

    dwRel = VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco_send, 1);
    cout<<"Torque_i :  "<<sendiqControl * 32 / 2000<<"  ---MotorCan::SendTorqueCommand"<<endl;
    getState(motorID);
    // cout << "SendTorqueCommand" << endl;
    //这些需要读取编码器值,然后更新当前角度变量
    return 1;
}

int MotorCan::sendMultiTorqueCommand(int16_t sendiqControl_1,int16_t sendiqControl_2,int16_t sendiqControl_3,int16_t sendiqControl_4)
{
    int dwRel;
    /*send指令初始化*/
    vco_send[0].ID = (UINT)(0x00000280);
    vco_send[0].RemoteFlag = 0;
    vco_send[0].ExternFlag = 0; //扩展帧
    vco_send[0].DataLen = 8;
    BYTE b0 = vco_send[0].Data[0] = *(uint8_t*)(&sendiqControl_1);
    BYTE b1 = vco_send[0].Data[1] = *((uint8_t*)(&sendiqControl_1)+1);
    BYTE b2 = vco_send[0].Data[2] = *(uint8_t*)(&sendiqControl_2);
    BYTE b3 = vco_send[0].Data[3] = *((uint8_t*)(&sendiqControl_2)+1);

    BYTE b4 = vco_send[0].Data[4] = *(uint8_t*)(&sendiqControl_3);
    BYTE b5 = vco_send[0].Data[5] = *((uint8_t*)(&sendiqControl_3)+1);
    BYTE b6 = vco_send[0].Data[6] = *(uint8_t*)(&sendiqControl_4);
    BYTE b7 = vco_send[0].Data[7] = *((uint8_t*)(&sendiqControl_4)+1);

    dwRel = VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco_send, 1);
    //cout << "SendSteeringCommand" << endl;
    //这些需要读取编码器值,然后更新当前角度变量
    return 1;
}



int MotorCan::ReadMultiCircleCommand(uint8_t motorID)
{
    int dwRel;


    /*send指令初始化*/
    vco_send[0].ID = (UINT)(0x00000140+motorID);
    vco_send[0].RemoteFlag = 0;
    vco_send[0].ExternFlag = 0; //扩展帧
    vco_send[0].DataLen = 8;
    BYTE b0 = vco_send[0].Data[0] = 0x92;
    BYTE b1 = vco_send[0].Data[1] = 0x00;
    BYTE b2 = vco_send[0].Data[2] = 0x00;
    BYTE b3 = vco_send[0].Data[3] = 0x00;

    BYTE b4 = vco_send[0].Data[4] = 0x00;
    BYTE b5 = vco_send[0].Data[5] = 0x00;
    BYTE b6 = vco_send[0].Data[6] = 0x00;
    BYTE b7 = vco_send[0].Data[7] = 0x00;

    dwRel = VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco_send, 1);
    //cout << "SendSteeringCommand" << endl;
    //这些需要读取编码器值,然后更新当前角度变量
    return dwRel;
}

int MotorCan::ReceiveMultiCircleAngle(uint8_t motorID) {
    int dwRel;
    int i =0;
    while(i++ < 500) //把循环次数设置多一点 100 -> 500
    {
	ReadMultiCircleCommand(motorID);
        dwRel = VCI_Receive(nDeviceType, nDeviceInd, nCANInd, vco_receive, 64, 400);//设置等待超时时间

        int current_steeringangle = 0;
        // std::cout << vco_receive[0].ID-0x140 << " id" << std::endl;
        if (dwRel > 0 && vco_receive[0].ID == (UINT)(0x140+motorID)) {
            BYTE id = (BYTE)(vco_receive[0].Data[0]);
            // cout << (int)id << endl;
            if(id == (0x92))
            {
                // std::cout << "id == 0x92" << std::endl;
                // std::cout << "data0 " << vco_receive[0].Data[1] << std::endl;
                current_steeringangle += (int)vco_receive[0].Data[1];
                current_steeringangle += ((int)vco_receive[0].Data[2]<<8);
                current_steeringangle += ((int)vco_receive[0].Data[3]<<16);
                current_steeringangle += ((int)vco_receive[0].Data[4]<<24);
                // current_steeringangle += ((int)vco_receive[0].Data[5]<<32);
                // current_steeringangle += ((int)vco_receive[0].Data[6]<<40);
                // current_steeringangle += ((int)vco_receive[0].Data[7]<<48);
            }

            // BYTE low = (BYTE)(vco_receive[0].Data[1]);
            // temp_angle = ((int)high << 8) + (int)low;

            // std::cout << "current_steeringangle = " << current_steeringangle << endl;
            // std::cout << "_current_steeringangle = " << _current_angle_count[motorID-1] << endl;
            //std::cout << "current_steering_angle_speed = " << steering_feedback.steering_angle_speed << endl;
            if(id == (0x92))
            {
                //cout<<current_steeringangle<<"     "<<(int)motorID<<endl;
                return current_steeringangle;                
            }


        } 
        // else if (dwRel == -1) {
        //     cout << "no streeing CAN data. " << endl;
        //     return  _current_angle_count[motorID-1];
        // } else {
        //     return  _current_angle_count[motorID-1];
        // }
    }
    return 0;
}

int MotorCan::SendReadErrorState(uint8_t motorID)
{
    int dwRel;
    /*send指令初始化*/
    vco_send[0].ID = (UINT)(0x00000140+motorID);
    vco_send[0].RemoteFlag = 0;
    vco_send[0].ExternFlag = 0; //扩展帧
    vco_send[0].DataLen = 8;
    BYTE b0 = vco_send[0].Data[0] = 0x9A;
    BYTE b1 = vco_send[0].Data[1] = 0x00;
    BYTE b2 = vco_send[0].Data[2] = 0x00;
    BYTE b3 = vco_send[0].Data[3] = 0x00;

    BYTE b4 = vco_send[0].Data[4] = 0x00;
    BYTE b5 = vco_send[0].Data[5] = 0x00;
    BYTE b6 = vco_send[0].Data[6] = 0x00;
    BYTE b7 = vco_send[0].Data[7] = 0x00;

    dwRel = VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco_send, 1);
    return dwRel;
}

int MotorCan::ReceiveReadErrorState(uint8_t motorID)
{
    int dwRel;
    int i =0;
    while(i++ < 100)
    {
        dwRel = VCI_Receive(nDeviceType, nDeviceInd, nCANInd, vco_receive, 64, 400);//设置等待超时时间

        int error_state = 0;
        // std::cout << vco_receive[0].ID-0x140 << " id" << std::endl;
        if (dwRel > 0 && vco_receive[0].ID == (UINT)(0x140+motorID)) {
            BYTE id = (BYTE)(vco_receive[0].Data[0]);
            // cout << (int)id << endl;
            if(id == (0x9A))
            {
                error_state += (int)vco_receive[0].Data[7];
                return error_state;                
            }
        } 
    }
    return 0;
}

int MotorCan::SendClearErrorState(uint8_t motorID){
    int dwRel;
    /*send指令初始化*/
    vco_send[0].ID = (UINT)(0x00000140+motorID);
    vco_send[0].RemoteFlag = 0;
    vco_send[0].ExternFlag = 0; //扩展帧
    vco_send[0].DataLen = 8;
    BYTE b0 = vco_send[0].Data[0] = 0x9B;
    BYTE b1 = vco_send[0].Data[1] = 0x00;
    BYTE b2 = vco_send[0].Data[2] = 0x00;
    BYTE b3 = vco_send[0].Data[3] = 0x00;

    BYTE b4 = vco_send[0].Data[4] = 0x00;
    BYTE b5 = vco_send[0].Data[5] = 0x00;
    BYTE b6 = vco_send[0].Data[6] = 0x00;
    BYTE b7 = vco_send[0].Data[7] = 0x00;

    dwRel = VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco_send, 1);
    return dwRel;
}

int MotorCan::ReceiveClearErrorState(uint8_t motorID)
{
    int dwRel;
    int i =0;
    while(i++ < 100)
    {
        dwRel = VCI_Receive(nDeviceType, nDeviceInd, nCANInd, vco_receive, 64, 400);//设置等待超时时间

        int error_state = 0;
        // std::cout << vco_receive[0].ID-0x140 << " id" << std::endl;
        if (dwRel > 0 && vco_receive[0].ID == (UINT)(0x140+motorID)) {
            BYTE id = (BYTE)(vco_receive[0].Data[0]);
            // cout << (int)id << endl;
            if(id == (0x9B))
            {

                error_state += (int)vco_receive[0].Data[7];
                return error_state;                
            }
        } 
    }
    return 0;
}

int MotorCan::MotorStop(uint8_t motorID)
{
    int dwRel;
    /*send指令初始化*/
    vco_send[0].ID = (UINT)(0x00000140+motorID);
    vco_send[0].RemoteFlag = 0;
    vco_send[0].ExternFlag = 0; //扩展帧
    vco_send[0].DataLen = 8;
    BYTE b0 = vco_send[0].Data[0] = 0x81;
    BYTE b1 = vco_send[0].Data[1] = 0x00;
    BYTE b2 = vco_send[0].Data[2] = 0x00;
    BYTE b3 = vco_send[0].Data[3] = 0x00;

    BYTE b4 = vco_send[0].Data[4] = 0x00;
    BYTE b5 = vco_send[0].Data[5] = 0x00;
    BYTE b6 = vco_send[0].Data[6] = 0x00;
    BYTE b7 = vco_send[0].Data[7] = 0x00;

    dwRel = VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco_send, 1);
    //cout << "SendSteeringCommand" << endl;
    //这些需要读取编码器值,然后更新当前角度变量
    return 1;
}
int MotorCan::UpdateMotorCurrentAngle(int32_t curr_angle,uint8_t motorID)
{
    // _current_angle_count = cangle;
    _current_angle_count[motorID-1] = curr_angle;
}
int MotorCan::MotorClose(uint8_t motorID)
{
    int dwRel;
    /*send指令初始化*/
    vco_send[0].ID = (UINT)(0x00000140+motorID);
    vco_send[0].RemoteFlag = 0;
    vco_send[0].ExternFlag = 0; //扩展帧
    vco_send[0].DataLen = 8;
    BYTE b0 = vco_send[0].Data[0] = 0x80;
    BYTE b1 = vco_send[0].Data[1] = 0x00;
    BYTE b2 = vco_send[0].Data[2] = 0x00;
    BYTE b3 = vco_send[0].Data[3] = 0x00;

    BYTE b4 = vco_send[0].Data[4] = 0x00;
    BYTE b5 = vco_send[0].Data[5] = 0x00;
    BYTE b6 = vco_send[0].Data[6] = 0x00;
    BYTE b7 = vco_send[0].Data[7] = 0x00;
   

    dwRel = VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco_send, 1);
    //cout << "SendSteeringCommand" << endl;
    //这些需要读取编码器值,然后更新当前角度变量
    return 1;
}
int MotorCan::MotorRun(uint8_t motorID)
{
    int dwRel;
    /*send指令初始化*/
    vco_send[0].ID = (UINT)(0x00000140+motorID);
    vco_send[0].RemoteFlag = 0;
    vco_send[0].ExternFlag = 0; //扩展帧
    vco_send[0].DataLen = 8;
    BYTE b0 = vco_send[0].Data[0] = 0x88;
    BYTE b1 = vco_send[0].Data[1] = 0x00;
    BYTE b2 = vco_send[0].Data[2] = 0x00;
    BYTE b3 = vco_send[0].Data[3] = 0x00;

    BYTE b4 = vco_send[0].Data[4] = 0x00;
    BYTE b5 = vco_send[0].Data[5] = 0x00;
    BYTE b6 = vco_send[0].Data[6] = 0x00;
    BYTE b7 = vco_send[0].Data[7] = 0x00;

    dwRel = VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco_send, 1);
    //cout << "SendSteeringCommand" << endl;
    //这些需要读取编码器值,然后更新当前角度变量
    return 1;
}
int MotorCan::ResetMotorAngle(uint8_t motorID)
{
    int dwRel;
    /*send指令初始化*/
    vco_send[0].ID = (UINT)(0x00000140+motorID);
    vco_send[0].RemoteFlag = 0;
    vco_send[0].ExternFlag = 0; //扩展帧
    vco_send[0].DataLen = 8;
    BYTE b0 = vco_send[0].Data[0] = 0x95;
    BYTE b1 = vco_send[0].Data[1] = 0x00;
    BYTE b2 = vco_send[0].Data[2] = 0x00;
    BYTE b3 = vco_send[0].Data[3] = 0x00;

    BYTE b4 = vco_send[0].Data[4] = 0x00;
    BYTE b5 = vco_send[0].Data[5] = 0x00;
    BYTE b6 = vco_send[0].Data[6] = 0x00;
    BYTE b7 = vco_send[0].Data[7] = 0x00;

    dwRel = VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco_send, 1);
    //cout << "SendSteeringCommand" << endl;
    //这些需要读取编码器值,然后更新当前角度变量
    return 1;
}

int MotorCan::ClearUSBCanBuffer()
{
    int dwRel;
    dwRel = VCI_ClearBuffer(nDeviceType,nDeviceInd,nCANInd);
    return dwRel;
}

int MotorCan::UpdateSingleMotorAngle(uint8_t motorID)
{
        ClearUSBCanBuffer();
        int dw;
        dw = ReadMultiCircleCommand(motorID);
        // std::cout << dw << " dw" << std::endl;
        int32_t cangle = 0;
        if(dw == 1)
        {
                cangle = ReceiveMultiCircleAngle(motorID);
		std::cout<<"读取当前的angle: "<<cangle<<std::endl;
                UpdateMotorCurrentAngle(cangle,motorID);
                return cangle;
        }
}

int MotorCan::UpdateSingleMotorState(uint8_t motorID)
{
        ClearUSBCanBuffer();
        int dw;
        dw = readMotorState(motorID);
        // std::cout << dw << " dw" << std::endl;
        int32_t cangle = 0;
        if(dw == 1)
        {
                cangle = ReceiveMotorState(motorID);
                return cangle;
        }
        return 0;
}

int MotorCan::UpdateAllMotorAngle()
{

        // std::cout << "!" << std::endl ;
        
        ClearUSBCanBuffer();
        
        // std::cout << "!" << std::endl ;
        
        int dw;
        dw = ReadMultiCircleCommand(1);
        // std::cout << dw << " dw" << std::endl;
        int32_t cangle = 0;
        if(dw == 1)
        {
                cangle = ReceiveMultiCircleAngle(1);
                UpdateMotorCurrentAngle(cangle,1);
        }        
        std::cout << "dw" << std::endl ;

        dw = ReadMultiCircleCommand(2);
        if(dw == 1)
        {
                cangle = ReceiveMultiCircleAngle(2);
                UpdateMotorCurrentAngle(cangle,2);
        }
        std::cout << "dw" << std::endl ;
        dw = ReadMultiCircleCommand(3);
        if(dw == 1)
        {
                cangle = ReceiveMultiCircleAngle(3);
                UpdateMotorCurrentAngle(cangle,3);
        }        
        std::cout << "dw" << std::endl ;

        dw = ReadMultiCircleCommand(4);
        if(dw == 1)
        {
                cangle = ReceiveMultiCircleAngle(4);
                UpdateMotorCurrentAngle(cangle,4);
        }        
        std::cout << "dw" << std::endl ;



        // 
        // dw = ReadMultiCircleCommand(5);
        // if(dw == 1)
        // {
        //         cangle = ReceiveMultiCircleAngle(5);
        //         UpdateMotorCurrentAngle(cangle,5);
        // }
        // std::cout << "dw" << std::endl ;
        // dw = ReadMultiCircleCommand(6);
        // if(dw == 1)
        // {
        //         cangle = ReceiveMultiCircleAngle(6);
        //         UpdateMotorCurrentAngle(cangle,6);
        // }
        // std::cout << "dw" << std::endl ;
        // dw = ReadMultiCircleCommand(7);
        // if(dw == 1)
        // {
        //         cangle = ReceiveMultiCircleAngle(7);
        //         UpdateMotorCurrentAngle(cangle,7);
        // }
        // std::cout << "dw" << std::endl ;
        // dw = ReadMultiCircleCommand(8);
        // if(dw == 1)
        // {
        //         cangle = ReceiveMultiCircleAngle(8);
        //         UpdateMotorCurrentAngle(cangle,8);
        // }
        // std::cout << "dw" << std::endl ;
}

/*
直接发送角度和速度,不需要*100,可以更加平稳
*/

int MotorCan::SendGearIncrementCommand(uint16_t sendspeed,int32_t sendangle,uint8_t motorID)
{
    UpdateSingleMotorAngle(motorID);
    uint16_t speed = sendspeed;
    //cout << "id:" << (int)motorID  << " " << speed << " speed " << "sendangle " << sendangle<<std::endl;
    // ReadMultiCircleCommand(motorID);
    // int32_t angle= ReceiveMultiCircleAngle(motorID)+sendangle*100;
    int32_t angle = _current_angle_count[motorID-1] + sendangle;
    cout << "id:" << (int)motorID  << " " << speed << " speed " << " angle " << angle<<std::endl;
    int dwRel;
    // cout << " _cur_ang:" << _current_angle_count[motorID-1] << " angle:" << angle  << endl;

    /*send指令初始化*/
    vco_send[0].ID = (UINT)(0x00000140+motorID);
    vco_send[0].RemoteFlag = 0;
    vco_send[0].ExternFlag = 0; //扩展帧
    vco_send[0].DataLen = 8;
    BYTE b0 = vco_send[0].Data[0] = 0xA4;
    BYTE b1 = vco_send[0].Data[1] = 0x00;
    BYTE b2 = vco_send[0].Data[2] = *(uint8_t*)(&speed);
    BYTE b3 = vco_send[0].Data[3] = *((uint8_t*)(&speed)+1);

    BYTE b4 = vco_send[0].Data[4] = *(uint8_t*)(&angle);
    BYTE b5 = vco_send[0].Data[5] = *((uint8_t*)(&angle)+1);
    BYTE b6 = vco_send[0].Data[6] = *((uint8_t*)(&angle)+2);
    BYTE b7 = vco_send[0].Data[7] = *((uint8_t*)(&angle)+3);

    dwRel = VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco_send, 1);
    cout<<(int)motorID<<endl;
    cout<<"speed:  "<<speed<<"   ---MotorCan::SendGearIncrementCommand"<<endl;
    //getState(motorID);
    // _current_angle_count[motorID-1] = angle;//todo:发送成功再更新
    return 1;
}

int MotorCan::readMotorState(uint8_t motorID)
{
    int dwRel;
    /*send指令初始化*/
    vco_send[0].ID = (UINT)(0x00000140+motorID);
    vco_send[0].RemoteFlag = 0;
    vco_send[0].ExternFlag = 0; //扩展帧
    vco_send[0].DataLen = 8;
    BYTE b0 = vco_send[0].Data[0] = 0x9C;
    BYTE b1 = vco_send[0].Data[1] = 0x00;
    BYTE b2 = vco_send[0].Data[2] = 0x00;
    BYTE b3 = vco_send[0].Data[3] = 0x00;

    BYTE b4 = vco_send[0].Data[4] = 0x00;
    BYTE b5 = vco_send[0].Data[5] = 0x00;
    BYTE b6 = vco_send[0].Data[6] = 0x00;
    BYTE b7 = vco_send[0].Data[7] = 0x00;

    dwRel = VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, vco_send, 1);
    //cout << "SendSteeringCommand" << endl;
    //这些需要读取编码器值,然后更新当前角度变量
    
    //timer_.SetTimer(10*1000,std::bind(ReadMotorStatePeriod,this,motorID));
    return 1;
}
int16_t MotorCan::ReceiveMotorState(uint8_t motorID) {
    int dwRel;
    int i =0;
    while(i++ < 100)
    {
        dwRel = VCI_Receive(nDeviceType, nDeviceInd, nCANInd, vco_receive, 64, 400);//设置等待超时时间

        int16_t iq = 0;
        // std::cout << vco_receive[0].ID-0x140 << " id" << std::endl;
        if (dwRel > 0 && vco_receive[0].ID == (UINT)(0x140+motorID)) {
            BYTE id = (BYTE)(vco_receive[0].Data[0]);
            // cout << (int)id << endl;
            if(id == (0x9C))
            {
                // std::cout << "id == 0x92" << std::endl;
                // std::cout << "data0 " << vco_receive[0].Data[1] << std::endl;
                iq += (int)vco_receive[0].Data[2];
                iq += ((int)vco_receive[0].Data[3]<<8);
                //cout<<iq<<endl;
                // current_steeringangle += ((int)vco_receive[0].Data[3]<<16);
                // current_steeringangle += ((int)vco_receive[0].Data[4]<<24);
                // current_steeringangle += ((int )vco_receive[0].Data[5]<<32);
                // current_steeringangle += ((int)vco_receive[0].Data[6]<<40);
                // current_steeringangle += ((int)vco_receive[0].Data[7]<<48);
                _current_state[motorID-1][1] = (int)vco_receive[0].Data[1];
                _current_state[motorID-1][2] = (int)vco_receive[0].Data[2];
                _current_state[motorID-1][3] = (int)vco_receive[0].Data[3];
                _current_state[motorID-1][4] = (int)vco_receive[0].Data[4];
                _current_state[motorID-1][5] = (int)vco_receive[0].Data[5];
                _current_state[motorID-1][6] = (int)vco_receive[0].Data[6];
                _current_state[motorID-1][7] = (int)vco_receive[0].Data[7];

            }
            //std::cout << "iq: " << iq <<", motorID: " <<motorID << "\n";
            // BYTE low = (BYTE)(vco_receive[0].Data[1]);
            // temp_angle = ((int)high << 8) + (int)low;

            // std::cout << "current_steeringangle = " << current_steeringangle << endl;
            // std::cout << "_current_steeringangle = " << _current_angle_count[motorID-1] << endl;
            //std::cout << "current_steering_angle_speed = " << steering_feedback.steering_angle_speed << endl;
            
            if(id == (0x9C))
            {
                return iq;                
            }


        } 
        // else if (dwRel == -1) {
        //     cout << "no streeing CAN data. " << endl;
        //     return  _current_angle_count[motorID-1];
        // } else {
        //     return  _current_angle_count[motorID-1];
        // }
    }
    return 0;
}

long long MotorCan::getAngle(uint8_t motorID)
{
    return _current_angle_count[motorID - 1];
}

int MotorCan::WriteAngle()
{
    Json::StyledWriter writer;
    Json::Reader reader;  
 
	Json::Value root;
  
    char szPath[128] = { 0 };
    getcwd(szPath, sizeof(szPath)-1);

    std::string current_path(szPath);
    current_path += "/test.json";
	ofstream ofs(current_path);
 
	if (!ofs.is_open())
	{
		cout << "Error opening file\n";
	}
    else
    {
        root["angle0"] = _current_angle_count[0];

        string jsonContents = writer.write(root);
        ofs<<jsonContents;
        cout<<(int)_current_angle_count[0]<<"  angle0"<<endl;

        cout<<"write end"<<endl;
    }

    return 0;
}

int MotorCan::ReadAngle()
{
    
    Json::Reader reader;  
 
	Json::Value root;

  
	std::ifstream infs("/home/slam/motorControl_bak_bak/src/motorControl/src/test.json");
 
	if (!infs.is_open())
	{
		std::cout << "Error opening file\n";
	}
  
    if (reader.parse(infs, root)){
        _current_angle_count[0] = root["angle0"].asInt();
        _current_angle_count[1] = root["angle1"].asInt();
        cout<<(int)_current_angle_count[0]<<"  angle0"<<endl;
        cout<<(int)_current_angle_count[1]<<"  angle1"<<endl;
    }

}

int MotorCan::ResetAngle(uint8_t motorID)
{
    SendGearIncrementCommand(10*100, -2*_current_angle_count[motorID-1],motorID);
}

int MotorCan::getState(uint8_t motorID)
{
    UpdateSingleMotorState(motorID);
    int16_t iq = 0;
    iq += _current_state[motorID-1][2];
    iq += _current_state[motorID-1][3]<<8;
    float incurrent = (float)iq * 33 / 2048;
    cout<<"i  =  "<<incurrent<<"  ---MotorCan::getState"<<endl;
    int16_t state_speed = 0;
    state_speed += _current_state[motorID - 1][4];
    state_speed += _current_state[motorID - 1][5]<<8;
    cout<<"speed  =  "<<state_speed<<"  ---MotorCan::getState"<<endl;
    uint16_t state_encoder = 0;
    state_encoder += _current_state[motorID - 1][6];
    state_encoder += _current_state[motorID - 1][7]<<8;
    cout<<"encoder  =  "<<state_encoder<<"  ---MotorCan::getState"<<endl;
    return 0;
}

void MotorCan::thread_getState(uint8_t motorID)
{
    UpdateSingleMotorState(motorID);
    uint16_t state_encoder = 0;
    state_encoder += _current_state[motorID - 1][6];
    state_encoder += _current_state[motorID - 1][7]<<8;
    //int16_t iq = 0;
    // iq += _current_state[motorID-1][2];
    // iq += _current_state[motorID-1][3]<<8;
    cout<<state_encoder<<endl;
    //if(iq>10000) cout<<"high iq:    "<<iq<<endl;
}

void MotorCan::setTorqueMode()
{
   mode = 1;
   cout<<"now Torquemode:  "<<mode<<endl;
}

float MotorCan::getStateIncurrent(uint8_t motorID)
{
   int16_t iq = 0;
   iq += _current_state[motorID-1][2];
   iq += _current_state[motorID-1][3]<<8;
   float incurrent = (float)iq * 33 / 2048;
   return incurrent;
}

int MotorCan::getStateSpeed(uint8_t motorID)
{
   int16_t state_speed = 0;
   state_speed += _current_state[motorID - 1][4];
   state_speed += _current_state[motorID - 1][5]<<8;
   return state_speed;
}

int MotorCan::getStateEncode(uint8_t motorID)
{
    uint16_t state_encoder = 0;
    state_encoder += _current_state[motorID - 1][6];
    state_encoder += _current_state[motorID - 1][7]<<8;
    return state_encoder;
}
