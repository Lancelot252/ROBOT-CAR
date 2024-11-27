//存放控制相关的物理参数
// motor相关的宏观操作



#ifndef ROBOT_CONTROL
#define ROBOT_CONTROL
#include "MotorCan.h"
class RobotControl
{
protected:
    /* data */
    int32_t angle;
    int16_t ntorque;
    uint16_t motor_speed;
    uint16_t motor_max_speed;
    MotorCan usbCan;

    const uint8_t motor_1 = 1;
    const uint8_t motor_2 = 2;
    const uint8_t motor_3 = 3;
    const uint8_t motor_4 = 4;
    const uint8_t motor_5 = 5;
    const uint8_t motor_6 = 6;
    const uint8_t motor_7 = 7;
    const uint8_t motor_8 = 8;

    const uint16_t minMotorSpeed = 5;
public:
    RobotControl();
    RobotControl(MotorCan *usbCan);
    RobotControl(int canID);
    ~RobotControl();

    
    int motorFoward(uint8_t motorID);
    int motorback(uint8_t motorID);
    int motorFoward(uint8_t motorID,uint16_t speedOrTorque);
    int motorback(uint8_t motorID,uint16_t speedOrTorque);
    int motorFoward(uint8_t motorID,uint16_t speedOrTorque,int32_t cangle);
    int motorback(uint8_t motorID,uint16_t speedOrTorque,int32_t cangle);
    int motorStop();
    int motorStop(uint8_t motorID);
    int motorClose(uint8_t motorID);
    int motorUpdateAll();
    int motorUpdateSingle(uint8_t motorID);


    int motorSetSpeed(uint16_t speed);

    int motorLock(uint8_t motorID);
    int motorUnlock(uint8_t motorID);
    int motorAllLock();
    int motorCarUnlock();
    int motorCarLock();
    int motorCar5Lock();
    int motorReadAllAngle();
    int motorSetAngle(int32_t cangle);
    long long motorReadAngle(uint8_t motorID);

    /*errar_state*/
    int readErrorState(uint8_t motorID);
    int clearErrorState(uint8_t motorID);

    uint16_t getMotrSpeed();

    MotorCan *getUsbCan();

    int motorTorqueFoward();
};
#endif