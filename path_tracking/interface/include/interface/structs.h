#ifndef __STRUCTS__
#define __STRUCTS__
#include <can2serial/can2serial.h>

//can总线报文ID
enum 
{
    //发送
    GPS_CAN_ID = 0x301,          //gps定位信息上报
    GPS_MSG_CAN_ID = 0x302,      //gps其他信息上报
    RESPONSE_CAN_ID = 0x205,     //应答报文
    HEARTBEAT_CAN_ID = 0x204,    //主控心跳包
    
    //接收
    REQUEST_RECORD_PATH_CAN_ID = 0x200,  //请求记录路径
    REQUEST_DRIVERLESS_CAN_ID = 0x201,   //请求自动驾驶
    REQUEST_RESET_CAN_ID = 0x203,        //系统复位(清除转向电机错误代码/重启电机/制动复位)
    UI_HEARTBEAT_CAN_ID = 0x206,         //ui界面心跳包
};

typedef struct CanMsgs
{
    CanMsg_t gpsPos;
    CanMsg_t gpsMsg;
    CanMsg_t heartbeat;
    CanMsg_t response;

    CanMsgs()
    {
        gpsPos.ID = GPS_CAN_ID;
        gpsMsg.ID = GPS_MSG_CAN_ID;
        heartbeat.ID = HEARTBEAT_CAN_ID;
        heartbeat.len = 2;
        response.ID = RESPONSE_CAN_ID;
	    response.len = 4;
    }
    
} canMsgs_t;

typedef struct systemResetMsg
{
    uint8_t clearMotorError : 1;
    uint8_t rebootMotor     : 1;
    uint8_t brakeReset      : 1;
} systemResetMsg_t;

typedef struct heartbeatStruct
{
    uint8_t gpsState         :2;
    uint8_t brakeSystemState :2;
    uint8_t driveSystemState :4;
    uint8_t steerMotorState;
} heartbeatStruct_t;

#endif
