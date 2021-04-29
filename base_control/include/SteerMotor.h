#include "ros/ros.h"
#include "can_msgs/FrameArray.h"
#include "iostream"
#include "vector"
#include "cmath"
#include "mutex"
#include "thread"

#define FRAME_SIZE 3
#define MAX_STEERINGANGLE 120
#define MIN_STEERINGANGLE -120
#define MAX_TORQUE 1000
#define MAX_BRAKE 700

class SteerMotor
{
public:
	SteerMotor();
	~SteerMotor();
    bool init();
    void data_callback(const can_msgs::FrameArray::ConstPtr& frames);
    float getRoadWheelAngle();
    float getMotorSpeed();
    void Torque_Control(int16_t[]);		//四轮驱动
	void Steering_Control(int16_t);								//前轮转向
    void SetBrake(int16_t brake);		//制动控制
	void disp();
    bool is_enabled(){return is_enabled_;};
    void startRequestState(int duration);
    void requestStateThread(int duration);
    void pidController(float speed, float current_speed);
    void publishCtrl();
private:
    float road_wheel_angle;
    float steer_turning_speed;
    float vehicle_speed;
    float torque;
    float kp;
    float ki;
    float kd;
    bool is_enabled_;


    can_msgs::FrameArray frames;
    std::mutex request_state_thread_mutex;
    std::shared_ptr<std::thread> request_state_thread_ptr;

	ros::Publisher pub_ctrl;
    ros::NodeHandle nh, priv_nh;
    ros::Subscriber sub_fb;
};
