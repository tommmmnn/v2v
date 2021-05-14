#include "ros/ros.h"
#include "can_msgs/FrameArray.h"
#include "vector"
#include <cmath>
#include <unistd.h>
#include "iostream"
#include "string"
#include "SteerMotor.h"
#include "driverless_msgs/ControlCmd.h"
#include "driverless_msgs/BaseControlState.h"
#include "atomic"
#include "gps_msgs/Inspvax.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#define __NAME__ "base_control_node"
using namespace message_filters;

class Car_Control
{
public:
	Car_Control();
	~Car_Control();
	bool init();

public:
	void cmd_callback(const gps_msgs::Inspvax::ConstPtr& gps_msg, const driverless_msgs::ControlCmd::ConstPtr& msg);
	void publishState_callback(const ros::TimerEvent&);

private:
	ros::Publisher pub_state;
	ros::Timer pub_state_tmr;

	ros::NodeHandle nh, priv_nh;

	message_filters::Subscriber<driverless_msgs::ControlCmd> sub_cmd;
	message_filters::Subscriber<gps_msgs::Inspvax> sub_gps;
	typedef message_filters::sync_policies::ApproximateTime<gps_msgs::Inspvax, driverless_msgs::ControlCmd> syncPolicy;
	typedef message_filters::Synchronizer<syncPolicy> Sync;
	boost::shared_ptr<Sync> sync_;


	driverless_msgs::BaseControlState state;

	float current_speed;
	float steering_angle;
	float max_speed;
	float max_steering_angle;
	float yaw_road;
	float theta;

	SteerMotor steermotor;
	
	std::atomic<bool> driverless_mode;
};

Car_Control::Car_Control():
	driverless_mode(false),
	priv_nh("~")
{
	current_speed = 0;
}

Car_Control::~Car_Control()
{
	
}

bool Car_Control::init()
{
	ros::NodeHandle priv_nh("~");
	priv_nh.param<float>("max_speed", max_speed, 20.0);     // km/h
	priv_nh.param<float>("max_steering_angle", max_steering_angle, 19.08);   //°
	
	yaw_road = priv_nh.param<float>("yaw_road", 0);

	if(yaw_road == 0)
	{
		ROS_ERROR("please input yaw of path in launch file!");
		return false;
	}
	theta = -yaw_road + M_PI/2;

	std::string cmd_topic = priv_nh.param<std::string>("cmd_topic", "/cmd");
	// sub_cmd = nh.subscribe(cmd_topic, 1, &Car_Control::cmd_callback, this);
	std::string gps_topic = priv_nh.param<std::string>("gps_topic", "/gps");
	sub_gps.subscribe(nh, gps_topic, 1);
	sub_cmd.subscribe(nh, cmd_topic, 1);
	sync_.reset(new Sync(syncPolicy(10),sub_gps,sub_cmd));
	sync_->registerCallback(boost::bind(&Car_Control::cmd_callback, this, _1, _2));
	// message_filters::TimeSynchronizer<gps_msgs::Inspvax, driverless_msgs::ControlCmd> sync(sub_gps, sub_cmd, 10);
	// sync.registerCallback(boost::bind(&Car_Control::cmd_callback, this, _1, _2));

	pub_state_tmr = nh.createTimer(ros::Duration(0.02), &Car_Control::publishState_callback, this);
	pub_state = nh.advertise<driverless_msgs::BaseControlState>("/base_control_state", 1);

	if(!steermotor.init())
	{
		ROS_ERROR("[%s] init steering motor failed." __NAME__);
		return false;
	}
	
	return true;
}

void Car_Control::cmd_callback(const gps_msgs::Inspvax::ConstPtr& gps_msg, const driverless_msgs::ControlCmd::ConstPtr& msg)
{
	static int try_disable_motor_cnt = 0;
	float v_e = gps_msg->east_velocity;
	float v_n = gps_msg->north_velocity;
	current_speed = v_e * cos(theta) + v_n * sin(theta);    //沿路径的纵向速度，theta为路径航向角
	if(!msg->driverless_mode && driverless_mode)	//上一次处于自动驾驶模式,当前请求关闭
	{
		try_disable_motor_cnt = 50;
	}
	if(!msg->driverless_mode)
	{
		steermotor.SetBrake(msg->set_brake);
		steermotor.pidController(msg->set_speed, current_speed);
		if(try_disable_motor_cnt > 0)
		{
			--try_disable_motor_cnt;
			steermotor.Steering_Control(0);
		}
		steermotor.publishCtrl();
	}
	else
	{
		steermotor.Steering_Control(msg->set_roadWheelAngle);
		steermotor.pidController(msg->set_speed, current_speed);
	//	steermotor.SetBrake(-0.5);
	//	steermotor.SetBrake(msg->set_brake);
		steermotor.publishCtrl();
	}
	driverless_mode = msg->driverless_mode;
}

void Car_Control::publishState_callback(const ros::TimerEvent&)
{
	state.header.stamp = ros::Time::now();
	state.header.frame_id = "state";
	state.roadWheelAngle = steermotor.getRoadWheelAngle();    //CAN总线读取的轮胎转向
	state.motorSpeed = steermotor.getMotorSpeed();			  //CAN总线读取的轮胎转速处理后的车速
	state.steerMotorEnabled = steermotor.is_enabled();
	state.steerMotorError = 0;
	
	state.brakeError = 0;
	state.brake_val = 0;
	pub_state.publish(state);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "base_control_node");
	Car_Control car_control;
	if(car_control.init())
		ros::spin();
	std::cout << "[" << __NAME__ << "] exit!" << std::endl;
	return 0;
}
