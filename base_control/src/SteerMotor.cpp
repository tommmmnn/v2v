//本程序用于驱动电机和转向电机的控制
#include "SteerMotor.h"

SteerMotor::SteerMotor():
	priv_nh("~"),
	torque(-250),
	kp(85),
	ki(2.5),
	kd(8)
{
	frames.frames.resize(FRAME_SIZE);
	is_enabled_ = true;
}

SteerMotor::~SteerMotor()
{

}

bool SteerMotor::init()
{
	std::string frame_id = priv_nh.param<std::string>("frame_id","channel1");
	frames.header.frame_id = frame_id;		//通过channel1发送报文

	std::string fb_topic = priv_nh.param<std::string>("from_usbcan", "/from_usbcan");
	sub_fb = nh.subscribe<can_msgs::FrameArray>(fb_topic, 1, &SteerMotor::data_callback, this);
	std::string ctrl_topic = priv_nh.param<std::string>("ctrl_topic", "/to_usbcan");
	pub_ctrl = nh.advertise<can_msgs::FrameArray>(ctrl_topic, 1);
	this->startRequestState(50);
	return true;
}

void SteerMotor::pidController(float speed, float current_speed)
{
	//增量式速度PID控制
	static float last_err = 0;
	static float last_last_err = 0;
	float spd_err = (current_speed - speed);
	torque += kp * (spd_err - last_err) + ki * spd_err + kd * (spd_err - 2*last_err + last_last_err);
	if(torque > 1000) torque = 1000;
	else if (torque < -1000) torque = -1000;
	torque = int16_t(torque);
	// ROS_INFO("TORQUE is %f", torque);
	int16_t arr[4] = {torque, torque, torque, torque};
	this->Torque_Control(arr);
	last_last_err = last_err;
	last_err = spd_err;

}

float SteerMotor::getRoadWheelAngle()
{
	return road_wheel_angle;
}

float SteerMotor::getMotorSpeed()
{
	return vehicle_speed;
}

void SteerMotor::startRequestState(int duration)
{
	request_state_thread_ptr = std::shared_ptr<std::thread>(new std::thread(&SteerMotor::requestStateThread, this, duration));
}

void SteerMotor::requestStateThread(int duration)
{
	
}

void SteerMotor::publishCtrl()
{
	pub_ctrl.publish(frames);
}

void SteerMotor::data_callback(const can_msgs::FrameArray::ConstPtr& frames)
{
	std::lock_guard<std::mutex> lck(request_state_thread_mutex);
	int reclen = frames->frames.size();
	for(int i=0; i<reclen; i++)	
	{
		switch(frames->frames[i].id)
		{
			case 0x217:{	
							int16_t spd;
							spd = (frames->frames[i].data[1]) << 8 | frames->frames[i].data[0];
							if(spd == 0) break;
						}
						break;

			case 0x227:{	
							int16_t spd;
							spd = (frames->frames[i].data[1]) << 8 | frames->frames[i].data[0];
							if(spd == 0) break;
						}
						break;

			case 0x237:{	
							int16_t spd;
							spd = (frames->frames[i].data[1]) << 8 | frames->frames[i].data[0];
							if(spd == 0) break;
							vehicle_speed = -(spd*2*M_PI*0.455/60*3.6);
							// ROS_INFO("speed is %f", vehicle_speed);
						}
						break;

			case 0x247:{	
							int16_t spd;
							spd = (frames->frames[i].data[1]) << 8 | frames->frames[i].data[0];
							if(spd == 0) break;
						}
						break;

			case 0x160:{
							int16_t angle = (frames->frames[i].data[0]) << 8 | frames->frames[i].data[1];
							uint8_t aspeed = frames->frames[i].data[2];
							road_wheel_angle = angle * 0.1587 + 0.1454;
							steer_turning_speed = aspeed;
						}
						break;

			// case 0x1801FFF4:{	
			// 				uint16_t soc;
			// 				soc = (frames->frames[i].data[5]) << 8 | frames->frames[i].data[4];
			// 				ROS_INFO("The bettery now is %d", soc);
			// 			}
			// 			break; 		
			default:break;
		}
	}
}

void SteerMotor::Torque_Control(int16_t arr[4])
{
	for(int i=0; i<4; i++)
	{
		if(arr[i] > MAX_TORQUE) arr[i] = MAX_TORQUE;
		else if(arr[i] < -MAX_TORQUE) arr[i] = -MAX_TORQUE;
	}
	frames.frames[0].id = 0x150;						//第一帧存放驱动控制数据
	frames.frames[0].is_rtr = 0;
	frames.frames[0].is_extended = 0;
	frames.frames[0].len = 8;
	for(int i=0; i<4; i++)
	{
		frames.frames[0].data[2*i] = (arr[i] & 0xFF00)>>8;
		frames.frames[0].data[2*i+1] = (arr[i] & 0xFF);
	}
}

void SteerMotor::Steering_Control(int16_t angle)
{
	int16_t ctrl_angle = int16_t(((angle-0.1454)/0.1587));
	if(ctrl_angle > MAX_STEERINGANGLE) ctrl_angle = MAX_STEERINGANGLE;
	else if(ctrl_angle < MIN_STEERINGANGLE) ctrl_angle = MIN_STEERINGANGLE;
	frames.frames[1].id = 0x151;						//第二帧存放转向控制数据
	frames.frames[1].is_rtr = 0;
	frames.frames[1].is_extended = 0;
	frames.frames[1].len = 8;
	frames.frames[1].data[0] = (ctrl_angle & 0xFF00)>>8;
	frames.frames[1].data[1] = (ctrl_angle & 0xFF);
	//ROS_ERROR("Set motor ok!!");
}

void SteerMotor::SetBrake(int16_t brake)
{
	int16_t ctrl_brk = brake * 100;
	if(ctrl_brk > MAX_BRAKE) ctrl_brk = MAX_BRAKE;
	else if (ctrl_brk < -100) ctrl_brk = -100;
	frames.frames[2].id = 0x151;						//第三帧存放制动控制数据
	frames.frames[2].is_rtr = 0;
	frames.frames[2].is_extended = 0;
	frames.frames[2].len = 8;
	frames.frames[2].data[4] = (ctrl_brk & 0xFF00)>>8;
	frames.frames[2].data[5] = (ctrl_brk & 0xFF);
	//ROS_ERROR("Set Brake OK !!!");
}

void SteerMotor::disp()
{
		for(int j=0; j<FRAME_SIZE; j++)
		{
			std::cout<<"send data: ID "<<std::hex<<frames.frames[j].id<<'\t';
			for(int i=0; i<4; i++)
			{
				uint8_t a = frames.frames[j].data[2*i];
				uint8_t b = frames.frames[j].data[2*i+1];
				std::cout<<std::hex<<(a<<8|b)<<'\t';		
			}
			std::cout<<'\n';
		}
}
