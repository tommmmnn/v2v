#ifndef AVOIDING_H_
#define AVOIDING_H_

#include "ros/ros.h"
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include "function.h"
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

class Avoider
{
public:
	Avoider();
	~Avoider();
	bool init();
	void detected_Callback(const nav_msgs::Odometry::ConstPtr& utm_msg,const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& bboxes);
	void get_offset(float target_speed, float t_ad, float sum_duration, float &offset);
	float getPathOffset(float target_speed);
	float return_Offset_By_Time(float target_speed);
	bool setPath(path_t path);
	bool generate_Acceleration(gpsMsg_t m_pose, float &a_A1, float &a_A2, float &a_B1, float &a_B2);
	float adjust_Velocity(float origin_v, float a, float t_ad);
	float generate_Adjust_Speed(float a1, float a2, uint8_t t_ad, float origin_speed);
	void get_Target_Speed(float target_speed);
	
public:
	uint8_t is_adjusting; //0:非调整 1：纵向调整 2：变道 3:完成变道

private:
	uint8_t lane;
	float vehicle_width;
	float vehicle_length;
	path_t m_path;
	float yaw_road;
	float m_target_speed;
	float a_max;		//最大调整加速度
	float res;			//for循环分辨率
	float t_ad;			//每一段调整时间
	float D_detect;		//激光雷达检测障碍物时开始调整的距离



	message_filters::Subscriber<jsk_recognition_msgs::BoundingBoxArray> sub_boundingboxes;
	message_filters::Subscriber<nav_msgs::Odometry> sub_utm;
	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, jsk_recognition_msgs::BoundingBoxArray> syncPolicy;
	typedef message_filters::Synchronizer<syncPolicy> Sync;
	boost::shared_ptr<Sync> sync_;

	ros::NodeHandle nh, priv_nh;
	// ros::Subscriber sub_boundingboxes;
	
};

#endif