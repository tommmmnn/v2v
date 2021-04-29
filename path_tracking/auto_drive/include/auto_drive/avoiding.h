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

private:
	uint8_t lane;
	float vehicle_width;
	path_t m_path;
	float yaw_road;
	float m_target_speed;

	message_filters::Subscriber<jsk_recognition_msgs::BoundingBoxArray> sub_boundingboxes;
	message_filters::Subscriber<nav_msgs::Odometry> sub_utm;
	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, jsk_recognition_msgs::BoundingBoxArray> syncPolicy;
	typedef message_filters::Synchronizer<syncPolicy> Sync;
	boost::shared_ptr<Sync> sync_;

	ros::NodeHandle nh, priv_nh;
	// ros::Subscriber sub_boundingboxes;
	
};

#endif