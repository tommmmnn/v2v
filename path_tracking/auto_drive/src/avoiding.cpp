#include "auto_drive/avoiding.h"

#define MAX_OFFSET -2

Avoider::Avoider():
	lane(0),
	m_target_speed(0),
	priv_nh("~")
{
}

Avoider::~Avoider()
{}

bool Avoider::init()
{
	std::string boundingboxes_topic = priv_nh.param<std::string>("boundingboxes_topic", "/detected_bounding_boxs");

	// sub_boundingboxes = nh.subscribe(boundingboxes_topic, 1, &Avoider::detected_Callback, this);
	vehicle_width = priv_nh.param<float>("vehicle_width", 1.10); 

	std::string utm_topic = priv_nh.param<std::string>("utm_topic", "/odom");
	sub_utm.subscribe(nh, utm_topic, 1);
	sub_boundingboxes.subscribe(nh, boundingboxes_topic, 1);
	sync_.reset(new Sync(syncPolicy(10),sub_utm,sub_boundingboxes));
	sync_->registerCallback(boost::bind(&Avoider::detected_Callback, this, _1, _2));
	yaw_road = priv_nh.param<float>("yaw_road", 0);
	if(yaw_road == 0)
	{
		ROS_ERROR("**********please input yaw of road!*******");
		return 0;
	}

	return 1;
	
}

void Avoider::detected_Callback(const nav_msgs::Odometry::ConstPtr& utm_msg,const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& bboxes)
{
	size_t object_num = bboxes->boxes.size();
	if(lane == 0) //位于中间车道
	{	
		for (size_t i = 0; i < object_num; i++)
		{
			auto x = bboxes->boxes[i].pose.position.x / 2;
			auto y = bboxes->boxes[i].pose.position.y / 2;
			auto width = bboxes->boxes[i].dimensions.x;
			auto length = bboxes->boxes[i].dimensions.y;
			auto dis = sqrt((x-width/2)*(x-width/2) + y*y);
			auto theta = utm_msg->pose.covariance[0] - yaw_road;
			auto x_on_road = x * cos(theta) + y * sin(theta);
			auto y_on_road = y * cos(theta) - x * sin(theta);
			auto length_on_road = length * cos(theta) + width * sin(theta);
			if(x_on_road > 0)		//只看前方的物体
			{
				if(y_on_road > 0)			//判断障碍物在车道左侧还是右侧，再判断障碍物是否占领了路径而使得车辆无法通过，且设置壁障距离
				{
					if(y_on_road - 0.8*length_on_road/2 < vehicle_width/2 && dis < (m_target_speed*3+3))
					{
						lane = 1;
						ROS_ERROR("the distance of the obstacle is %f, x_road:%f   y_road:%f  width:%f   length:%f length_road:%f", dis, x_on_road, y_on_road, width, length, length_on_road);
					}
				}
				else
				{
					if(y_on_road - 0.8*length_on_road/2 > -vehicle_width/2 && dis < (m_target_speed*3+3))
					{
						lane = 1;
						ROS_ERROR("the distance of the obstacle is %f, x_road:%f   y_road:%f  width:%f   length:%f length_road:%f", dis, x_on_road, y_on_road, width, length, length_on_road);
					}
				}
			}
		}
		// ROS_ERROR("************not detected yet!********");
	}
}

void Avoider::get_offset(float target_speed, float t_ad, float sum_duration, float &offset)
{
	// y = c0 + c1*x + c2 * x^2 + c3* x^3
	float x, x_ad, c0, c1, c2, c3;
	x = sum_duration * target_speed;
	x_ad = target_speed * (t_ad);
	c0 = 0;
	c1 = 0;
	c2 = (3 * MAX_OFFSET - 2 * c1 * x_ad - 3 * c0) / (x_ad * x_ad);
	c3 = (2 * c0 + c1 * x_ad - 2 * MAX_OFFSET) / (x_ad * x_ad * x_ad);
	offset = c0 + c1 * x + c2 * x * x + c3 * x * x * x;
}

float Avoider::return_Offset_By_Time(float target_speed)
{
	ros::Time time = ros::Time::now();
	static ros::Duration duration(0);
	static ros::Time last_time = ros::Time::now();
	static ros::Duration sum_duration(0);
	ros::Duration revolution(0.1);
	ros::Duration reach_time(2);

	static float offset = 0;
	// ROS_ERROR("the offset now is %f", offset);
	if(sum_duration >= reach_time)
		return MAX_OFFSET;
	duration += time - last_time;
	//ROS_ERROR("sum time: %f   ,   duration: %f", time.toSec(), (time - last_time).toSec());
	if(duration >= revolution)
	{
		sum_duration += duration;
		duration = ros::Duration(0);
		this->get_offset(target_speed, 2, sum_duration.toSec(), offset);
	}
	last_time = time;
	return offset;
}

float Avoider::getPathOffset(float target_speed)
{
	m_target_speed = target_speed;
	if(lane == 0)
		return 0;
	else 
	{
		return this->return_Offset_By_Time(target_speed);
		// return -2;
	}
}

bool Avoider::setPath(path_t path)
{
	m_path = path;
	return true;
}