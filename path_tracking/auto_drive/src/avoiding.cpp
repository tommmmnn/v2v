#include "auto_drive/avoiding.h"

#define MAX_OFFSET -2.5
#define PA 3
Avoider::Avoider():
	lane(0),
	m_target_speed(0),
	priv_nh("~"),
	is_adjusting(0)
{

}

Avoider::~Avoider()
{}

bool Avoider::init()
{
	std::string boundingboxes_topic = priv_nh.param<std::string>("boundingboxes_topic", "/detected_bounding_boxs");

	// sub_boundingboxes = nh.subscribe(boundingboxes_topic, 1, &Avoider::detected_Callback, this);
	vehicle_width = priv_nh.param<float>("vehicle_width", 1.10);
	vehicle_length = priv_nh.param<float>("vehicle_length", 2.1);
	a_max = priv_nh.param<float>("a_max", 2.0);
	res = priv_nh.param<float>("resolution", 0.1); 
	t_ad = priv_nh.param<float>("t_ad", 2);
	D_detect = priv_nh.param<float>("D_detect", 25);

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

//激光雷达检测回调函数，在其中加入避障算法
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
			auto dis = sqrt((x-width/2)*(x-width/2) + y*y);			//距障碍物的距离
			auto theta = utm_msg->pose.covariance[0] - yaw_road;	//与路径的夹角
			auto x_on_road = x * cos(theta) + y * sin(theta);		//路径坐标系下障碍物的坐标
			auto y_on_road = y * cos(theta) - x * sin(theta);
			auto length_on_road = length * cos(theta) + width * sin(theta);
			if(x_on_road > 0)		//只看前方的物体
			{
				if(y_on_road > 0)			//判断障碍物在车道左侧还是右侧，再判断障碍物是否占领了路径而使得车辆无法通过，且设置壁障距离
				{
					if(y_on_road - 0.8*length_on_road/2 < vehicle_width/2 && dis < m_target_speed*PA + 2.1)	//障碍物膨胀
					{
						lane = 1;
						is_adjusting = 1;		//检测到障碍物开始调整
						ROS_ERROR("the distance of the obstacle is %f, x_road:%f   y_road:%f  width:%f   length:%f length_road:%f", dis, x_on_road, y_on_road, width, length, length_on_road);
						break;
					}
				}
				else
				{
					if(y_on_road - 0.8*length_on_road/2 > -vehicle_width/2 && dis < m_target_speed*PA + 2.1)
					{
						lane = 1;
						is_adjusting = 1;
						ROS_ERROR("the distance of the obstacle is %f, x_road:%f   y_road:%f  width:%f   length:%f length_road:%f", dis, x_on_road, y_on_road, width, length, length_on_road);
						break;
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
	x = sum_duration * target_speed + 3.75 + target_speed*1.0;
	// ROS_ERROR("x is  %f", x);
	x_ad = target_speed * (t_ad) + 2.1;
	if(x >= x_ad) offset = MAX_OFFSET;
	else
	{
		c0 = 0;
		c1 = 0;
		c2 = (3 * MAX_OFFSET - 2 * c1 * x_ad - 3 * c0) / (x_ad * x_ad);
		c3 = (2 * c0 + c1 * x_ad - 2 * MAX_OFFSET) / (x_ad * x_ad * x_ad);
		offset = c0 + c1 * x + c2 * x * x + c3 * x * x * x;
	}
}

float Avoider::return_Offset_By_Time(float target_speed)
{
	ros::Time time = ros::Time::now();
	static ros::Duration duration(0);
	static ros::Time last_time = ros::Time::now();
	static ros::Duration sum_duration(0);
	ros::Duration revolution(0.1);

	static float offset = 0;
	// ROS_ERROR("the offset now is %f", offset);
	if(MAX_OFFSET == offset)	//完成变道
	{
		is_adjusting = 3;
	}
	duration += time - last_time;
	//ROS_ERROR("sum time: %f   ,   duration: %f", time.toSec(), (time - last_time).toSec());
	if(duration >= revolution)
	{
		sum_duration += duration;
		duration = ros::Duration(0);
		this->get_offset(target_speed, PA, sum_duration.toSec(), offset);
	}
	last_time = time;
	return offset;
}

void Avoider::get_Target_Speed(float target_speed)
{
	m_target_speed = target_speed;
}

float Avoider::getPathOffset(float target_speed)
{
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

bool Avoider::generate_Acceleration(gpsMsg_t m_pose,float &a_A1, float &a_A2, float &a_B1, float &a_B2)
{
	for(float a_A=-a_max; a_A<=a_max; a_A+=res)
	{
		for(float a_B=-a_max; a_B<=a_max; a_B+=res)
		{
			float XA_relative = a_A*t_ad*t_ad;		//相对原始速度多出来的调整距离
			float XB_relative = a_A*t_ad*t_ad; 
			float XA_remained = D_detect - (PA*m_target_speed+vehicle_length);
		}
	}
}

float Avoider::generate_Adjust_Speed(float a1, float a2, uint8_t t_ad, float origin_speed)
{
	ros::Time time = ros::Time::now();
	static ros::Duration duration(0);
	static ros::Time last_time = ros::Time::now();
	static ros::Duration sum_duration(0);
	ros::Duration revolution(0.1);
	ros::Duration reach_time(2*t_ad);

	static float vehicle_speed = origin_speed;
	// ROS_ERROR("the offset now is %f", offset);
	if(sum_duration >= reach_time)
	{
		is_adjusting = 2;		//调整完成，开始变道
		return origin_speed + a1*t_ad + a2*t_ad;
	}	
	duration += time - last_time;
	//ROS_ERROR("sum time: %f   ,   duration: %f", time.toSec(), (time - last_time).toSec());
	if(duration >= revolution)
	{
		sum_duration += duration;
		duration = ros::Duration(0);
		if(sum_duration.toSec() < t_ad)
		{
			vehicle_speed += origin_speed + a1*sum_duration.toSec();
		}
		else
			vehicle_speed += origin_speed + a1*t_ad + a2*(sum_duration.toSec() - t_ad);
	}
	last_time = time;
	return vehicle_speed;
}