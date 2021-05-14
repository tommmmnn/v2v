#include "auto_drive/record_path.h"
#include "auto_drive/path_tracking.h"
#include "auto_drive/avoiding.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread/locks.hpp"
#include "boost/thread/shared_mutex.hpp"
#include "auto_drive/state_machine.h"
#include "std_msgs/UInt8.h"
#include "interface/RecordPath.h"
#include "interface/Driverless.h"
#include "interface/DriverlessStatus.h"
#include "driverless_msgs/BaseControlState.h"

#define __NAME__ "my_tracker"
#define AVOIDING 1		//0普通跟踪，1避障，2调整后避障
namespace fs = boost::filesystem;

typedef boost::shared_mutex shared_mutex_t;
typedef boost::unique_lock<shared_mutex_t> write_lock_t;
typedef boost::shared_lock<shared_mutex_t> read_lock_t;

class Tracker{
public:
	Tracker();
	~Tracker();
	bool init();
	void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
	const gpsMsg_t currentPose();
	void update_timer_callback(const ros::TimerEvent&);
	bool recordPathService(interface::RecordPath::Request  &req,
						   		  interface::RecordPath::Response &res);
	bool driverlessService(interface::Driverless::Request &req, 
									interface::Driverless::Response &res);
	void autoDriveThread(float speed);	
	void base_control_state_callback(const driverless_msgs::BaseControlState::ConstPtr& msg);
private:
	float m_max_speed;
	float m_vehicle_speed;
	gpsMsg_t m_pose;
	path_t m_path;
	driverless_msgs::ControlCmd m_cmd;
	driverless_msgs::BaseControlState m_base_state;

	ros::NodeHandle nh, priv_nh;
	ros::Subscriber sub_utm;
	ros::Subscriber sub_ctrl_state;
	ros::Publisher pub_cmd;
	ros::Publisher pub_state;
	ros::Timer update_timer;

	ros::ServiceServer srv_recorder;
	ros::ServiceServer srv_driverless;

	std::string path_file_dir;
	
	StateMachine sys_state;
	Recorder recorder;
	PathTracking tracker;	
	Avoider avoider;

	shared_mutex_t pose_wr_mutex;
	std::mutex auto_drive_thread_mutex;
	std::mutex base_state_mutex;
};

Tracker::Tracker():
	priv_nh("~"),
	tracker(nh, priv_nh)
{
	m_cmd.set_speed = 0;			//控制消息置零
	m_cmd.set_roadWheelAngle = 0;
	m_pose.reset();		//位姿置0
}

Tracker::~Tracker()
{
}

bool Tracker::init()
{
	priv_nh.param<float>("max_speed", m_max_speed, 20.0);
	path_file_dir = priv_nh.param<std::string>("path_file_dir", "");
	if(path_file_dir.empty())
	{
		ROS_ERROR("no input file directory!");
		return false;
	}

	std::string utm_topic = priv_nh.param<std::string>("utm_topic", "/odom");
	sub_utm = nh.subscribe(utm_topic, 1, &Tracker::odom_callback, this);

	sub_ctrl_state = nh.subscribe("/base_control_state", 1, &Tracker::base_control_state_callback, this);

	std::string cmd_topic = priv_nh.param<std::string>("cmd_topic", "/cmd");
	pub_cmd = nh.advertise<driverless_msgs::ControlCmd>(cmd_topic, 1);
	pub_state = nh.advertise<std_msgs::UInt8>("/system_state", 1);

	update_timer = nh.createTimer(ros::Duration(0.02), &Tracker::update_timer_callback, this);

	srv_recorder = nh.advertiseService("record_path_service", &Tracker::recordPathService, this);
	srv_driverless = nh.advertiseService("driverless_service", &Tracker::driverlessService, this);

	if(!recorder.init()) return false;		//初始化记录对象

	while(ros::ok() && !is_gps_data_valid(m_pose))				
	{
		ROS_INFO("[%s]: GPS data is invalid, please check the GPS topic or waiting...", __NAME__);
		if(priv_nh.param<bool>("ignore_gps_offline", false))
		{
			ROS_ERROR("[%s]: GPS offline, but the system ignore it." __NAME__);
			break;
		}
		ros::Duration(0.5).sleep();
	}
	return true;
}


void Tracker::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    write_lock_t write_lock(pose_wr_mutex);
    m_pose.x = msg->pose.pose.position.x;
    m_pose.y = msg->pose.pose.position.y;
    m_pose.yaw = msg->pose.covariance[0];
    
    m_pose.longitude = msg->pose.covariance[1];
    m_pose.latitude = msg->pose.covariance[2];
     
    //std::cout << std::fixed << std::setprecision(2) << m_pose.x << "\t" << m_pose.y << std::endl;
    m_vehicle_speed = msg->twist.twist.linear.x;
}

void Tracker::base_control_state_callback(const driverless_msgs::BaseControlState::ConstPtr& msg)
{
	base_state_mutex.lock();
	m_base_state = *msg;
	base_state_mutex.unlock();
}

const gpsMsg_t Tracker::currentPose()
{
    //ROS_INFO("const gpsMsg_t AutoDrive::currentPose()_");
    read_lock_t read_lock(pose_wr_mutex);
    return m_pose;
}          

void Tracker::update_timer_callback(const ros::TimerEvent&)
{
	m_cmd.header.stamp = ros::Time::now();
	m_cmd.header.frame_id = "cmd";
    int system_state = sys_state.get();
    if(system_state == sys_state.State_VertexTracking ||
       system_state == sys_state.State_CurveTracking)
    {
        m_cmd.driverless_mode = true;
        m_cmd.set_brake = 0;
    }
    else if(system_state == sys_state.State_CompleteTrack ||
            system_state == sys_state.State_SuspendTrack)
    {
        m_cmd.driverless_mode = false;
        m_cmd.set_brake = 3;
		m_cmd.set_speed = 0;
    }
    else
    {
        m_cmd.driverless_mode = false;
		m_cmd.set_brake = 3;
		m_cmd.set_speed = 0;
    }
	pub_cmd.publish(m_cmd);

    std_msgs::UInt8 state;
    state.data = system_state;
    pub_state.publish(state);
}

bool Tracker::recordPathService(interface::RecordPath::Request  &req,
						   		  interface::RecordPath::Response &res)
{
	if(   sys_state.isBusy()      //系统正忙, 防止指令重复请求
	   || sys_state.isTracking()) //正在路径跟踪 上位机逻辑正确的情况下不会出现这种情况
	{
		res.success = res.FAIL;
		return true;
	}

	if(req.command_type == req.START_RECORD_PATH ) //请求开始记录
	{
		if(sys_state.isRecording()) //正在记录
			recorder.stopWithoutSave(); //强制终止当前记录

		sys_state.set(sys_state.State_SystemBusy);

		if(req.path_type ==req.CURVE_TYPE) //记录连续型路径
		{
			ROS_INFO("[%s] Request start record path: curve type.",__NAME__);
		
			//开始连续型路径记录，传入文件名、位置获取函数
			bool ok = recorder.startCurvePathRecord(req.path_file_name, &Tracker::currentPose, this);
			if(!ok)
			{
				sys_state.set(sys_state.State_SystemIdle);
				res.success = res.FAIL;
				return true;
			}
			sys_state.set(sys_state.State_CurvePathRecording);
		}
		else if(req.path_type == req.VERTEX_TYPE) //顶点型路径记录
		{
			ROS_INFO("[%s] Request start record path: vertex type.",__NAME__);
			//开始顶点型路径记录
			bool ok = recorder.startVertexPathRecord(req.path_file_name);
			if(!ok)
			{
				sys_state.set(sys_state.State_SystemIdle);
				res.success = res.FAIL;
				return true;
			}
			sys_state.set(sys_state.State_VertexPathRecording);
		}
		else
		{
			sys_state.set(sys_state.State_SystemIdle);
			ROS_ERROR("[%s] Expected path type error !",__NAME__);
			res.success = res.FAIL;
			return true;
		}
	}
	//请求记录当前点,仅对顶点型有效
	else if(req.command_type == req.RECORD_CURRENT_POINT)
	{
		//若当前非顶点型记录中,上位机逻辑错误
		if(sys_state.get() != sys_state.State_VertexPathRecording)
		{
			ROS_ERROR("[%s] Request record current point, but system is not in vertex recording!",__NAME__);
			res.success = res.FAIL;
			return true;
		}

		bool ok = recorder.recordCurrentVertex(m_pose);
		res.point_cnt = recorder.getRecordedVerTexCnt();
		if(!ok)
		{
			res.success = res.FAIL;
			return true;
		}
	}
	//请求停止记录
	else if(req.command_type == req.STOP_RECORD_PATH)
	{
		if(!sys_state.isRecording()) //未处于记录状态
		{
			recorder.stopWithoutSave();
			res.success = res.SUCCESS;
			return true;
		}
		ROS_INFO("[%s] Record path completed.",__NAME__);
		recorder.stopAndSave();
		sys_state.set(sys_state.State_SystemIdle);
	}
	else
		ROS_ERROR("[%s] recordPathService, cmd or type error.", __NAME__);
		
	res.success = res.SUCCESS;
	return true;
}

bool Tracker::driverlessService(interface::Driverless::Request  &req,
								  interface::Driverless::Response &res)
{
	if(sys_state.isBusy() ||    //系统正忙
	   sys_state.isRecording()) //正在记录路径，无法启动自动驾驶 上位机逻辑正确的情况下不会出现这种情况
	{
		ROS_ERROR("[%s] System busy. please wait amoment.",__NAME__);
		res.success = res.FAIL;
		return true;
	}
	std::cout << "driverlessService command type: " << int(req.command_type) << std::endl;
	//请求开始自动驾驶，首先中断当前已有的自动驾驶任务
	if(req.command_type == req.START)
	{
		sys_state.set(sys_state.State_SystemBusy); //将状态置为忙，正在执行的自动驾驶将自动退出
		ros::Duration(0.5).sleep();   //等待正在自动驾驶的线程退出(如果有)
		int max_try_num = 5;
		while(!auto_drive_thread_mutex.try_lock()) //尝试加锁, 失败:表明自动驾驶线程正在运行
		{
			if(--max_try_num ==0) //超出最大等待时长
			{
				sys_state.set(sys_state.State_SystemIdle);
				ROS_ERROR("[%s] Another task is running now, please waiting...", __NAME__);
				res.success = res.FAIL;
				return true;
			}
			ros::Duration(0.1).sleep();
		}
		auto_drive_thread_mutex.unlock(); //此处一定不能忘记解锁

		fs::path file = fs::path(path_file_dir)/req.path_file_name;
		if(!fs::exists(file))
		{
			sys_state.set(sys_state.State_SystemIdle);
		    ROS_ERROR("[%s] %s FileNotExist.",__NAME__,fs::system_complete(file).string().c_str());
			res.success = res.PATH_FILE_NOT_EXIST;
			return true;
			}

		std::string file_name = fs::system_complete(file).string();

		std::cout << "driverlessService file " << file_name <<  " exist." << std::endl;

		if(req.path_type == req.CURVE_TYPE) //连续型路径
		{
			//载入路径文件，并配置路径跟踪控制器
			if(!loadPath(file_name, m_path) || !this->tracker.setPath(m_path))
			{
				sys_state.set(sys_state.State_SystemIdle);
				ROS_ERROR("[%s] PATH_FILE_ERROR", __NAME__);
				res.success = res.PATH_FILE_ERROR;
				return true;
			}
			ROS_INFO("[%s] load %s complete.", __NAME__, file_name.c_str());
		}
		else if(req.path_type == req.VERTEX_TYPE)   //顶点型路径
		{
			if(!loadPath(file_name,m_path, 0.1))
			{
				sys_state.set(sys_state.State_SystemIdle);
				ROS_ERROR("[%s] PATH_FILE_ERROR", __NAME__);
				res.success = res.PATH_FILE_ERROR;
				return true;
			}
		}
		else		//路径类型错误
		{
			ROS_ERROR("[%s] Unknown path type", __NAME__);
			res.success = res.FAIL;
			sys_state.set(sys_state.State_SystemIdle);
			return true;
		}

		ROS_INFO("[%s] start to init tracker.", __NAME__);

		//初始化跟踪控制器
		tracker.setPath(m_path);
		if(!tracker.init(m_pose))
		{
			ROS_ERROR("[%s] init tracker failed.", __NAME__);
			res.success = res.FAIL;
			sys_state.set(sys_state.State_SystemIdle);
			return true;
		}
		ROS_INFO("[%s] init tracker complete.", __NAME__);

#if AVOIDING		
		//初始化避障控制器
		if(!avoider.init())
		{
			ROS_ERROR("[%s] init avoider failed.", __NAME__);
			res.success = res.FAIL;
			sys_state.set(sys_state.State_SystemIdle);
			return true;
		}
		ROS_INFO("[%s] init avoider complete.", __NAME__);
#endif
		//系统状态设置
		if(req.path_type == req.VERTEX_TYPE)  
		{
			sys_state.set(sys_state.State_VertexTracking);
			ROS_INFO("[%s] Vertex tracking path points size:%lu",__NAME__, m_path.points.size());
		}
		else if(req.path_type == req.CURVE_TYPE)
		{
			sys_state.set(sys_state.State_CurveTracking); //状态置为连续型跟踪中
			ROS_INFO("[%s] Curve tracking path points size: %lu",__NAME__, m_path.points.size());
		}
		else
		{
			ROS_ERROR("[%s] Unkown Path type",__NAME__);
			res.success = res.FAIL;
			sys_state.set(sys_state.State_SystemIdle);
			return true;
		}
		std::thread t(std::bind(&Tracker::autoDriveThread, this,req.speed));
		t.detach();
	}
	else if(req.command_type == req.STOP)     //请求停止，状态置为完成
		sys_state.set(sys_state.State_CompleteTrack);
	else if(req.command_type == req.SUSPEND)  //请求暂停
		sys_state.set(sys_state.State_SuspendTrack);
	else if(req.command_type == req.CONFIRM_STOP) //确认停止，状态置为空闲，释放制动
		sys_state.set(sys_state.State_SystemIdle);
	else
	{
		ROS_ERROR("[%s] Unknown command type", __NAME__);
		res.success = res.FAIL;
		return true;
	}
	
	res.success = res.SUCCESS;
	return true;
}

void Tracker::autoDriveThread(float speed)
{
	std::lock_guard<std::mutex> lck(auto_drive_thread_mutex);

	if(speed > m_max_speed)
		speed = m_max_speed;
	avoider.get_Target_Speed(speed);
	ros::Rate loop_rate(100);
	
	while(ros::ok() && sys_state.isTracking())
	{
		/*
		if(sys_state_.get() == sys_state_.State_SuspendTrack)
		{
			loop_rate.sleep();
			continue;
		}
		*/
		pose_wr_mutex.lock_shared();
		#if AVOIDING == 2 
		if(!avoider.is_adjusting || 1==avoider.is_adjusting)	//非变道阶段
		{
			bool update_state = tracker.update(speed, m_base_state.roadWheelAngle, m_pose, 0);
		}
		else if(2 == avoider.is_adjusting)
		{
			bool update_state = tracker.update(speed, m_base_state.roadWheelAngle, m_pose, avoider.getPathOffset(speed));
		}
		else 
			bool update_state = tracker.update(speed, m_base_state.roadWheelAngle, m_pose, -2.5);
		#elif AVOIDING == 1
			bool update_state = tracker.update(speed, m_base_state.roadWheelAngle, m_pose, avoider.getPathOffset(speed));
		#else 
		bool update_state = tracker.update(speed, m_base_state.roadWheelAngle, m_pose, 0); //当前速度用GPS的数据，转角用CAN总线的数据
		#endif
		pose_wr_mutex.unlock_shared();

        if(!update_state) //更新失败,抵达目标地或出现异常
        {
			//此处状态为跟踪完成，待上位机确认后置为空闲
			sys_state.set(sys_state.State_CompleteTrack);
			ROS_INFO("[%s] reach the destination.", __NAME__);
		//	m_cmd.driverless_mode = false;
			break;
		}
		else
        {
			tracker.getTrackingCmd(m_cmd.set_speed, m_cmd.set_roadWheelAngle);    //速度和转角控制
		#if AVOIDING==2
			if(1 == avoider.is_adjusting)		//调整阶段
			{
				m_cmd.set_speed = avoider.generate_Adjust_Speed(-0.6, 0.6, 2, speed);
			}
		#endif
		}
		loop_rate.sleep();
	} //自动驾驶完成
	
	ROS_INFO("[%s] auto drive completed...",__NAME__); 
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "my_tractor_node");
	ros::AsyncSpinner spinner(4);
	spinner.start();

	Tracker tracker;
	if(tracker.init())
	{
		ros::waitForShutdown();
	}
	return 0;
}
