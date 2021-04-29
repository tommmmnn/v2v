#include <ros/ros.h>
#include <interface/interface.h>

/* can bus information interface node for intelligent tractor
 * author: liushuaipeng, southeast university
 * email:  castiel_liu@outlook.com
 */

#define __NAME__ "interface_node"

Interface::Interface():
	gps_validity_(false),
	last_gps_time_(0.0),
	is_msg_reading_(false),
	can2serial_(NULL)
{
	setlocale(LC_ALL, ""); //调试信息中文编码
}

Interface::~Interface()
{
	can2serial_->StopReading();
	delete can2serial_;
	can2serial_ = NULL;
}

bool Interface::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	std::string gps_topic = nh_private.param<std::string>("gps_topic","");
	nh_private.param<std::string>("can2serial_port",can2serial_port_,"");
	nh_private.param<int>("can_baudrate",can_baudrate_,250);
	if(can2serial_port_.empty())
	{
		ROS_ERROR("[%s] Please input can2serial_port in launch file!", __NAME__);
		return false;
	}
	
	if(gps_topic.empty())
	{
		ROS_ERROR("[%s] Please input gps_topic in launch file!", __NAME__);
		return false;
	}

	sub_gps_ = nh.subscribe(gps_topic, 1,&Interface::gps_callback,this);

	sub_pathtracking_info_ = 
		nh.subscribe("tracking_info",1,&Interface::path_tracking_info_callback, this);

	sub_steerMoter_state_ = nh.subscribe("/base_control_state", 1, &Interface::baseControlState_callback, this);
	sub_system_state_ = nh.subscribe("/system_state", 1, &Interface::driveSystemState_callback, this);
		
	msg_report_timer_ = nh.createTimer(ros::Duration(0.1), &Interface::msgReport_callback,this);
	heartbeat_timer_ = nh.createTimer(ros::Duration(0.5), &Interface::heartbeat_callback, this);
	ui_heartbeat_timer_=nh.createTimer(ros::Duration(4.0), &Interface::uiHeartbeatOvertime_callback, this);
	
	//创建路径记录服务客户端
	client_recordPath_ = nh.serviceClient<interface::RecordPath>("record_path_service");
	//创建自动驾驶服务客户端
	client_driverless_ = nh.serviceClient<interface::Driverless>("driverless_service");
	//创建清除转向电机错误代码客户端
	client_clearMotorError_ = nh.serviceClient<std_srvs::Empty>("clear_motor_error_flag");
	//创建重启转向电机客户端
	client_rebootMotor_ = nh.serviceClient<std_srvs::Empty>("reboot_motor");
	//创建复位制动执行器客户端
	client_resetBraker_ = nh.serviceClient<std_srvs::Empty>("reset_braker");
	
	return configCan2Serial();
}

bool Interface::configCan2Serial(bool reconfig)
{
	is_msg_reading_ = false;
	std::lock_guard<std::mutex> lck(msg_reading_mutex_); //锁定读取线程
	
	ROS_INFO("[%s] start configCan2Serial.", __NAME__);
	
	if(can2serial_ != NULL)
	{
		ROS_INFO("[%s] stop history reading thread .", __NAME__);
		can2serial_->StopReading();
	}
	else	
		can2serial_ = new Can2serial;

	ROS_INFO("[%s] start configure can serial port.", __NAME__);
	int try_cnt = 0, max_try_cnt = 5;
	while(ros::ok() && !can2serial_->configure_port(can2serial_port_))
	{
		ROS_ERROR("[%s] %d/%d Try configure can2serial failed.", __NAME__, ++try_cnt, max_try_cnt);
		if(try_cnt >= max_try_cnt)
			return false;
		ros::Duration(1.0).sleep();
	}

	ROS_INFO("[%s] configure can baud_rate, filter.", __NAME__);
	can2serial_->configBaudrate(can_baudrate_);
	//can2serial_->setCanFilter(0x00,0x200,0x7f0); //200-20f
	can2serial_->setCanFilter_alone(0x01,REQUEST_RECORD_PATH_CAN_ID);
	can2serial_->setCanFilter_alone(0x02,REQUEST_RESET_CAN_ID);
	can2serial_->setCanFilter_alone(0x03,REQUEST_DRIVERLESS_CAN_ID);
	can2serial_->setCanFilter_alone(0x04,UI_HEARTBEAT_CAN_ID);

	if(!reconfig)
	{
		//can2serial_->clearCanFilter();
		
	}
	
	ros::Duration(0.5).sleep();
	ROS_INFO("[%s] start read pkg from buffer.", __NAME__);
	can2serial_->StartReading();
	
	//接收can总线消息线程
	ROS_INFO("[%s] start read can msgs thread.", __NAME__);
	std::thread t(&Interface::readCanMsg, this);
	t.detach();

	return true;
}

//根据can消息在新线程中请求对应的服务，防止阻塞接收线程
void Interface::callServiceThread(const CanMsg_t& can_msg)
{
	static bool threadIsRunning = false;
	if(threadIsRunning)
	{
		can_pkgs_.response.data[0] = 0x02; //通用应答
		can_pkgs_.response.data[1] = 0x00; //系统正忙
		can2serial_->sendCanMsg(can_pkgs_.response);
		return;
	}

	threadIsRunning = true;

	if(can_msg.ID == REQUEST_RECORD_PATH_CAN_ID) //记录路径ID
	{
		interface::RecordPath srv_record_path;
		int file_seq = can_msg.data[1]*256 + can_msg.data[0];   //文件序号
		srv_record_path.request.path_type = can_msg.data[2];    //路径类型
		srv_record_path.request.path_file_name = std::to_string(file_seq)+"_"  //文件名 = 序号+类型
												+ std::to_string(srv_record_path.request.path_type)+".txt";
		
		srv_record_path.request.command_type = can_msg.data[3]; //指令类型，开始?停止?记录当前?
		
		ROS_INFO("\n[%s] <记录路径> \n\t路径类型: (%d->顶点型, %d->连续型)\t \n\t指令类型: (%d->开始记录, %d->停止记录, %d->记录当前)", __NAME__, 
					srv_record_path.request.VERTEX_TYPE, srv_record_path.request.CURVE_TYPE,
					srv_record_path.request.START_RECORD_PATH, srv_record_path.request.STOP_RECORD_PATH, srv_record_path.request.RECORD_CURRENT_POINT);
				
		ROS_INFO("[%s] <Request Recod Path>: %s\t type:%d\t cmd:%d",__NAME__, 
							srv_record_path.request.path_file_name.c_str(),
							srv_record_path.request.path_type,
							srv_record_path.request.command_type);

		can_pkgs_.response.data[0] = 0x02; //universal response
		//can_pkgs_.response.data[1] = 0xff; //request received.
		can2serial_->sendCanMsg(can_pkgs_.response); //response

		client_recordPath_.call(srv_record_path);
		can_pkgs_.response.data[0] = 0x00;//response record path
		can_pkgs_.response.data[1] = srv_record_path.response.success;
		can_pkgs_.response.data[2] = srv_record_path.response.point_cnt%256;
		can_pkgs_.response.data[3] = srv_record_path.response.point_cnt/256;
		
		can2serial_->sendCanMsg(can_pkgs_.response); //response
		//can2serial_->showCanMsg(can_msg, "request record path");
		can2serial_->showCanMsg(can_pkgs_.response, "response record path");
	}
	else if(can_msg.ID == REQUEST_DRIVERLESS_CAN_ID)  //自动驾驶
	{
		interface::Driverless srv_driverless;
		srv_driverless.request.command_type = can_msg.data[3];   //指令类型
		srv_driverless.request.path_type = can_msg.data[2];      //路径类型
		int file_seq = can_msg.data[1]*256 + can_msg.data[0];         //路径序号
		// seq_type.txt
		srv_driverless.request.path_file_name = std::to_string(file_seq)+"_" 
												+ std::to_string(srv_driverless.request.path_type)+".txt";
		srv_driverless.request.speed = can_msg.data[4];

		ROS_INFO("[%s] <自动驾驶> \n\t路径类型: (%d->顶点型, %d->连续型)\t \n\t指令类型: (%d->开始, %d->停止, %d->暂停， %d->停止确认)", __NAME__, 
					srv_driverless.request.VERTEX_TYPE, srv_driverless.request.CURVE_TYPE,
					srv_driverless.request.START, srv_driverless.request.STOP, srv_driverless.request.SUSPEND, srv_driverless.request.CONFIRM_STOP);

		ROS_INFO("[%s] Request auto drive:%s\t type:%d\t cmd:%d",__NAME__,
							srv_driverless.request.path_file_name.c_str(),
							srv_driverless.request.path_type,
							srv_driverless.request.command_type);  

		can_pkgs_.response.data[0] = 0x02; //universal response
		can_pkgs_.response.data[1] = 0xff; //request received.
		//can2serial_->sendCanMsg(can_pkgs_.response); //response

		client_driverless_.call(srv_driverless);
		can_pkgs_.response.data[0] = 0x01;//response driverless
		can_pkgs_.response.data[1] = srv_driverless.response.success;
		
		can2serial_->sendCanMsg(can_pkgs_.response); //response
		//can2serial_->showCanMsg(can_msg, "request diverless");
		can2serial_->showCanMsg(can_pkgs_.response, "response diverless");
	}
	else if(can_msg.ID == REQUEST_RESET_CAN_ID) //系统复位
	{
		can_pkgs_.response.data[0] = 0x02; //universal response
		can_pkgs_.response.data[1] = 0xff; //request received.
		//can2serial_->sendCanMsg(can_pkgs_.response); //response

		systemResetMsg_t *resetMsg = (systemResetMsg_t *)can_msg.data;
		std_srvs::Empty empty;
		if(resetMsg->clearMotorError) //清除转向电机错误标志
			client_clearMotorError_.call(empty);
		if(resetMsg->rebootMotor) //重启转向电机
			client_rebootMotor_.call(empty);
		if(resetMsg->brakeReset)  //复位制动执行器
		{
			interface::Driverless srv_driverless; //确认停止自动驾驶，恢复系统空闲状态
			srv_driverless.request.command_type = srv_driverless.request.CONFIRM_STOP; 
			client_driverless_.call(srv_driverless);

			client_resetBraker_.call(empty); //复位制动执行器
		}
		
		can2serial_->showCanMsg(can_msg, "request reset");
	}

	threadIsRunning = false;
}

/*can2serial 定时状态检查，如果出现异常，尝试重新配置
	异常情况1: 长时间没有收到界面心跳包(can2serial可能初始化未成功)
	异常情况2: USB松动出现读取/发送异常
 */
void Interface::uiHeartbeatOvertime_callback(const ros::TimerEvent& event)
{
	if(ros::Time::now().toSec() - last_ui_heatbeat_time_ > 8.0)
	{//上位机心跳超时，可能can模块故障
		ROS_ERROR("[%s] UI heartbeat overtime, try to relaunch the can.", __NAME__);
		this->configCan2Serial(true); //重启can模块
	}
/*	else if(!can2serial_->isRunning()) //USB松动导致读取/发送异常
	{
		ROS_ERROR("[%s] can2serial running failed. try to relaunch the can.", __NAME__);
		this->configCan2Serial(true); //重启can模块
	}*/
}

void Interface::readCanMsg()
{
	CanMsg_t can_msg;
	is_msg_reading_ = true;
	std::lock_guard<std::mutex> lck(msg_reading_mutex_);
	
	while(ros::ok() && is_msg_reading_)
	{
		if(!can2serial_->getCanMsg(can_msg))
		{
			ros::Duration(0.1).sleep();
			continue;
		}
		//can2serial_->showCanMsg(can_msg);
		switch(can_msg.ID)
		{
			case REQUEST_RECORD_PATH_CAN_ID: //记录路径相关can消息
			case REQUEST_DRIVERLESS_CAN_ID:  //请求自动驾驶相关can消息
			case REQUEST_RESET_CAN_ID:       //系统复位
			{
				//使用新线程调用服务器，避免阻塞
				std::thread t(&Interface::callServiceThread, this, can_msg);
				t.detach(); //防止线程句柄释放导致线程意外退出
				break;
			}
			case UI_HEARTBEAT_CAN_ID: //上位机UI界面心跳包
				last_ui_heatbeat_time_ = ros::Time::now().toSec();
				//ROS_INFO("[%s] Received UI heart beat.",__NAME__);
				break;
			default:
				ROS_ERROR("[%s] Unknown CAN ID : 0x%02x", __NAME__, can_msg.ID);
				break;
		}
	}	
}

void Interface::gps_callback(const gps_msgs::Inspvax::ConstPtr& msg)
{
	last_gps_time_ = ros::Time::now().toSec();
	double yaw = msg->azimuth;
	uint16_t height = msg->height * 10 + 2000;
	
	*(uint32_t *)(can_pkgs_.gpsPos.data) = uint32_t (msg->longitude*10000000);
	*(uint32_t *)(can_pkgs_.gpsPos.data+4) = uint32_t (msg->latitude*10000000);
	
	*(uint16_t *)(can_pkgs_.gpsMsg.data) = uint16_t(yaw*10); //yaw
	can_pkgs_.gpsMsg.data[2] = 10 << 3;  //satellite_num << 3
	can_pkgs_.gpsMsg.data[2] |= 0x01; // none differentiation positioning 
	*(uint16_t *)(can_pkgs_.gpsMsg.data+6) = height;

	float speed = sqrt(msg->east_velocity*msg->east_velocity + 
					   msg->north_velocity*msg->north_velocity) * 3.6;
	uint16_t u16_speed = uint16_t(speed *10);
	can_pkgs_.gpsMsg.data[3] = u16_speed%256;
	can_pkgs_.gpsMsg.data[4] = u16_speed/256;
}

void Interface::path_tracking_info_callback(const driverless_msgs::PathTrackingInfo::ConstPtr& info)
{
	last_track_info_time_ = ros::Time::now().toSec();

	uint16_t lateral_err = uint16_t(info->lateral_err*100) + 255;
	//ROS_INFO("lateral_err:%.2f \t speed:%.2f",info->lateral_err,info->speed );
	can_pkgs_.gpsMsg.data[4] |= lateral_err%2 << 7;
	can_pkgs_.gpsMsg.data[5] = lateral_err/2;
}

//状态数据定时上报
void Interface::msgReport_callback(const ros::TimerEvent& event)
{
	if(gps_validity_)
	{
		can2serial_->sendCanMsg(can_pkgs_.gpsPos);
//		can2serial_->showCanMsg(can_pkgs_.gpsPos);
		usleep(1000);
	}
	if(ros::Time::now().toSec() - last_track_info_time_ > 0.3)
	{	//若跟踪信息超时,横向偏差置0
		uint16_t lateral_err = uint16_t(0.0*100) + 255;
		can_pkgs_.gpsMsg.data[4] |= lateral_err%2 << 7;
		can_pkgs_.gpsMsg.data[5] = lateral_err/2;
	}
	can2serial_->sendCanMsg(can_pkgs_.gpsMsg);
	//can2serial_->showCanMsg(can_pkgs_.gpsMsg);
}

//发送心跳包
void Interface::heartbeat_callback(const ros::TimerEvent& event)
{
	std::lock_guard<std::mutex> lck(heart_beat_pkg_mutex_);
	if(ros::Time::now().toSec() - last_gps_time_ > 2.0)
	{
		heart_beat_pkg_.gpsState = 1; //offline
		ROS_ERROR("[%s] gps offline!", __NAME__);
		gps_validity_ = false;
	}
	else
	{
		heart_beat_pkg_.gpsState = 0; //online
		gps_validity_ = true;
	}

	memcpy(can_pkgs_.heartbeat.data, &heart_beat_pkg_, sizeof(heart_beat_pkg_));
	
	can2serial_->sendCanMsg(can_pkgs_.heartbeat);
	//can2serial_->showCanMsg(can_pkgs_.heartbeat);
}

//驾驶系统状态反馈
void Interface::driveSystemState_callback(const std_msgs::UInt8::ConstPtr& msg)
{
	heart_beat_pkg_mutex_.lock();
	heart_beat_pkg_.driveSystemState = msg->data;
	heart_beat_pkg_mutex_.unlock();
}

//底层控制状态反馈
void Interface::baseControlState_callback(const driverless_msgs::BaseControlState::ConstPtr& msg)
{
	heart_beat_pkg_mutex_.lock();
	heart_beat_pkg_.brakeSystemState = msg->brakeError;     //制动系统状态
	heart_beat_pkg_.steerMotorState = msg->steerMotorError; //转向系统状态
	heart_beat_pkg_mutex_.unlock();

	if(msg->steerMotorError != 0)
		ROS_ERROR("[%s] steerMotor error: code:  %d",__NAME__, msg->steerMotorError);
	if(msg->brakeError != 0)
		ROS_ERROR("[%s] brake system error: code:  %d",__NAME__, msg->brakeError);
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, "interface_node");
	Interface interface;
	if(interface.init())
		ros::spin();
	
	return 0;
}
