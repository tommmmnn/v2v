#ifndef INTERFCE_H_
#define INTERFCE_H_

#include <can2serial/can2serial.h>
#include <std_msgs/Float32.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <driverless_msgs/PathTrackingInfo.h>
#include <driverless_msgs/BaseControlState.h>
#include <interface/RecordPath.h>
#include <interface/Driverless.h>
#include <interface/DriverlessStatus.h>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt8.h>
#include <gps_msgs/Inspvax.h>
#include <thread>
#include <mutex>
#include <atomic>
#include "structs.h"


class Interface
{
  public:
	Interface();
	~Interface();
	bool init();
	void run();
	
  private:
	void readCanMsg();
	bool configCan2Serial(bool reconfig = false);
	void gps_callback(const gps_msgs::Inspvax::ConstPtr& msg);
	void msgReport_callback(const ros::TimerEvent& event);
	void heartbeat_callback(const ros::TimerEvent& event);
	void uiHeartbeatOvertime_callback(const ros::TimerEvent& event);
	void callServiceThread(const CanMsg_t& can_msg);

	void path_tracking_info_callback(const driverless_msgs::PathTrackingInfo::ConstPtr& );
	void baseControlState_callback(const driverless_msgs::BaseControlState::ConstPtr& msg);
	void driveSystemState_callback(const std_msgs::UInt8::ConstPtr& msg);
	
  private:
	Can2serial * can2serial_;
	std::string can2serial_port_;
	int can_baudrate_;
	std::mutex msg_reading_mutex_;
	bool is_msg_reading_;

	canMsgs_t can_pkgs_;
	heartbeatStruct_t heart_beat_pkg_;
	std::mutex heart_beat_pkg_mutex_;

	bool gps_validity_;
	std::atomic<double> last_gps_time_;
	std::atomic<double> last_track_info_time_;
	
	ros::Subscriber sub_gps_;
	ros::Subscriber sub_pathtracking_info_;
	ros::Subscriber sub_steerMoter_state_;
	ros::Subscriber sub_system_state_;
	ros::Subscriber sub_brake_state_;

	ros::Timer msg_report_timer_;
	ros::Timer heartbeat_timer_;
	ros::Timer ui_heartbeat_timer_;
	
	std::atomic<double> last_ui_heatbeat_time_;
	
	boost::shared_ptr<boost::thread> read_canMsg_thread_;
	
	ros::ServiceClient client_recordPath_;
	ros::ServiceClient client_driverless_;
	
	ros::ServiceServer server_driverless_status_;
	ros::ServiceClient client_clearMotorError_;//清除转向电机错误代码
	ros::ServiceClient client_rebootMotor_;
	ros::ServiceClient client_resetBraker_;
};


#endif
