#ifndef PATH_RECORDER_H_
#define PATH_RECORDER_H_

#include <ros/ros.h>
#include <unistd.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <interface/RecordPath.h>
#include "auto_drive/structs.h"
#include "auto_drive/function.h"
#include <functional>
#include <thread>
#include <mutex>

/* record path node for intelligent tractor
 * author: liushuaipeng, southeast university
 * email:  castiel_liu@outlook.com
 */

class Recorder
{
public:
	typedef const gpsMsg_t (getPoseFun_t)(void); //定义函数指针
	
	Recorder();
	~Recorder();
	bool init();
	void stopWithoutSave();
	void stopAndSave();

	enum CurvePathRecorderState
	{
		CurvePathRecorderState_Idle,
		CurvePathRecorderState_Recording,
	};

	enum VertexPathRecorderState
	{
		VertexPathRecorderState_Idle,
		VertexPathRecorderState_Waiting,
		VertexPathRecorderState_RecordNow,
	};

	void curvePathRecordThread();
	template <typename classT>
	bool startCurvePathRecord(const std::string& file_name, const gpsMsg_t (classT::*fun)(void),classT* obj)
	{
		full_file_now_ = file_dir_ + file_name;

		if(!tryOpenFile(full_file_now_))        //新建并打开路径文件
			return false;

		getPose_ = std::bind(fun, obj); //绑定位姿获取回调函数
		std::thread t(&Recorder::curvePathRecordThread, this); //启动连续性路径记录线程
		t.detach();
		return true;
	}
	void stopCurveRecord(bool discard);

	bool startVertexPathRecord(const std::string& file_name);
	bool recordCurrentVertex(const gpsMsg_t& pose);
	void stopVertexRecord(bool discard);
	int  getRecordedVerTexCnt(){return recorded_vertex_cnt_;}

private:
	bool tryOpenFile(const std::string& full_file);
	
	void sleep_ms(int ms);
	float calculate_dis2(gpsMsg_t & point1,gpsMsg_t& point2);
	bool is_location_ok(const gpsMsg_t pose);

	std::string file_dir_;           //路径文件所在目录
	std::string full_file_now_;      //当前正在记录的文件全名
	uint8_t curve_recorder_state_;   //连续路径记录器状态
	uint8_t vertex_recorder_state_;  //顶点路径记录器状态
	std::mutex curve_recorder_mutex_;//连续路径记录线程互斥量，用于判断线程是否已经退出
	FILE *fp_;                       //路径文件句柄
	int recorded_vertex_cnt_;        //当前顶点型记录任务已经记录的顶点个数
	
	gpsMsg_t last_point_ , current_point_;
	std::function<getPoseFun_t> getPose_;
	
	float sample_distance_;
	ros::ServiceServer srv_record_path_;
	ros::Subscriber sub_utm_ ;
};

#endif
