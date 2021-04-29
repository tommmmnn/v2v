#include <auto_drive/record_path.h>

#define __NAME__ "record_path"

/* record path node for intelligent tractor
 * author: liushuaipeng, southeast university
 * email:  castiel_liu@outlook.com
 */


Recorder::Recorder()
{
	current_point_ = last_point_ = {0.0,0.0,0.0,0.0,0.0};
	vertex_recorder_state_ = VertexPathRecorderState_Idle;
	curve_recorder_state_ = CurvePathRecorderState_Idle;
}

Recorder::~Recorder()
{
	if(fp_ != NULL)
		fclose(fp_);
}

bool Recorder::init()
{
	ros::NodeHandle private_nh("~");
	
	private_nh.param<std::string>("path_file_dir",file_dir_,"");
	private_nh.param<float>("sample_distance",sample_distance_,0.1);
	
	if(file_dir_.empty())
	{
		ROS_ERROR("[%s] Please input record file path in launch file!", __NAME__);
		return false;
	}
	return true;
}

bool Recorder::tryOpenFile(const std::string& full_file)
{
	fp_ = fopen(full_file.c_str(),"w");
	if(fp_ == NULL)
	{
		ROS_ERROR("[%s] Open %s failed!",__NAME__, full_file.c_str());
		return false;
	}
	else
		ROS_INFO("[%s] New path file: %s created.",__NAME__, full_file.c_str());
	return true;
}

/*@brief 停止连续性路径记录
 *@param discard 是否放弃当前记录，放弃/保存
 */
void Recorder::stopCurveRecord(bool discard)
{
	if(curve_recorder_state_ == CurvePathRecorderState_Idle)
		return ;
		
	curve_recorder_state_ = CurvePathRecorderState_Idle;

	//此处加锁，只有记录线程退出后才能获得
	std::lock_guard<std::mutex> lock(curve_recorder_mutex_);

	if(discard && !full_file_now_.empty())
	{
		std::string rm_cmd = std::string("rm ") + full_file_now_;
		ROS_INFO("[%s] %s", __NAME__, rm_cmd.c_str());
		system(rm_cmd.c_str());
		full_file_now_ = ""; //防止多次删除
	}
}

/*@brief 连续性路径记录线程
*/
void Recorder::curvePathRecordThread()
{
	std::lock_guard<std::mutex> lock(curve_recorder_mutex_);

	curve_recorder_state_ = CurvePathRecorderState_Recording;
	while(curve_recorder_state_ == CurvePathRecorderState_Recording)
	{
		current_point_ = getPose_();

		if(calculate_dis2(current_point_,last_point_) >= sample_distance_*sample_distance_)
		{
			printf("[%s] recording: %.3f\t%.3f\t%.3f\r\n",__NAME__, current_point_.x,current_point_.y,current_point_.yaw);
			fprintf(fp_,"%.3f\t%.3f\t%.3f\r\n",current_point_.x,current_point_.y,current_point_.yaw);
			fflush(fp_);
			last_point_ = current_point_;
		}
		sleep_ms(50);
	}
}

bool Recorder::startVertexPathRecord(const std::string& file_name)
{
	full_file_now_ = file_dir_ + file_name;

	if(!tryOpenFile(full_file_now_))        //新建并打开路径文件
		return false;
	
	recorded_vertex_cnt_ = 0;    //开始新的顶点型路径记录，复位点数
	vertex_recorder_state_ = VertexPathRecorderState_Waiting;
	return true;
}

/*@brief 停止顶点型路径记录
 *@param discard 是否放弃当前记录，放弃/保存
 */
void Recorder::stopVertexRecord(bool discard)
{
	if(vertex_recorder_state_ == VertexPathRecorderState_Idle)
		return ;
		
	vertex_recorder_state_ = VertexPathRecorderState_Idle;

	if(discard && !full_file_now_.empty())
	{
		std::string rm_cmd = std::string("rm ") + full_file_now_;
		system(rm_cmd.c_str());
		full_file_now_ = ""; //防止多次删除
	}
}

bool Recorder::recordCurrentVertex(const gpsMsg_t& pose)
{
	if(vertex_recorder_state_ != VertexPathRecorderState_Waiting)
		return false;
	
	static gpsMsg_t last_vertex = {0., 0., 0.};

	float dis = dis2Points(pose, last_vertex, true);
	
	if(dis < 1.0)
	{
		ROS_ERROR("[%s] Request record current point, but too close to the previous point.", __NAME__);
		return false;
	}
	
	printf("[%s] recoding: %.3f\t%.3f\t%.3f\r\n",__NAME__, pose.x,pose.y,pose.yaw);
	fprintf(fp_,"%.3f\t%.3f\t%.3f\r\n",pose.x,pose.y,pose.yaw);
	fflush(fp_);

	recorded_vertex_cnt_ ++;  //已记录顶点个数自加
	last_vertex = pose;
	return true;
}

void Recorder::stopWithoutSave()
{
	ROS_ERROR("[%s] Close current recording without save!", __NAME__);
	stopCurveRecord(true); //退出连续路径记录器，并丢弃记录文件
	stopVertexRecord(true); //退出顶点路径记录器，并丢弃记录文件
}

void Recorder::stopAndSave()
{
	if(curve_recorder_state_ != CurvePathRecorderState_Idle)
		this->stopCurveRecord(false);
	if(vertex_recorder_state_ != VertexPathRecorderState_Idle)
		this->stopVertexRecord(false);
}

float Recorder::calculate_dis2(gpsMsg_t & point1,gpsMsg_t& point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	return x*x+y*y;
}

bool Recorder::is_location_ok(const gpsMsg_t pose)
{
	if(fabs(pose.x) < 1.0 && fabs(pose.y) < 1.0)
		return false;
	return true;
}


void Recorder::sleep_ms(int ms)
{
	std::this_thread::sleep_for(std::chrono::microseconds(ms));
}

