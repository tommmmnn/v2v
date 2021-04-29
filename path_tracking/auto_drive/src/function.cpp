#include "auto_drive/function.h"


/*@brief 计算目标点到达路径的距离,点在路径左侧为负,右侧为正
 *@brief 该函数主要用于计算主体车辆到路径的距离(横向偏差),并更新最近点索引
 *@param x,y         目标点坐标
 *@param path_points 路径点集
 *@param ref_point_index 参考点索引，以此参考点展开搜索，加速计算
 *@param nearest_point_index 输出与目标点最近的路径点索引
 */
float calculateDis2path(const double& x,const double& y,
						 const path_t & path_points, //path
						 size_t   ref_point_index, //参考点索引
						 size_t& nearest_point_index)
{
	int searchDir; //搜索方向 -1:向后搜索， 1：向前搜索， 0 搜索完毕
	if(ref_point_index == 0)
	{
		ref_point_index = 1;
		searchDir = 1;
	}
	else if(ref_point_index == path_points.size()-1)
	{
		ref_point_index = path_points.size()-2;
		searchDir = -1;
	}
	else
	{
		float dis2ref  = pow(path_points[ref_point_index].x   - x, 2) + 
					     pow(path_points[ref_point_index].y   - y, 2);
		float dis2last = pow(path_points[ref_point_index-1].x - x, 2) + 
					     pow(path_points[ref_point_index-1].y - y, 2);
		float dis2next = pow(path_points[ref_point_index+1].x - x, 2) + 
					     pow(path_points[ref_point_index+1].y - y, 2);
		if(dis2next > dis2ref && dis2last > dis2ref) 
			searchDir = 0;
		else if(dis2next > dis2ref && dis2ref > dis2last)
			searchDir = -1;
		else
			searchDir = 1;
	}
	
	//std::cout  <<  "searchDir:"  << "\t" << searchDir << "\r\n";
	while(ref_point_index>0 && ref_point_index<path_points.size()-1)
	{
		float dis2ref  = pow(path_points[ref_point_index].x   - x, 2) + 
							pow(path_points[ref_point_index].y   - y, 2);
		float dis2last = pow(path_points[ref_point_index-1].x - x, 2) + 
							pow(path_points[ref_point_index-1].y - y, 2);
		float dis2next = pow(path_points[ref_point_index+1].x - x, 2) + 
							pow(path_points[ref_point_index+1].y - y, 2);
	//std::cout  << ref_point_index << "\t" <<  sqrt(dis2last)  << "\t" << sqrt(dis2ref) << "\t" << sqrt(dis2next) << "\r\n";		
		if((searchDir == 1 && dis2next > dis2ref) ||
		   (searchDir ==-1 && dis2last > dis2ref) ||
		   (searchDir == 0))
			break;

		ref_point_index += searchDir;
	}
	float anchor_x,anchor_y, anchor_yaw; //锚点的位置和航向
	anchor_x = path_points[ref_point_index].x;
	anchor_y = path_points[ref_point_index].y;
	anchor_yaw = path_points[ref_point_index].yaw;

	nearest_point_index = ref_point_index;
	//float dis2anchor = sqrt((x-anchor_x)*(x-anchor_x)+(y-anchor_y)*(y-anchor_y));
	float dx = (x-anchor_x)*cos(anchor_yaw) - (y-anchor_y) * sin(anchor_yaw);
	return dx;
}

/*@brief 计算目标点到达路径的距离,点在路径左侧为负,右侧为正
 *@brief 该函数主要用于计算目标到路径的距离,并考虑路径终点问题
 *@param x,y         目标点坐标
 *@param path_points 路径点集
 *@param ref_point_index 参考点索引，以此参考点展开搜索，加速计算
 *@param max_search_index 最大搜索索引,超出此索引的目标物则输出距离为FLT_MAX
 */
float calculateDis2path2(const double& x,const double& y,
						 const path_t& path_points, 
						 size_t  start_index, //参考点索引
						 size_t  end_index,
						 size_t& nearest_point_index)
{
	float min_dis2 = FLT_MAX;
	size_t output_index = start_index;
	for(int i=start_index; i<end_index; ++i)
	{
		float dis2  = pow(path_points[i].x  - x, 2) + pow(path_points[i].y  - y, 2);
		if(dis2 < min_dis2)
		{
			min_dis2 = dis2;
			output_index = i;
		}
	}

	nearest_point_index = output_index;
	
	float anchor_x,anchor_y, anchor_yaw; //锚点的位置和航向
	anchor_x = path_points[output_index].x;
	anchor_y = path_points[output_index].y;
	anchor_yaw = path_points[output_index].yaw;
	
	return (x-anchor_x)*cos(anchor_yaw) - (y-anchor_y) * sin(anchor_yaw);
}

bool is_gps_data_valid(gpsMsg_t& point)
{
	if(fabs(point.x) >100 && fabs(point.y) >100)
		return true;
	return false;
}

float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}

std::pair<float, float> get_dis_yaw(const gpsMsg_t &point1, const gpsMsg_t &point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	std::pair<float, float> dis_yaw;
	dis_yaw.first = sqrt(x * x + y * y);
	dis_yaw.second = atan2(y,x);
	
	if(dis_yaw.second <0)
		dis_yaw.second += 2*M_PI;
	return dis_yaw;
}

/*@brief findNearestPoint 在路径中查找距离当前位置最近的路径点
 *@param path             路径
 *@param current_point    当前位置
 *@return 最近点索引，距离
*/
std::pair<size_t,float> findNearestPoint(const path_t& path, const gpsMsg_t& current_point)
{
	int index = 0;
	float min_dis = FLT_MAX;
	float dis;
	
	for(size_t i=0; i<path.points.size(); )
	{
		float yaw_diff = path.points[i].yaw - current_point.yaw;

		if(yaw_diff > M_PI) yaw_diff-= 2*M_PI;
		else if(yaw_diff < -M_PI) yaw_diff += 2*M_PI;

		//printf("pathyaw: %.2f\t yaw: %.2f\t error:%.2f\r\n", path.points[i].yaw*180.0/M_PI, current_point.yaw*180.0/M_PI, yaw_diff*180.0/M_PI);
		if(fabs(yaw_diff) > M_PI/4)
		{
			++i;
			continue;
		}
			
		dis = dis2Points(path.points[i],current_point,true);
		// printf("i=%d\t dis:%f\n",i,dis);
		// printf("path.points[%d] x:%lf\t y:%lf\n",i,path.points[i].x,path.points[i].y);
		// printf("current_point  x:%lf\t y:%lf\n",current_point.x,current_point.y);
		if(dis < min_dis)
		{
			min_dis = dis;
			index = i;
		}
		i += int(dis)+1;
	}
	
	return std::make_pair(index,min_dis);
}

gpsMsg_t pointOffset(const gpsMsg_t& point,float offset)
{
	gpsMsg_t result = point; //copy all infos of point
	result.x =  offset * cos(point.yaw) + point.x;
	result.y = -offset * sin(point.yaw) + point.y;
	return result;
}

float generateRoadwheelAngleByRadius(const float& radius, const float& wheel_base)
{
	assert(radius!=0);
	//return asin(AXIS_DISTANCE /radius)*180/M_PI;  //the angle larger
	return atan(wheel_base/radius)*180/M_PI;    //correct algorithm 
}

/*@brief loadPath       载入路径文件
 *@param file_path      文件路径
 *@param path           路径点容器
 *@param interpolation  路径点插值
 *					    interpolation 为零时表示连续型路径，无需再插值
 *                      interpolation 不为零时表示顶点型路径，需差值
 */
bool loadPath(const std::string& file_path, path_t& path, float interpolation)
{
	path.points.clear();
	path.vertexes.clear();

	FILE *fp = fopen(file_path.c_str(),"r");
	
	if(fp==NULL)
	{
		printf("[loadPath] open %s failed\r\n",file_path.c_str());
		return false;
	}
	
	gpsMsg_t point;
	while(!feof(fp))
	{
		fscanf(fp,"%lf\t%lf\t%lf\n",&point.x,&point.y,&point.yaw);
		if(interpolation == 0.0)
			path.points.push_back(point);
		else
			path.vertexes.push_back(point);
	}
	fclose(fp);

	if(interpolation == 0.0)
		path.type = path.PathType_Curve;
	else
	{
		path.resolution = interpolation; 
		path.type = path.PathType_Vertex;
		path.generatePointsByVertexes(interpolation);
	}

	if(path.size() < 2)
		return false;
	return true;
}


