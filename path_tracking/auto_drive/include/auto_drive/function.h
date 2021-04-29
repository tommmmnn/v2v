#ifndef FUNCTION_H_
#define FUNCTION_H_

#include <vector>
#include <iostream>
#include <cmath>
#include <float.h>
#include <assert.h>
#include "structs.h"


float calculateDis2path(const double& x,const double& y,
						 const path_t & path_points, //path
						 size_t   ref_point_index, //参考点索引
						 size_t& nearest_point_index);

float calculateDis2path2(const double& x,const double& y,
						 const path_t& path_points, 
						 size_t  start_index, //参考点索引
						 size_t  end_index,
						 size_t& nearest_point_index);

bool is_gps_data_valid(gpsMsg_t& point);

float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt=true);

std::pair<float, float> get_dis_yaw(const gpsMsg_t &point1, const gpsMsg_t &point2);

std::pair<size_t,float>  findNearestPoint(const path_t& path, const gpsMsg_t& current_point);

gpsMsg_t pointOffset(const gpsMsg_t& point,float offset);

float generateRoadwheelAngleByRadius(const float& radius, const float& wheel_base);

bool loadPath(const std::string& file_path, path_t& path, float interpolation=0.0);

#endif
