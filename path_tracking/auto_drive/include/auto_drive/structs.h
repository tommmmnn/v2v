#ifndef STRUCTS_H_
#define STRUCTS_H_

typedef struct
{
	double longitude;
	double latitude;
	double yaw;
	double x;
	double y;
	float curvature;
	float maxOffset_left;
	float maxOffset_right;

	void reset()
	{
		longitude = latitude = yaw = x = y 
				  = curvature = maxOffset_left 
				  = maxOffset_right = 0.0;
	}
}gpsMsg_t;

typedef struct
{
	bool flag; // invalid ? validity
	float roadwheel_angle;
	float speed;
	float brake;
}controlMsg_t;

typedef struct
{
	std::vector<gpsMsg_t> vertexes;
	std::vector<gpsMsg_t> points;
	float resolution; //分辨率,即路径点间的间距
	void clear()
	{
		points.clear();
		vertexes.clear();
	}
	size_t size() const
	{
		return points.size();
	}

	//下标索引重载
	const gpsMsg_t& operator[](size_t i) const
	{
		return points[i];
	}

	enum PathType
	{
		PathType_Vertex,
		PathType_Curve,
	};
	uint8_t type;

	bool generatePointsByVertexes(float increment)
	{
		if(vertexes.size()<2)
			return false;
		
		for(int i=1; i<vertexes.size(); ++i)
		{
			gpsMsg_t startPoint = vertexes[i-1];
			gpsMsg_t endPoint = vertexes[i];

			float delta_x = endPoint.x - startPoint.x;
			float delta_y = endPoint.y - startPoint.y;
			float distance = sqrt(delta_x*delta_x + delta_y*delta_y);

			double yaw = atan2(delta_y, delta_x); 
			int point_cnt = distance / increment;
			
			double x_increment = (endPoint.x - startPoint.x)/point_cnt;
			double y_increment = (endPoint.y - startPoint.y)/point_cnt;
			
			for(int j=0; j<point_cnt; ++j)
			{
				gpsMsg_t now;
				now.x = startPoint.x + x_increment*j;
				now.y = startPoint.y + y_increment*j;
				now.yaw = yaw;
				points.push_back(now);
			}
		}
		if(points.size()==0)
			return false;
		return true;
	}

}path_t;


#endif



