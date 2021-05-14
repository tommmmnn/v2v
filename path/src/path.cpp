#include "ros/ros.h"
#include "unistd.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>

#define MAX_OFFSET -2.5

using namespace std;

class Path
{
public:
	Path();
	~Path();
	bool init();
	void linear_Process();
	bool get_Offset(double deltax, float &offset);
	void poly_Process();
private:
	ros::NodeHandle nh, priv_nh;

	ifstream infile;
	ofstream outfile;
	
	string in;
	string out;
	string file_path;
	string in_file_name;
	string out_file_name;

	std::vector<double> x, y, yaw;
};

Path::Path():
	priv_nh("~")
{
	
}

Path::~Path()
{

}

bool Path::init()
{
	file_path = priv_nh.param<std::string>("file_path","");
	in_file_name = priv_nh.param<std::string>("in_file_name","");
	out_file_name = priv_nh.param<std::string>("out_file_name","2_101.txt");
	
	if(file_path.empty() || in_file_name.empty())
	{
		ROS_ERROR("please input record file path and file name in launch file!");
		return false;
	};

	in.append(file_path);			//使用append()函数将路径命和文件名组合
	out.append(file_path);
	in.append("/");
	out.append("/");
	in.append(in_file_name);
	out.append(out_file_name);

	cout << in << '\t' << out << endl;

	infile.open(in.c_str(), ios::in); 		//open()的参数不能使用string类，因此通过c_str转换成字符串
	if(!infile.is_open())
		ROS_ERROR("Open input file falure!!");

	double x_t, y_t, yaw_t;
	while(!infile.eof())
	{
		infile >> x_t >> y_t >> yaw_t;
		x.push_back(x_t);
		y.push_back(y_t);
		yaw.push_back(yaw_t);
	}

	infile.close();
	return true;
}

void Path::linear_Process()
{
	//每两个数之间的间隔
	double x_revolution = (*(x.end()-1) - *x.begin()) / (x.size()-1);
	double y_revolution = (*(y.end()-1) - *y.begin()) / (y.size()-1);

	double theta = atan2(*(y.end() - 1) - *y.begin(), *(x.end() - 1) - *x.begin());		//atan2()取值范围为[-pi, pi]

	// if(theta<0) theta += 2 * M_PI;

	double yaw_road = M_PI/2 - theta;			//角度变换成GPS的航向角

	if(yaw_road < 0)	yaw_road += 2 * M_PI;		//取值范围变成[0, 2*pi]

	//覆盖原数组
	for(size_t i=0; i<x.size(); i++)
    {
		*(x.begin()+i) = *x.begin() + x_revolution * i;
		*(y.begin()+i) = *y.begin() + y_revolution * i;
		*(yaw.begin()+i) = yaw_road;
	}



	outfile.open(out.c_str(), ios::out);   //用ios::out覆盖原文件的内容，若增添则用ios::app
	if(!outfile.is_open())
		ROS_ERROR("Open input file falure!!");

	for (size_t i = 0; i<x.size(); i++)
	{
		//输出小数点后三位，固定为浮点输出
		outfile << setiosflags(ios::fixed) << setprecision(3) << x[i] << "\t" << setprecision(3) << y[i] << "\t" << setprecision(3) << yaw[i] << endl;
	}
	outfile.close();

	ROS_INFO("linear process finished!");

}

bool Path::get_Offset(double deltax, float &offset)
{
	// y = c0 + c1*x + c2 * x^2 + c3* x^3
	float x_ad, c0, c1, c2, c3;
	double x = abs(deltax);
	x_ad = 15;
	if(x >= x_ad) return true;
	else
	{
		c0 = 0;
		c1 = 0;
		c2 = (3 * MAX_OFFSET - 2 * c1 * x_ad - 3 * c0) / (x_ad * x_ad);
		c3 = (2 * c0 + c1 * x_ad - 2 * MAX_OFFSET) / (x_ad * x_ad * x_ad);
		offset = c0 + c1 * x + c2 * x * x + c3 * x * x * x;
		return false;
	}
}

void Path::poly_Process()
{
	//每两个数之间的间隔
	double x_revolution = (*(x.end()-1) - *x.begin()) / (x.size()-1);
	double y_revolution = (*(y.end()-1) - *y.begin()) / (y.size()-1);

	double theta = atan2(*(y.end() - 1) - *y.begin(), *(x.end() - 1) - *x.begin());		//atan2()取值范围为[-pi, pi]

	// if(theta<0) theta += 2 * M_PI;

	double yaw_road = M_PI/2 - theta;			//角度变换成GPS的航向角

	if(yaw_road < 0)	yaw_road += 2 * M_PI;		//取值范围变成[0, 2*pi]

	for (size_t i=0; i<x.size(); i++)
	{
		*(x.begin()+i) = *x.begin() + x_revolution * i;
		*(y.begin()+i) = *y.begin() + y_revolution * i;
		*(yaw.begin()+i) = yaw_road;
	}

	//覆盖原数组
	for(size_t i=0; i<x.size(); i++)
    {
		if(i > 200)
		{
			float offset = 0;
			float x_tmp = *(x.begin()+i);
			float y_tmp = *(y.begin()+i);
			float x1 = MAX_OFFSET*sin(theta);
			float y1 = MAX_OFFSET*cos(theta);
			float deltax = sqrt(pow(x_revolution * (i-200) ,2) + pow(y_revolution * (i-200) ,2));
			// ROS_ERROR("x_tmp y_tmp is %f %f", x_tmp, y_tmp);
			if(!this->get_Offset(deltax, offset))
			{
				// ROS_ERROR("offset is %f", offset);
				*(x.begin()+i) += -offset*sin(theta);
				*(y.begin()+i) -= offset*cos(theta);
				float theta_tmp = atan2(*(y.begin()+i) - *(y.begin()+i-1), *(x.begin()+i) - *(x.begin()+i-1));
				if(M_PI/2 - theta_tmp < 0)
				{
					*(yaw.begin()+i) = M_PI/2 - theta_tmp + 2 * M_PI;
				}
				else *(yaw.begin()+i) = M_PI/2 - theta_tmp;
			}
			else
			{
				*(x.begin()+i) += x1;
				*(y.begin()+i) -= y1;
				*(yaw.begin()+i) = yaw_road;
			}
		}
	}

	outfile.open(out.c_str(), ios::out);   //用ios::out覆盖原文件的内容，若增添则用ios::app
	if(!outfile.is_open())
		ROS_ERROR("Open input file falure!!");

	for (size_t i = 0; i<x.size(); i++)
	{
		//输出小数点后三位，固定为浮点输出
		outfile << setiosflags(ios::fixed) << setprecision(3) << x[i] << "\t" << setprecision(3) << y[i] << "\t" << setprecision(3) << yaw[i] << endl;
	}
	outfile.close();

	ROS_INFO("poly process finished!");

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "transform_path_node");
	Path my_path;
	if(my_path.init())
		ROS_INFO("loading file ok!");
	my_path.linear_Process();

	return 0;
}
