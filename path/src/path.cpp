#include "ros/ros.h"
#include "unistd.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace std;

class Path
{
public:
	Path();
	~Path();
	bool init();
	void linear_Process();
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "transform_path_node");
	Path my_path;
	if(my_path.init())
		ROS_INFO("loading file ok!");
	my_path.linear_Process();

	return 0;
}
