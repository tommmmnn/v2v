//#include "../include/auto_drive/trans.h"
#include<iostream>
#include<cmath>
using namespace std;

class Test
{
public:
	Test()
	{
		status_ = false;
	}
	bool status() const
	{
		return status_;
	}
private:
	bool status_;
};

void fun(const Test& test)
{
	cout << test.status() << endl;
}

//position_t transform(const position_t& global_position, const pose_t& coordinate_pose);

int main()
{
//	position_t src_point={2.0,5.0};
//	pose_t dst_pose = {1.0,5.0,-45.0/180.0*M_PI};
//	position_t dst_point = transform(src_point, dst_pose);
//	dst_point.print();
	
//	pose_t src_pose = {1,5,-45.0/180.0*M_PI};
//	pose_t cordi_pose = {2,1, 45.0/180.0*M_PI};
//	pose_t dst_pose = transform(src_pose, cordi_pose);
//	dst_pose.print();
	
//	position_t point={4,4};
//	pose_t line={6,0,45.0/180.0*M_PI};
//	cout << distance(point, line) << endl;;
//	
//	point.x = 2;
//	point.y = 2;
//	
//	cout << distance(point, line)<< endl;;
//	
//	point.x = 3;
//	point.y = 3;
//	
//	cout << distance(point, line)<< endl;;
	
	//pose_t pose={3,3,90.0/180.0*M_PI};
	//pose.offset(-1.0).print();
	
	Test test;
	fun(test);
}

