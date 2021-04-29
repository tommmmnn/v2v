#ifndef TRANS_H_
#define TRANS_H_

#include<cmath>
#include"function.h"
#include<iostream>

/** 
 * @brief 位姿信息
 * Pose 作为对局部坐标系的描述时，yaw为其相对于全局坐标系的旋转角
 * Pose 作为对有向射线的描述时，yaw为其相对于坐标系y轴的旋转角
 */
typedef struct Pose
{
    float x,y,yaw;
    Pose(){}
    Pose(float _x,float _y,float _yaw)
    {
        x = _x;
        y = _y;
        yaw = _yaw;
    }
    Pose(const gpsMsg_t& point)
    {
        x = point.x;
        y = point.y;
        yaw = - point.yaw;
    }
    
    void print(const std::string& info = "pose")
    {
        std::cout << info << ": (" << x << "," << y << "," << yaw*180.0/M_PI <<"deg)" << std::endl;
    }
    Pose offset(float c)
    {
        Pose pose = *this;
        pose.x += c*cos(this->yaw);
        pose.y += c*sin(this->yaw);
        return pose;
    }

    void offsetInPlace(float c)
    {
        x += c*cos(yaw);
        y += c*sin(yaw);
    }
    
}pose_t;

/** 
 * @brief 位置信息
 */
typedef struct Position
{
    float x,y;

    Position(){}
    Position(float _x,float _y)
    {
        x = _x;
        y = _y;
    }
    void print(const std::string& info = "position")
    {
        std::cout << info << ": (" << x << "," << y << ")" << std::endl;
    }
    Position(const gpsMsg_t& point)
    {
        x = point.x;
        y = point.y;
    }
    Position(const pose_t& pose)
    {
        x = pose.x;
        y = pose.y;
    }
    
}position_t;


/** 
 * @brief 矩形框位姿以及尺寸
 */
typedef struct Rect
{
    pose_t pose;
    float x_len, y_len;

    //矩形的4个顶点坐标
    //按照象限存放
    std::vector<position_t> vertexes; 

    Rect(){}

    Rect(float _x,float _y, float _yaw, float _x_len, float _y_len)
    {
        pose.x = _x;
        pose.y = _y;
        pose.yaw = _yaw;
        x_len = _x_len;
        y_len = _y_len;
    }
    Rect(pose_t& _pose, float _x_len, float _y_len)
    {
        pose = _pose;
        x_len = _x_len;
        y_len = _y_len;
    }
    void inflat(float c)
    {
        x_len += 2*c;
        y_len += 2*c;
    }

    void calVertexes()
    {
        vertexes.resize(4);
        vertexes[0].x = x_len/2;
        vertexes[0].y = y_len/2;
        
        vertexes[1].x = -x_len/2;
        vertexes[1].y = y_len/2;

        vertexes[2].x = -x_len/2;
        vertexes[2].y = -y_len/2;

        vertexes[3].x = x_len/2;
        vertexes[3].y = -y_len/2;
    }
   
    
}rect_t;


/** 
 * @brief 全局坐标转换到局部坐标
 * @param global_position        目标点的全局坐标
 * @param coordinate_pose        局部坐标在全局坐标下的位姿
 *
 * @return 目标点的局部坐标
 */
inline position_t transform(const position_t& global_position, const pose_t& coordinate_pose)
{
    position_t dst_position;
    dst_position.x = +(global_position.x - coordinate_pose.x) * cos(coordinate_pose.yaw)
                     +(global_position.y - coordinate_pose.y) * sin(coordinate_pose.yaw);
    dst_position.y = -(global_position.x - coordinate_pose.x) * sin(coordinate_pose.yaw)
                     +(global_position.y - coordinate_pose.y) * cos(coordinate_pose.yaw);
    return dst_position;
}

/** 
 * @brief 全局坐标系下的位姿转换到局部坐标系下的位姿
 * @param global_pose        全局坐标系下的位姿
 * @param coordinate_pose    局部坐标在全局坐标下的位姿
 *
 * @return 局部坐标系下的姿态
 */

inline pose_t transform(const pose_t& global_pose, const pose_t& coordinate_pose)
{
    pose_t dst_pose;
    dst_pose.x = +(global_pose.x - coordinate_pose.x) * cos(coordinate_pose.yaw)
                 +(global_pose.y - coordinate_pose.y) * sin(coordinate_pose.yaw);
    dst_pose.y = -(global_pose.x - coordinate_pose.x) * sin(coordinate_pose.yaw)
                 +(global_pose.y - coordinate_pose.y) * cos(coordinate_pose.yaw);
    dst_pose.yaw = global_pose.yaw - coordinate_pose.yaw ;
    return dst_pose;
}

/** 
 * @brief 求两位置点间的距离
 * @param p1    位置1
 * @param p2    位置2
 *
 * @return 距离
 */
inline float distance(const position_t& p1, const position_t& p2)
{
    float delta_x = p1.x - p2.x;
    float delta_y = p1.y - p2.y;
    return sqrt(delta_x*delta_x + delta_y*delta_y); 
}

/** 
 * @brief 求点到有向射线的距离(有方向)
 * @param p    位置点
 * @param line 有向线段
 *
 * @return 带方向的距离
 */

 inline float distance(const position_t& p, const pose_t line)
 {
    return (p.x - line.x) * cos(line.yaw) + (p.y - line.y) * sin(line.yaw);
 }


/** 
 * @brief 判断点是否在矩形内
 * @param rect   矩形
 * @param point  点
 *
 * @return 判断结果
 */
inline bool inRect(const rect_t& rect, const position_t& point)
{
    position_t dst_point = transform(point, rect.pose);//将point转换到rect的坐标系下
    // dst_point.print();
    if(dst_point.x <= rect.x_len/2 && dst_point.x >= -rect.x_len/2 &&
       dst_point.y <= rect.y_len/2 && dst_point.y >= -rect.y_len/2)
       return true;
    return false;
}


#endif
