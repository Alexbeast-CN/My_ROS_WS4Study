#include "ros/ros.h"
#include "turtlesim/Color.h"

/*
    需求：修改参数服务器中 turlesim 背景色相关的参数

    1. 初始化ROS节点
    2. 不一定需要创建节点句柄（和后续API有关）；
    3. 修改参数。
*/

int main(int argc, char *argv[])
{
    // 1. 初始化ROS节点
    ros::init(argc,argv,"changeColor");
    // 2. 不一定需要创建节点句柄（和后续API有关）；
    // ros::NodeHandle nh("turtlesim");
    // nh.setParam("background_r",255);
    // nh.setParam("background_g",255);
    // nh.setParam("background_b",255);


    // 3. 修改参数。
    // ros::param::set("/turtlesim/background_r",10);
    // ros::param::set("/turtlesim/background_g",10);
    // ros::param::set("/turtlesim/background_b",10);

    return 0;
}
