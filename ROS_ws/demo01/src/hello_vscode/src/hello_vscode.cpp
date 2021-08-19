#include "ros/ros.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 执行初始化节点
    ros::init(argc,argv,"Hello_vscode");
    //输出日志
    ROS_INFO("Hello vscode!!! yeah!");

    return 0;
}
