#include "ros/ros.h"

/* 
    演示参数的删除：

    实现：
        ros::NodeHandle
            deleteParam()
        ros::param
            del()

*/
int main(int argc, char *argv[])
{
        // 删除：NodeHandle ----------------------
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"Delete");
    ros::NodeHandle nh;

    bool flag1 = nh.deleteParam("r");
    if (flag1)
        ROS_INFO("删除成功！");
    else
        ROS_INFO("删除失败！");

    // 删除：ros::param ----------------------
    bool flag2 = ros::param::del("name");
    if (flag2)
        ROS_INFO("删除成功！");
    else
        ROS_INFO("删除失败！");
        
    return 0;
}
