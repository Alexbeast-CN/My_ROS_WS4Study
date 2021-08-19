/*
    需求：让5只海龟画圆

    实现：对每一个海龟循环发布运动信息，消息类型是：

    geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
    geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z
*/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"turtle_move");
    ros::NodeHandle nh;
    geometry_msgs::Twist msg[5];
    ros::Publisher pub[5];

    // 分别为5只海龟设置控制
    for (int i = 0; i < 5; i++)
    {
        std::string str1 = "/turtle";
        std::string str2 = "/cmd_vel";
        std::string str3 = std::to_string(i+1);
        std::string topic_name = str1 + str3 + str2;

        // 创建发布者对象
        pub[i] = nh.advertise<geometry_msgs::Twist>(topic_name,100);
        // 组织消息
        
        msg[i].linear.x = 1.0;
        msg[i].linear.y = 0.0;
        msg[i].linear.z = 0.0;

        msg[i].angular.x = 0.0;
        msg[i].angular.y = 0.0;
        msg[i].angular.z = 1.0;

        //设置发送频率
        ros::Rate r(1);

    }

    //循环发送
    while (ros::ok())
    {
        for (int j = 0; j < 5; j++)
            pub[j].publish(msg[j]);

        ros::spinOnce();
    }
return 0;
}
