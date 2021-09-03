#include "ros/ros.h"
#include "turtlesim/Pose.h"
// 用来发布动态坐标
#include "tf2_ros/transform_broadcaster.h"
// 我们使用到的信息类型
#include "geometry_msgs/TransformStamped.h"
// 用于将欧拉角转换为四元数
#include "tf2/LinearMath/Quaternion.h"

/*
    发布方：需要订阅乌龟的为姿信息，转换成相对于世界坐标系的坐标关系，并发布。

    准 备：
        话题：/turtle1/pose
        消息：turtlesim/Pose

    流程：
        1. 包含头文件；
        2. 初始化；
        3. 创建订阅对象，订阅 /turtle1/pose;
        4. 回调函数处理订阅的消息: 将位姿信息转换成坐标相对关系并发布（关注）
        5. spin（）

*/

// 4. 回调函数处理订阅的消息: 将位姿信息转换成坐标相对关系并发布（关注）
void doPose(const turtlesim::Pose::ConstPtr & pose)
{
    // 获取位姿信息，转换成坐标系相对关系（核心），并发布
    // a. 创建TF一个发布对象
    // 与静态发布的主要区别就在于下面的这个语句。
    static tf2_ros::TransformBroadcaster pub;
    // b. 组织被发布的数据
    geometry_msgs::TransformStamped ts;
    {
        ts.header.frame_id = "world";
        ts.header.stamp = ros::Time::now();
        ts.child_frame_id = "turtle1";

        // 坐标系偏移量设置
        ts.transform.translation.x = pose->x;
        ts.transform.translation.y = pose->y;
        ts.transform.translation.z = 0;
        // 坐标系四元数
        /*
            位姿信息中没有四元数，但是有一个偏航角度
            已知乌龟是 2D 的，没有翻滚和俯仰角，所以
            可以得出乌龟的欧拉角为： 0 0 theta
        */
       tf2::Quaternion qtn;
       qtn.setRPY(0, 0, pose->theta);
       ts.transform.rotation.x = qtn.getX();
       ts.transform.rotation.y = qtn.getY();
       ts.transform.rotation.z = qtn.getZ();
       ts.transform.rotation.w = qtn.getW();
    }
    
    // c. 发布
    pub.sendTransform(ts);
}

int main(int argc, char *argv[])
{
    // 2. 初始化；
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"dynamic_pub");
    ros::NodeHandle nh;
    // 3. 创建订阅对象，订阅 /turtle1/pose;
    ros::Subscriber sub = nh.subscribe("turtle1/pose",100,doPose);
    // 4. 回调函数处理订阅的消息: 将位姿信息转换成坐标相对关系并发布（关注）
    // 5. spin（）
    ros::spin();

    return 0;
}
