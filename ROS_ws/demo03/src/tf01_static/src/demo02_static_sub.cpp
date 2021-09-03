#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
/*
    订阅方：订阅发布的坐标系相对关系，传入一个坐标点，调用 tf 实现转换

    流程：
        1. 包含头文件；
        2. 初始化（编码，节点，NodeHandle）
        3. 创建订阅对象； --> 订阅坐标系相对关系
        4. 组织一个座标点数据；
        5. 转化算法，调用tf内置实现；
        6. 最后输出。
*/

int main(int argc, char *argv[])
{
    // 2. 初始化（编码，节点，NodeHandle）
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"Static_sub");
    ros::NodeHandle nh;
    // 3. 创建订阅对象； --> 订阅坐标系相对关系
    // 3-1. 创建一个 buffer 缓存
    tf2_ros::Buffer buffer;
    // 3-2. 创建一个监听对象（将订阅的数据缓存到 buffer）
    tf2_ros::TransformListener listener(buffer);
    // 4. 组织一个座标点数据；
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "laser";
    ps.header.stamp = ros::Time::now();
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;

    // 休眠
    // ros::Duration(2).sleep();
    // 5. 转化算法，调用tf内置实现；
    ros::Rate rate(10);
    while(ros::ok())
    {
        // 核心代码 -- 将 ps 转换成相对于 base_link 的坐标点
        geometry_msgs::PointStamped ps_out;
        /*
            调用了 buffer 的转换函数 transform
            参数1：被转化的座标点
            参数2：目标坐标系
            返回值：输出的坐标点

            PS1：调用时必须包含头文件 tf2_geometry_msgs/tf2_geometry_msgs.h
            PS2: 运行时存在的问题，抛出异常 base_link 不存在
                 原因： 订阅数据是一个耗时操作，可能在调用 transform 转换函数时，
                        坐标系的相对关系还没订阅到，因此出现异常
                 解决：
                       方案1：在调用转换函数前，执行休眠；
                       方案2： 进行异常处理(建议使用)。


        */

       try
       {
           
        ps_out =  buffer.transform(ps,"base_link");
        // 6. 最后输出
        ROS_INFO("转换后的坐标值:(%.2f,%.2f,%.2f),参考坐标系: %s",
                        ps_out.point.x,
                        ps_out.point.y,
                        ps_out.point.z,
                        ps_out.header.frame_id.c_str()
                        );
       }
       catch(const std::exception& e)
       {
        //    std::cerr << e.what() << '\n';
        ROS_INFO("异常消息：%s",e.what());
       }
       
       
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
