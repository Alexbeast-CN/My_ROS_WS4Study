#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "std_msgs/String.h"

/*
    需要：使用 rosbag 向磁盘文件读取数据 （话题 + 消息）

    流程：
        1. 导包
        2. 初始化
        3. 创建 rosbag 对象
        4. 打开文件流（以读取的方式打开）
        5. 写数据
        6. 关闭文件流 
*/

int main(int argc, char *argv[])
{
    // 2. 初始化
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"bag_read");
    ros::NodeHandle nh;
    // 3. 创建 rosbag 对象
    rosbag::Bag bag;
    // 4. 打开文件流（以读取的方式打开）
    bag.open("hello.bag",rosbag::BagMode::Read);
    // 5. 写数据
    // 取出话题，时间辍和消息
    // 可以县获取消息的集合，再迭代取出消息的相关字段
    for (auto &&m : rosbag::View(bag))
    {
        // 解析
        std::string topic =  m.getTopic();
        ros::Time time = m.getTime();
        std_msgs::StringPtr p =  m.instantiate<std_msgs::String>();
        ROS_INFO("解析的内容，话题：%s，时间辍：%.2f，消息：%s",
                topic.c_str(),
                time.toSec(),
                p->data.c_str());
    }
    
    // 6. 关闭文件流
    bag.close();
    return 0;
}
