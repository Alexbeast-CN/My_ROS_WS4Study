/* 
需求：
    生成5只小乌龟，
    小乌龟分别处于(4,4),(3,3),(2,2),(1,1)
    以及由turtlesim_node自动生成的小乌龟。乌龟处在（5.54，5.54）

实现：
    通过命令行查询到，生成小乌龟使用了：服务通讯
    通过获取的消息信息可知，生成小乌龟需要使用以下的输入参数：

    float32 x
    float32 y
    float32 theta
    string name
            
*/

#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"turtle_spawn");
    ros::NodeHandle nh;

    // 创建 service 的客户端
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    // 等待服务启动：
    ros::service::waitForService("/spawn");
    
    std::string str1 = "turtle";
    // 发送请求：
    for (int i = 1; i < 5; i++)
    {
        std::string str2 = std::to_string(i+1);
        std::string turtle_name = str1 + str2;
        turtlesim::Spawn spawn;
        spawn.request.x = 5.54 - i;
        spawn.request.y = 5.54 - i;
        spawn.request.theta = 0;
        spawn.request.name = turtle_name;
        bool flag = client.call(spawn);

    // 处理响应结果
        if (flag)
        {
            ROS_INFO("新的乌龟生成,名字:%s",spawn.response.name.c_str());
        } else {
            ROS_INFO("乌龟生成失败！！！");
        }
    }

    return 0;
}


