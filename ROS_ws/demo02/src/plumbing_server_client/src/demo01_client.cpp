#include "ros/ros.h"
#include "plumbing_server_client/Addints.h"

/* 
    客户端：提交两个整数，并处理相应的结果。

        1. 包含头文件；
        2. 初始化ROS节点；
        3. 创建节点句柄；
        4. 创建一个客户端对象；
        5. 提交请求并处理响应； 

    实现参数的动态提交：
        1. 格式： rosrun xxx xxx 12 32
        2. 节点执行时，需要获取命令中的参数，并组织进 request

    问题：
        如果先启动客户端，那么就会请求异常
    需求：
        如果先启动客户端，不要直接抛出异常，而是挂起，等在服务器启动后再正常请求
    解决：
        ROS中的内置函数可以让客户端启动后挂起，等待服务器启动。
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 优化实现，获取命令中的参数：
    if(argc != 3)
    {
        ROS_INFO("提交的参数数量不正确！");
        return 1;
    }

    //     2. 初始化ROS节点；
    ros::init(argc,argv,"Dabao");
    //     3. 创建节点句柄；
    ros::NodeHandle nh;
    //     4. 创建一个客户端对象；
    ros::ServiceClient client = nh.serviceClient<plumbing_server_client::Addints>("Addints");
    //     5. 提交请求并处理响应； 
    plumbing_server_client::Addints ai;
    //     5-1. 组织请求
    ai.request.num1 = atoi(argv[1]);
    ai.request.num2 = atoi(argv[2]);
    //     5-2. 处理响应
    //     调用判断服务器状态的函数
    //     函数1
    //client.waitForExistence();
    //     函数2
    ros::service::waitForService("Addints");
    bool flag = client.call(ai);
    if (flag)
    {
        ROS_INFO("响应成功！");
        // 获取结果
        ROS_INFO("两数之和是：%d",ai.response.sum);
    }
    else
        ROS_INFO("处理失败！");
    return 0;
}
