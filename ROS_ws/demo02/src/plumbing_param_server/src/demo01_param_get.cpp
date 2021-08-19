#include "ros/ros.h"
/*
    演示参数查询

    实现：
        ros::NodeHandle------------------------------------
              param(键,默认值) 
            存在，返回对应结果，否则返回默认值

            getParam(键,存储结果的变量)
            存在,返回 true,且将值赋值给参数2
            若果键不存在，那么返回值为 false，且不为参数2赋值

            getParamCached键,存储结果的变量)--提高变量获取效率
            存在,返回 true,且将值赋值给参数2
            若果键不存在，那么返回值为 false，且不为参数2赋值

            getParamNames(std::vector<std::string>)
            获取所有的键,并存储在参数 vector 中 

            hasParam(键)
            是否包含某个键，存在返回 true，否则返回 false

            searchParam(参数1，参数2)
            搜索键，参数1是被搜索的键，参数2存储搜索结果的变量

            ros::param--------------------------------------
            与上面的类似
*/
int main(int argc, char *argv[])
{
    // 设置编码
    setlocale(LC_ALL,"");
    // 初始化节点
    ros::init(argc,argv,"get_param_C");
    // 创建节点句柄
    ros::NodeHandle nh;

    //ros::NodeHandle------------------------------------
    // 1. param
    double r = nh.param("r2",0.5);
    ROS_INFO("r = %.2f",r);
    
    ROS_INFO("----------------------------\n");


    // 2. getParam
    double r2 = 0.0;
    bool result = nh.getParam("r",r2);
    if (result)
        ROS_INFO("获取的半径是：%.2f",r2);
    else
        ROS_INFO("查询值不存在！");

    ROS_INFO("----------------------------\n");


    // 3. getParamCached(先找缓存区，再看RPC)，性能比getParam高，功能与getParam类似。
    double r3 = 0.0;
    bool result2 = nh.getParamCached("ra",r3);
    if (result2)
        ROS_INFO("获取的半径是：%.2f",r2);
    else
        ROS_INFO("查询值不存在！");
    ROS_INFO("----------------------------\n");
    

    // 4. getParamNames
    std::vector<std::string> param_names1;
    nh.getParamNames(param_names1);
    ROS_INFO("获取到的信息有：");
    for (auto &&name : param_names1)
    {
        ROS_INFO("%s",name.c_str());        
    }
    ROS_INFO("----------------------------\n");

    // 5. hasParam(建)
    bool flag1 = nh.hasParam("radius");
    bool flag2 = nh.hasParam("r2");
    ROS_INFO("radius 存在吗？ %d",flag1);
    ROS_INFO("r2 存在吗？ %d",flag2);
    ROS_INFO("----------------------------\n");


    // 6. searchParam
    std::string key;
    nh.searchParam("name",key);
    ROS_INFO("搜索结果是：%s",key.c_str());
    ROS_INFO("----------------------------\n");


 return 0;   
}
