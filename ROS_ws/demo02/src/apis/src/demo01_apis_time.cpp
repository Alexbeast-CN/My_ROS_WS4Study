#include "ros/ros.h"

/*
    需求1：
        演示时间相关的操作：获取当前时刻 + 设定制定时刻

    实现：
        1. 准备（头文件、节点初始化、NodeHandle创建...）
        2. 获取当前时刻
        3. 设置指定时刻

    需求2：
        让程序执行中停顿 5 秒钟

    实现：
        1. 创建持续时间对象；
        2. 休眠。

    需求3：
        对时间进行运算。
    
    实现：
        1. ros::Time 之间的减法
        2. ros::Duration  之间的加减
        3. Time 与 Duration 之间的加减

    需求4：
        每隔1秒，在控制台输出一段文本。
    
    实现：
        1. 利用ros::rate() 学过了
        2. 利用定时器
            创建： nh.createTimer()
            参数1： 时间间隔
            参数2： 回调函数（时间事件 TimerEvent）
            参数3： 是否只执行一次
            参数4： 是否自动启动（默认是true，如需手动启动使用 timer.start())

            定时器启动后： ros::spin()

*/

// 回调函数
void cb(const ros::TimerEvent& event)
{
    ROS_INFO("-----------------");
    ROS_INFO("函数被调用的时刻:%.2f",event.current_real.toSec());
}

int main(int argc, char *argv[])
{
    // 1. 准备
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"Hello_time");
    ros::NodeHandle nh; // 与时间相关的函数必须有NodeHandle

    // 2. 获取当前时刻
    // now 函数会将当前时刻封装并返回
    // 当前时刻：是指 now 被调用执行的那一刻
    // 参考系： 1970年01月01日 00：00：00 中国是：GMT+8
    // 输出为距离参考系的时间
    ros::Time right_now = ros::Time::now();
    ROS_INFO("当前时刻：%.2f",right_now.toSec());
    ROS_INFO("当前时刻：%d",right_now.sec);
    

    // 3. 设置指定时刻
    ros::Time t1(20,312345678);
    ROS_INFO("t1 = %.2f",t1.toSec());

    // 持续时间
    ROS_INFO("---持续时间---");
    ros::Time start = ros::Time::now();
    ROS_INFO("开始休眠");

    // 设置睡眠时间
    ros::Duration(4).sleep();
    ros::Time end = ros::Time::now();
    ROS_INFO("休眠结束，共%.2f秒", end.toSec()-start.toSec());


    // 时间之间的运算
    ROS_INFO("---时间运算---");
    ros::Time now = ros::Time::now();
    ros::Duration du1(10);
    ros::Duration du2(20);
    ROS_INFO("当前时刻:%.2f",now.toSec());
    
    //1.time 与 duration 运算
    ros::Time after_now = now + du1;
    ros::Time before_now = now - du1;
    ROS_INFO("当前时刻之后:%.2f",after_now.toSec());
    ROS_INFO("当前时刻之前:%.2f",before_now.toSec());

    
    //2.duration 之间相互运算
    ros::Duration du3 = du1 + du2;
    ros::Duration du4 = du1 - du2;
    ROS_INFO("du3 = %.2f",du3.toSec());
    ROS_INFO("du4 = %.2f",du4.toSec());

    //PS: time 与 time 不可以相加运算
    // ros::Time nn = now + before_now;//异常
    ros::Duration du5 = now - before_now;
    ROS_INFO("时刻相减:%.2f",du5.toSec());

    // 定时器
    ROS_INFO("---定时器---");
    // ros::Timer createTimer(ros::Duration period,     // 设置间隔时间
    //          const ros::TimerCallback &callback,     // 回调函数 -- 封装业务
    //          bool oneshot = false,                   // 是否是一次性
    //          bool autostart = true)                  // 自动回调

    // ros::Timer timer = nh.createTimer(ros::Duration(1),cb);
    // ros::Timer timer = nh.createTimer(ros::Duration(1),cb,true);
    ros::Timer timer = nh.createTimer(ros::Duration(1),cb,false,false);
    timer.start(); //手动启动定时器

    ros::spin();

    return 0;
}
