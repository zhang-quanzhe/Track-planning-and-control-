/**
 * 该例程将订阅/person_info话题，自定义消息类型learning_topic::Person
 */

#include <ros/ros.h>
#include "/home/ustb/xxjcar/devel/include/ultra_serial_port/Ultrasound.h"


// 接收到订阅的消息后，会进入消息回调函数

void personInfoCallback(const ultra_serial_port::Ultrasound::ConstPtr& Ultrasound_msg)
{
    // 将接收到的消息打印出来
    ROS_INFO("Ultrasound_info: dis=%d", Ultrasound_msg->dis);
}



int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "Ultrasound_subscriber");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
   ros::Subscriber person_info_sub = n.subscribe("/Ultrasound_info", 10, personInfoCallback);

    // 循环等待回调函数
    ros::spin();

    return 0;

}
