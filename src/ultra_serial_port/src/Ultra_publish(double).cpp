/**
 * 该例程将发布/person_info话题，自定义消息类型learning_topic::Person
 */
 
#include <ros/ros.h>
#include "Ultra_serial_port/Ultrasound.h"

#include <serial/serial.h>
#include <iostream>

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "Ultrasound_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
    ros::Publisher Ultrasound_pub = n.advertise<Ultra_serial_port::Ultrasound>("/Ultrasound_info", 10);


    //创建一个serial类
    serial::Serial sp;

    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");

    //设置串口通信的波特率
    sp.setBaudrate(9600);

    //串口设置timeout
    sp.setTimeout(to);

    try
    {
        //打开串口
        sp.open();

    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }

//********串口2
    serial::Serial sp2;
    sp2.setPort("/dev/ttyUSB1");
    sp2.setBaudrate(9600);
    sp2.setTimeout(to);  

    try
    {
        sp2.open();

    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port2.");
        return -1;
    }  


    if(sp2.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB1 is opened.");
    }
    else
    {
        return -1;
    }


    // 设置循环的频率10=10Hz
    ros::Rate loop_rate(10);

int dis=0;
int dis2=0;

    //unsigned char rcv_buf[24];


    while (ros::ok())
    {

        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t rcv_buf[128];
            //读出数据
            n = sp.read(rcv_buf, n);


/*		for(int i=0; i<n; i++)
            {
                //16进制的方式打印到屏幕
                std::cout << std::hex << (rcv_buf[i] & 0xff) << " ";
            }
*/

		if ((rcv_buf[0]==0x68)&&(rcv_buf[1]==0x05)&&(rcv_buf[2]==0x01))
		{
			dis=rcv_buf[3]*0xFF+rcv_buf[4];
		}
		else if((rcv_buf[1]==0x68)&&(rcv_buf[2]==0x05)&&(rcv_buf[3]==0x01))
		{
			dis=rcv_buf[4]*0xFF+rcv_buf[5];
		}
		else if((rcv_buf[2]==0x68)&&(rcv_buf[3]==0x05)&&(rcv_buf[4]==0x01))
		{
			dis=rcv_buf[5]*0xFF+rcv_buf[6];
		}
		else if((rcv_buf[3]==0x68)&&(rcv_buf[4]==0x05)&&(rcv_buf[4]==0x01))
		{
			dis=rcv_buf[6]*0xFF+rcv_buf[7];
		}


        }
//获取缓冲区内的字节数
        size_t n2 = sp2.available();
        if(n2!=0)
        {
            uint8_t rcv_buf2[128];
            //读出数据
            n2 = sp2.read(rcv_buf2, n2);

		if ((rcv_buf2[0]==0x68)&&(rcv_buf2[1]==0x05)&&(rcv_buf2[2]==0x01))
		{
			dis2=rcv_buf2[3]*0xFF+rcv_buf2[4];
		}
		else if((rcv_buf2[1]==0x68)&&(rcv_buf2[2]==0x05)&&(rcv_buf2[3]==0x01))
		{
			dis2=rcv_buf2[4]*0xFF+rcv_buf2[5];
		}
		else if((rcv_buf2[2]==0x68)&&(rcv_buf2[3]==0x05)&&(rcv_buf2[4]==0x01))
		{
			dis2=rcv_buf2[5]*0xFF+rcv_buf2[6];
		}
		else if((rcv_buf2[3]==0x68)&&(rcv_buf2[4]==0x05)&&(rcv_buf2[4]==0x01))
		{
			dis2=rcv_buf2[6]*0xFF+rcv_buf2[7];
		}
        } 

        // 初始化learning_topic::Person类型的消息
    	Ultra_serial_port::Ultrasound Ultrasound_msg;
		Ultrasound_msg.dis = dis;
		Ultrasound_msg.dis2 = dis2;

        // 发布消息
		Ultrasound_pub.publish(Ultrasound_msg);

       		ROS_INFO("Ultrasound_info: dis=%d", Ultrasound_msg.dis);
       		ROS_INFO("Ultrasound_info: dis2=%d", Ultrasound_msg.dis2);

        // 按照循环频率延时
        loop_rate.sleep();
    }
    //关闭串口
    sp.close();
    sp2.close();
    return 0;
}


