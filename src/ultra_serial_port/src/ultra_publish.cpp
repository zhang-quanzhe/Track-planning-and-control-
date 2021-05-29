/**
 * 该例程将发布/person_info话题，自定义消息类型learning_topic::Person
 */
 
#include <ros/ros.h>
#include "ultra_serial_port/Ultrasound.h"

#include <serial/serial.h>
//#include <iostream>

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "Ultrasound_publisher");

    // 创建节点句柄
    ros::NodeHandle nu;

    // 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
    ros::Publisher ultrasound_pub = nu.advertise<ultra_serial_port::Ultrasound>("/Ultrasound_info", 1);

    std::string ultra_uart_port;
    int ultra_uart_baud_rate;
    nu.param<std::string>("ultra_uart_port", ultra_uart_port, "/dev/ttyUSB0");
    nu.param<int>("ultra_uart_baud_rate", ultra_uart_baud_rate, 9600);

    //创建一个serial类
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort(ultra_uart_port.c_str());
    //设置串口通信的波特率
    sp.setBaudrate(ultra_uart_baud_rate);
    //串口设置timeout
    sp.setTimeout(to);

    ROS_INFO("_baud_rate is %d; uart_port is %s",ultra_uart_baud_rate,ultra_uart_port.c_str());

    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open ultra port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("ultra port is opened.");
    }
    else
    {
        return -1;
    }
    


    // 设置循环的频率10=10Hz
    ros::Rate loop_rate(10);

    int dis=0;


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
            else if((rcv_buf[4]==0x68)&&(rcv_buf[5]==0x05)&&(rcv_buf[6]==0x01))
            {
                dis=rcv_buf[7]*0xFF+rcv_buf[8];
            }
        }
  

        // 初始化learning_topic::Person类型的消息
    	ultra_serial_port::Ultrasound Ultrasound_msg;
		Ultrasound_msg.dis = dis;

        // 发布消息
		ultrasound_pub.publish(Ultrasound_msg);

       	ROS_INFO("Ultrasound_info: dis=%d", Ultrasound_msg.dis);

        // 按照循环频率延时
        loop_rate.sleep();
    }
    //关闭串口
    sp.close();
    return 0;
}
