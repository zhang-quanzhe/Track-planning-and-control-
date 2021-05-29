/**
 * 该例程将发布/person_info话题，自定义消息类型learning_topic::Person
 */
 
#include <ros/ros.h>
#include "Ultra_serial_port/Ultrasound.h"
/* 包含的头文件 */
#include <stdio.h>        //标准输入输出,如printf、scanf以及文件操作
#include <stdlib.h>        //标准库头文件，定义了五种类型、一些宏和通用工具函数
#include <unistd.h>        //定义 read write close lseek 等Unix标准函数
#include <sys/types.h>    //定义数据类型，如 ssiz e_t off_t 等
#include <sys/stat.h>    //文件状态
#include <fcntl.h>        //文件控制定义
#include <termios.h>    //终端I/O
#include <errno.h>        //与全局变量 errno 相关的定义
#include <getopt.h>        //处理命令行参数
#include <string.h>        //字符串操作
#include <time.h>        //时间
#include <sys/select.h>    //select函数
#include <iostream>
#define DEV_NAME    "/dev/ttyUSB0"    ///< 串口设备

int setOpt(int fd, int nSpeed, int nBits, int nParity, int nStop);
int UART_Recv(int fd, unsigned char *rcv_buf, int data_len, int timeout);
int UART_Send(int fd, char *send_buf, int data_len);
int Ultrasound_open ();
int Ultrasound_check ();

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "Ultrasound_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
    ros::Publisher Ultrasound_pub = n.advertise<Ultra_serial_port::Ultrasound>("/Ultrasound_info", 10);

    // 设置循环的频率
    ros::Rate loop_rate(25);

    int count = 0;
int fdSerial = Ultrasound_open ();
int dis=0;
printf("Ultrasound_open () is ok \n");

    unsigned char rcv_buf[24];
    int len;

    while (ros::ok())
    {

        len = UART_Recv(fdSerial, rcv_buf, 20, 10000);
	//printf ("%d",len);
	
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



        // 初始化learning_topic::Person类型的消息
    	Ultra_serial_port::Ultrasound Ultrasound_msg;
		Ultrasound_msg.dis = dis;

        // 发布消息
		Ultrasound_pub.publish(Ultrasound_msg);

       		ROS_INFO("Ultrasound_info: dis=%d", Ultrasound_msg.dis);

        // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}


/**@file      main.c
 * @brief       串口应用编程测试
 * @details
 * @author      wanghuan  any question please send mail to 371463817@qq.com
 * @date        2019-06-17
 * @version     V1.0
 * @copyright    Copyright (c) 2019-2022 
 **********************************************************************************
 * @attention
 * 硬件平台:iMX6ULL \n
 * 内核版本：L4.1.15
 * @par 修改日志:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2019/06/17  <td>1.0      <td>       <td>创建初始版本
 * </table>
 **********************************************************************************
 */


/**@brief   设置串口参数：波特率，数据位，停止位和效验位
 * @param[in]  fd         类型  int      打开的串口文件句柄
 * @param[in]  nSpeed     类型  int     波特率
 * @param[in]  nBits     类型  int     数据位   取值 为 7 或者8
 * @param[in]  nParity     类型  int     停止位   取值为 1 或者2
 * @param[in]  nStop      类型  int      效验类型 取值为N,E,O,,S
 * @return     返回设置结果
 * - 0         设置成功
 * - -1     设置失败
 */
int setOpt(int fd, int nSpeed, int nBits, int nParity, int nStop)
{
    struct termios newtio, oldtio;

    // 保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息
   if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("SetupSerial 1");
        return -1;
    }

    bzero(&newtio, sizeof(newtio));        //新termios参数清零
    newtio.c_cflag |= CLOCAL | CREAD;    //CLOCAL--忽略 modem 控制线,本地连线, 不具数据机控制功能, CREAD--使能接收标志


    // 设置数据位数
    newtio.c_cflag &= ~CSIZE;    //清数据位标志
    switch (nBits)
    {
        case 7:
            newtio.c_cflag |= CS7;
        break;
        case 8:
            newtio.c_cflag |= CS8;
        break;
        default:
            fprintf(stderr, "Unsupported data size\n");
            return -1;
    }
    // 设置校验位
    switch (nParity)
    {
        case 'o':
        case 'O':                     //奇校验
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'e':
        case 'E':                     //偶校验
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'n':
        case 'N':                    //无校验
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            fprintf(stderr, "Unsupported parity\n");
            return -1;
    }
    // 设置停止位
    switch (nStop)
    {
        case 1:
            newtio.c_cflag &= ~CSTOPB;
        break;
        case 2:
            newtio.c_cflag |= CSTOPB;
        break;
        default:
            fprintf(stderr,"Unsupported stop bits\n");
            return -1;
    }
    // 设置波特率 2400/4800/9600/19200/38400/57600/115200/230400
    switch (nSpeed)
    {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 19200:
            cfsetispeed(&newtio, B19200);
            cfsetospeed(&newtio, B19200);
            break;
        case 38400:
            cfsetispeed(&newtio, B38400);
            cfsetospeed(&newtio, B38400);
            break;
        case 57600:
            cfsetispeed(&newtio, B57600);
            cfsetospeed(&newtio, B57600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 230400:
            cfsetispeed(&newtio, B230400);
            cfsetospeed(&newtio, B230400);
            break;
        default:
            printf("\tSorry, Unsupported baud rate, set default 9600!\n\n");
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }
    // 设置read读取最小字节数和超时时间
    newtio.c_cc[VTIME] = 0.01;     // 读取一个字符等待1*(1/10)s
    newtio.c_cc[VMIN] = 1;        // 读取字符的最少个数为1

      tcflush(fd,TCIFLUSH);         //清空缓冲区
      if (tcsetattr(fd, TCSANOW, &newtio) != 0)    //激活新设置
      {
        perror("SetupSerial 3");
          return -1;
     }

      printf("Serial set done!\n");
    return 0;
}

/**@brief 串口读取函数
 * @param[in]  fd          打开的串口文件句柄
 * @param[in]  *rcv_buf    接收缓存指针
 * @param[in]  data_len    要读取数据长度
 * @param[in]  timeout     接收等待超时时间，单位ms
 * @return     返回设置结果
 * - >0      设置成功
 * - 其他      读取超时或错误
 */
int UART_Recv(int fd, unsigned char *rcv_buf, int data_len, int timeout)
{
    int len, fs_sel;
    fd_set fs_read;
    struct timeval time;

    time.tv_sec = timeout / 1000;              //set the rcv wait time
    time.tv_usec = timeout % 1000 * 1000;    //100000us = 0.1s

    FD_ZERO(&fs_read);        //每次循环都要清空集合，否则不能检测描述符变化
    FD_SET(fd, &fs_read);    //添加描述符

    // 超时等待读变化，>0：就绪描述字的正数目， -1：出错， 0 ：超时
    fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
//    printf("fs_sel = %d\n", fs_sel);
    if(fs_sel)
    {
        len = read(fd, rcv_buf, data_len);
        return len;
    }
    else
    {
//        printf("Sorry,I am wrong!");
        return -1;
    }
}

/**@brief 串口发送函数
 * @param[in]  fd            打开的串口文件句柄
 * @param[in]  *send_buf     发送数据指针
 * @param[in]  data_len     发送数据长度
 * @return     返回结果
 * - data_len    成功
 * - -1            失败
 */
int UART_Send(int fd, char *send_buf, int data_len)
{
    ssize_t ret = 0;

    ret = write(fd, send_buf, data_len);
    if (ret == data_len)
    {
        printf("send data is %s\n", send_buf);
        return ret;
    }
    else
    {
        printf("write device error\n");
        tcflush(fd,TCOFLUSH);
        return -1;
    }
}


/**@fn main
 * @brief main入口函数
 */
//using namespace std;
int Ultrasound_open ()
{
	int fdSerial;
    // 打开串口设备
	fdSerial = open(DEV_NAME, O_RDWR | O_NOCTTY | O_NDELAY);

	fcntl(fdSerial, F_SETFL, FNDELAY);
	isatty(fdSerial);

    // 设置串口参数
    if (setOpt(fdSerial, 9600, 8, 'N', 1)== -1)    //设置8位数据位、1位停止位、无校验
    {
        fprintf(stderr, "Set opt Error\n");
        close(fdSerial);
        exit(1);
    }

    tcflush(fdSerial, TCIOFLUSH);    //清掉串口缓存
    fcntl(fdSerial, F_SETFL, 0);    //串口阻塞

return fdSerial;

}

/*
int Ultrasound_check ()
{
int fdSerial = Ultrasound_open ();
int dis=0;
printf("Ultrasound_open () is ok \n");

    unsigned char rcv_buf[24];
    int len;

    while(1)    //循环读取数据
    {


        len = UART_Recv(fdSerial, rcv_buf, 20, 10000);
	//printf ("%d",len);
	
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
return dis;
        usleep(100*1000);    //休眠100ms
    }
return dis;
}
*/
