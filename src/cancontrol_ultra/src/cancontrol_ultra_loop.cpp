/**
 * 该例程将订阅/person_info话题，自定义消息类型learning_topic::Person
 */

#include <ros/ros.h>
#include "/home/ustb/xxjcar/devel/include/ultra_serial_port/Ultrasound.h"
#include "/home/ustb/xxjcar/src/cancontrol/include/controlcan.h"

#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <ctime>
#include <cstdlib>
#include "unistd.h"

// 接收到订阅的消息后，会进入消息回调函数



int xorcheck( int matrix[][8], int rowsize );//异或校验取值
void *receive_func(void* param);//接收线程。
void send_command_to_car(int cmd_Gear_p, double cmd_Steering_p, double cmd_Drive_V_p, int Braking_p, int Parking_p, int rownum);//处理下发的消息
VCI_BOARD_INFO pInfo;//用来获取设备信息。
VCI_BOARD_INFO pInfo1 [50];
//Ackermann_Car_Command  car_command[1];//目标消息
Ackermann_Car_Feedback car_back[1];//反馈消息

clock_t startTime,endTime;//计时
int count = 0;//数据列表中，用来存储列表序号。
int num = 0;
int ultra_dis = 30;//超声波距离
int ultra_dis2 = 30;
int ultra_en_flag = 0;
int Datasource[5][8] = {{1,0x04,0},{1,0,8},{1,0xF4,0x01},{1,0,0},{1,0,0}};
int Datasourcestop[5][8] = {{1,0x03,0},{1,0,8},{1,0x00,0x00},{1,0,0},{1,1,0}};//空挡、抱闸

//void UltrasoundInfoCallback(const Ultra_serial_port::Ultrasound::ConstPtr& Ultrasound_msg)
void UltrasoundInfoCallback(const ultra_serial_port::Ultrasound::ConstPtr& Ultrasound_msg)
{/*
	if(Ultrasound_msg->dis>30)
	{
		ROS_INFO(" dis_rx=%d", Ultrasound_msg->dis);// 将接收到的消息打印出来
	}
	else
	{
		ROS_INFO(" WARRING dis_rx=%d", Ultrasound_msg->dis);// 将接收到的消息打印出来
	}
*/
ultra_en_flag = 1;
		printf("xxx%d\n",ultra_en_flag);
ultra_dis = Ultrasound_msg->dis;
ultra_dis2 = Ultrasound_msg->dis2;

}

int main(int argc, char **argv)
{

    // 初始化ROS节点
    ros::init(argc, argv, "cancontrol_ultra");

    // 创建节点句柄
    ros::NodeHandle nc;

    // 创建一个Subscriber，订阅名为/person_info的topic
    ros::Subscriber Ultrasound_info_sub = nc.subscribe("/Ultrasound_info", 1,UltrasoundInfoCallback);


	
	//printf(">>Dis=%d\n",dis);
	//printf(">>this is hello !\r\n");//指示程序已运行
	
	num=VCI_FindUsbDevice2(pInfo1);
	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		printf(">>open deivce success!\n");//打开设备成功
	}else
	{
		printf(">>open deivce error!\n");
		exit(1);
	}
	//VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo);//读取设备序列号、版本等信息。


	//初始化参数，严格参数二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	config.Filter=1;//接收所有帧
	config.Timing0=0x00;/*波特率500 Kbps  0x00  0x1C*/
	config.Timing1=0x1C;
	config.Mode=0;//正常模式
		
	VCI_InitCAN(VCI_USBCAN2,0,0,&config);
	VCI_StartCAN(VCI_USBCAN2,0,0);

	//需要发送的帧，结构体设置
	VCI_CAN_OBJ send[1];
	send[0].ID=0x18C4D1D0;
	send[0].SendType=0;
	send[0].RemoteFlag=0;
	send[0].ExternFlag=1;
	send[0].DataLen=8;
	
	int i=0;
	int WARRING_dis=80;//超声波安全距离
//	int Datasource[5][8] = {{1,0x04,0,0,0,0,0,0x06},{1,0,8,0,0,0,0,9},{1,0xF4,0x01,0,0,0,0,0xF4},{1,0,0,0,0,0,0,1},{1,0,0,0,0,0,0,1}};//档位、转向0度=0X0800、车速1m/s=0x03E8,0.5m/s=0x01F4/F4、
//	int Datasource[5][8] = {{1,0x04,0},{1,0,8},{1,0xF4,0x01},{1,0,0},{1,0,0}};
//	int Datasourcestop[5][8] = {{1,0x03,0},{1,0,8},{1,0x00,0x00},{1,0,0},{1,1,0}};//空挡、抱闸


	int m_run0=1;
	pthread_t threadid;
	int ret;
	ret=pthread_create(&threadid,NULL,receive_func,&m_run0);

	//startTime = clock();//计时开始
int loopnum=0;
	int times = 500;//times = 500 = 1S
	int rownum=0;
	int rownum_total = sizeof(Datasource)/sizeof(Datasource[0]);
    // 设置循环的频率10=10Hz
    ros::Rate loop_rate(100);
	while(ros::ok())
	{//printf("开始\n");
//can1TX**
	//cmd_Gear_p：02=后退；03=空挡；04=前进；
	//cmd_Steering_p：角度
	//cmd_Drive_V_p：速度
	//Braking_p：00即可
	//Parking_p：00=Release；01=Apply(驻车)
		send_command_to_car(3,0,0,0,1,rownum);	

		ros::spinOnce();//循环等待回调函数

		if(rownum == rownum_total ) 
		{
			rownum = 0;////复位计数Datasource 行
			send[0].ID = 0x18C4D1D0;//复位计数ID
		}
		//if(send[0].ID == 0x18C4D6D0) send[0].ID = 0x18C4D1D0;//复位计数ID
		if(Datasource[rownum][6] == 0xFF+0x1){ Datasource[rownum][6] = 0;}//复位计数Alivecount

		

//		if(times >= 5 )
//		{ 
if (loopnum == 0)
{
			if(ultra_dis > WARRING_dis )
			{		
				send_command_to_car(4,0,0.5,0,0,rownum);	

				printf("ultra_dis %d is OK\n",ultra_dis);

				for(i = 0; i < send[0].DataLen; i++)//给数据
				{
					send[0].Data[i] = Datasource[rownum][i];
				}
			}
			else
			{
				send_command_to_car(3,0,0,0,1,rownum);	
				printf("WARRING ultra_dis %d< %d cm\n",ultra_dis,WARRING_dis);
				Datasource[0][1] = Datasourcestop[0][1];
				Datasource[4][1] = Datasourcestop[4][1];
				for(i = 0; i < send[0].DataLen; i++)//给数据
				{
					send[0].Data[i] = Datasource[rownum][i];
				}
				loopnum = 1;
			}
}
if (loopnum == 1)
{
			if(ultra_dis < 150 )
			{		
				send_command_to_car(2,0,0.5,0,0,rownum);	

				printf("ultra_dis %d is OK\n",ultra_dis);

				for(i = 0; i < send[0].DataLen; i++)//给数据
				{
					send[0].Data[i] = Datasource[rownum][i];
				}
			}
			else
			{
				send_command_to_car(3,0,0,0,1,rownum);	
				printf("WARRING ultra_dis %d< %d cm\n",ultra_dis,WARRING_dis);
				Datasource[0][1] = Datasourcestop[0][1];
				Datasource[4][1] = Datasourcestop[4][1];
				for(i = 0; i < send[0].DataLen; i++)//给数据
				{
					send[0].Data[i] = Datasource[rownum][i];
				}

				loopnum = 0;
			}
}
		// }
		// else//最后给空挡、刹车
		// {
		// 	send_command_to_car(3,0,0,0,0,rownum);	
		// 	for(i = 0; i < send[0].DataLen; i++)//给数据
		// 	{
		// 		send[0].Data[i] = Datasource[rownum][i];
		// 	}
		// }

		Datasource[rownum][6] += 1;//Alivecount 循环+1
		
		rownum++;

		if(VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
		{

			printf("Index:%04d  ",count);count++;
			printf("CAN1 TX ID:0x%08X",send[0].ID);
			if(send[0].ExternFlag==0) printf(" Standard ");
			if(send[0].ExternFlag==1) printf(" Extend   ");
			if(send[0].RemoteFlag==0) printf(" Data   ");
			if(send[0].RemoteFlag==1) printf(" Remote ");
			printf("DLC:0x%02X",send[0].DataLen);
			printf(" data:0x");

			for(i=0;i<send[0].DataLen;i++)
			{
				printf(" %02X",send[0].Data[i]);
			}

			printf("\n");
			//printf("发送一帧\n");
		}
		else
		{
			printf("CAN1 TX ERROR");
		}
		send[0].ID+=0x0100;//给ID+1

		loop_rate.sleep();
		//usleep(10000);//延时10ms。
		//usleep(10000);//延时10ms。
	
	}

//endTime = clock();//计时结束
//cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	usleep(10000000/5);//延时单位us，这里设置 10 000 000=10s    10s后关闭接收线程，并退出主程序。
	m_run0=0;//线程关闭指令。
	pthread_join(threadid,NULL);//等待线程关闭。
	usleep(100000);//延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
	usleep(100000);//延时100ms。

	//VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
	//usleep(100000);//延时100ms。
	VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
	//除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
	//goto ext;

return 0;
}

void *receive_func(void* param)  //接收线程。
{
	int reclen=0;
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
	int i,j;
	
	int *run=(int*)param;//线程启动，退出控制。
    	int ind=0;//can1的通道号
	
	while((*run)&0x0f)
	{

		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,5,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(j=0;j<reclen;j++)
			{
				printf("Index:%04d  ",count);count++;//序号递增
				printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
				if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
				if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
				if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
				if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
				printf("DLC:0x%02X",rec[j].DataLen);//帧长度
				printf(" data:0x");	//数据
				for(i = 0; i < rec[j].DataLen; i++)
				{
					printf(" %02X", rec[j].Data[i]);
				}
				printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
				printf("\n");


				if(rec[j].ID == 0x18C4D1EF)//档位信息
				{
					car_back[0].Gear = rec[j].Data[1];
				}
				if(rec[j].ID == 0x18C4D2EF)//方向盘转角
				{
					car_back[0].Steering = (rec[j].Data[1]+rec[j].Data[2]*256)*0.043945-90;
					printf("%02X",rec[j].Data[2]);//序号递增
					printf("%02X",rec[j].Data[1]);//序号递增

				}

				if(rec[j].ID == 0x18C4D3EF)//车速
				{
					car_back[0].Drive_V = (rec[j].Data[1]+rec[j].Data[2]<<8)*0.001;//m/s
				}

				if(rec[j].ID == 0x18C4D4EF)//制动状态信息（预留）
				{
					car_back[0].Braking =  rec[j].Data[2];
				}

				if(rec[j].ID == 0x18C4D5EF)//电子驻车状态
				{
					car_back[0].Parking =  rec[j].Data[1];
				}

				if(rec[j].ID == 0x18C4D7EF)//2个后轮轮速
				{
					car_back[0].WheelSpd[0] =  rec[j].Data[1]*0.04-8;//左后轮轮速
					car_back[0].WheelSpd[1] =  rec[j].Data[2]*0.04-8;//右后轮轮速
				}

				if(rec[j].ID == 0x18C4D8EF)//2个后轮脉冲数
				{
				}

				if(rec[j].ID == 0x18C4D9EF)//累计里程
				{
				}
				//printf("方向盘转角=%f°\n",car_back[0].Steering);

			}
			
		}
			usleep(1000000);//延时10ms。	
		//ind=!ind;//变换通道号，以便下次读取另一通道，交替读取。		
	}
	printf("run thread exit\n");//退出接收线程	
	pthread_exit(0);
}

int xorcheck( int matrix[][8], int rowsize)
{
	int checksum=matrix[rowsize][0];
	for (int xori = 0;  xori<6;  xori++)
    	{	
        	checksum = checksum ^ matrix[rowsize][xori+1]; 
		//matrix[rownum][7] = checksum;//printf ("xor%d",Datasource[rownum][7]);
    	}
	return checksum;
}

void send_command_to_car(int cmd_Gear_p, double cmd_Steering_p, double cmd_Drive_V_p, int Braking_p, int Parking_p, int rownum)
{
	//cmd_Gear_p：02=后退；02=空挡；04=前进；
	//cmd_Steering_p：角度
	//cmd_Drive_V_p：速度
	//Braking_p：00即可
	//Parking_p：00=Release；01=Apply(驻车)
	//int Datasource[5][8] = {{1,0x04,0},{1,0,8},{1,0xF4,0x01},{1,0,0},{1,0,0}};
	//int Datasourcestop[5][8] = {{1,0x03,0},{1,0,8},{1,0x00,0x00},{1,0,0},{1,1,0}};//空挡、抱闸

	Datasource[0][1] = cmd_Gear_p;

	int int_cmd_Steering_p=(cmd_Steering_p+90)/0.043945;
	Datasource[1][1] = int_cmd_Steering_p&0xFF;
	Datasource[1][2]=(int_cmd_Steering_p>>8)&0xFF;

	int int_cmd_Drive_V_p=cmd_Drive_V_p/0.001;
	Datasource[2][1] = int_cmd_Drive_V_p&0xFF;
	Datasource[2][2]=(int_cmd_Drive_V_p>>8)&0xFF;

	Datasource[3][1] = Braking_p;

	Datasource[4][1] = Parking_p;

	Datasource[rownum][7] = xorcheck(Datasource, rownum);//初始化,异或校验

	ultra_en_flag = 0;

}
// void * Datasource_handle(int Datasource,int rownum,int rownum_total)
// {
// 	if(rownum == rownum_total ) 
// 	{
// 		rownum = 0;////复位计数Datasource 行
// 		send[0].ID = 0x18C4D1D0;//复位计数ID
// 	}
// 		//if(send[0].ID == 0x18C4D6D0) send[0].ID = 0x18C4D1D0;//复位计数ID
// 	if(Datasource[rownum][6] == 0xFF+0x1) Datasource[rownum][6] = 0;//复位计数Alivecount

// 	Datasource[rownum][7] = xorcheck(Datasource, rownum);//初始化,异或校验
// }