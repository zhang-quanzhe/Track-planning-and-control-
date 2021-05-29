#include <stdio.h>
#include <string.h>
#include <stdlib.h>
 
#include <fcntl.h>
#include <unistd.h>
 
#include <termios.h> //set baud rate
 
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
 
//#define rec_buf_wait_2s 2
#define buffLen 1024
#define rcvTimeOut 2
 
 
/*******************************************************/
/**下面的函数暂时并没什么意思，完全可以忽略**/
int read_data_tty(int fd, char *rec_buf, int rec_wait) {
	int retval;
	fd_set rfds;
	struct timeval tv;
 
	int ret, pos;
	tv.tv_sec = rec_wait;
	tv.tv_usec = 0;
	pos = 0;
 
	while (1) {
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
 
		retval = select(fd + 1, &rfds, NULL, NULL, &tv);
 
		if (retval == -1) {
			perror("select()");
			break;
		}
		else if (retval) {
			ret = read(fd, rec_buf + pos, 2048);
			pos += ret;
			if (rec_buf[pos - 2] == '\r' && rec_buf[pos - 1] == '\n') {
				FD_ZERO(&rfds);
				FD_SET(fd, &rfds);
				retval = select(fd + 1, &rfds, NULL, NULL, &tv);
 
				if (!retval) {
					break;
				}
			}
		}
		else {
			break;
		}
	}
 
	return 1;
}
 
int device_485_receive(int fd) {
	int ret;
	char rec_buf[1024];
	int i;
	char send_buf[] = { "02030202f925" };
 
	for (i = 0; i<10; i++) {
		/*ret = write(fd, send_buf, strlen(send_buf));
		if (ret == -1) {
			printf("write device %s error\n", device);
			return -1;
		}*/
 
		//if (read_data_tty(fd, rec_buf, rec_buf_wait_2s)) {
		if (read_data_tty(fd, rec_buf,2)) {
			printf("%s\n", rec_buf);
		}
		else {
			printf("read_error\n");
		}
 
		//if ((read(fd, rec_buf, strlen(rec_buf))) == -1) {
		//      printf("error reading string\n");
		//      return -1;
		//} else {
		//      printf("%s\n", rec_buf);
	}
	return 0;
}
/*-----------------------------------------------------*/
/*******************************************************/
 
/*************Linux and Serial Port *********************/
/*************Linux and Serial Port *********************/
int openPort(int fd, int comport)
{
 
	if (comport == 1)
	{
		fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
		if (-1 == fd)
		{
			perror("Can't Open Serial Port");
			return(-1);
		}
		else
		{
			printf("open ttyS0 .....\n");
		}
	}
	else if (comport == 2)
	{
		fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
		if (-1 == fd)
		{
			perror("Can't Open Serial Port");
			return(-1);
		}
		else
		{
			printf("open ttyS1 .....\n");
		}
	}
	else if (comport == 3)
	{
		fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY);
		if (-1 == fd)
		{
			perror("Can't Open Serial Port");
			return(-1);
		}
		else
		{
			printf("open ttyS2 .....\n");
		}
	}
	/*************************************************/
	else if (comport == 4)
	{
		fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
		if (-1 == fd)
		{
			perror("Can't Open Serial Port");
			return(-1);
		}
		else
		{
			printf("open ttyUSB0 .....\n");
		}
	}
 
	if (fcntl(fd, F_SETFL, 0)<0)
	{
		printf("fcntl failed!\n");
	}
	else
	{
		printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
	}
	if (isatty(STDIN_FILENO) == 0)
	{
		printf("standard input is not a terminal device\n");
	}
	else
	{
		printf("is a tty success!\n");
	}
	printf("fd-open=%d\n", fd);
	return fd;
}
 
int setOpt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio, oldtio;
	if (tcgetattr(fd, &oldtio) != 0)
	{
		perror("SetupSerial 1");
		return -1;
	}
	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
 
	switch (nBits)
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}
 
	switch (nEvent)
	{
	case 'O':                     //奇校验
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E':                     //偶校验
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':                    //无校验
		newtio.c_cflag &= ~PARENB;
		break;
	}
 
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
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	default:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	}
	if (nStop == 1)
	{
		newtio.c_cflag &= ~CSTOPB;
	}
	else if (nStop == 2)
	{
		newtio.c_cflag |= CSTOPB;
	}
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd, TCIFLUSH);
	if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
	{
		perror("com set error");
		return -1;
	}
	printf("set done!\n");
	return 0;
}
 
int readDataTty(int fd, char *rcv_buf, int TimeOut, int Len)
{
	int retval;
	fd_set rfds;
	struct timeval tv;
	int ret, pos;
	tv.tv_sec = TimeOut / 1000;  //set the rcv wait time  
	tv.tv_usec = TimeOut % 1000 * 1000;  //100000us = 0.1s  
 
	pos = 0;
	while (1)
	{
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		retval = select(fd + 1, &rfds, NULL, NULL, &tv);
		if (retval == -1)
		{
			perror("select()");
			break;
		}
		else if (retval)
		{
			ret = read(fd, rcv_buf + pos, 1);
			if (-1 == ret)
			{
				break;
			}
 
			pos++;
			if (Len <= pos)
			{
				break;
			}
		}
		else
		{
			break;
		}
	}
 
	return pos;
}
 
int sendDataTty(int fd, char *send_buf, int Len)
{
	ssize_t ret;
 
	ret = write(fd, send_buf, Len);
	if (ret == -1)
	{
		printf("write device error\n");
		return -1;
	}
 
	return 1;
}
 
//这个函数已经没有用了，修改设备不在这里改，在主函数的fdSerial = openPort(fdSerial, 2)函数处，修改
void serialInit(void)//这个函数已经没有用了，修改设备不在这里改，在主函数的fdSerial = openPort(fdSerial, 2)函数处，修改
{
	int fd = 0;
 
	fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);//串口2
	if (1)
	{
		setOpt(fd, 9600, 8, 'N', 1);
		//set_opt(SerFd, BAUD_115200, DATA_BIT_8, PARITY_NONE, STOP_BIT_1);
	}
	else
	{
		printf("open_port ERROR !\n");
	}
}
 
int main(int argc, char** argv)
{
	int iSetOpt = 0;//SetOpt 的增量i
 
	//serialInit();
	//send_data_tty(SerFd, "hello series\n", sizeof("hello series\n"));
 
	int fdSerial = 0;
 
	//openPort
	if ((fdSerial = openPort(fdSerial, 4))<0)//1--"/dev/ttyS0",2--"/dev/ttyS1",3--"/dev/ttyS2",4--"/dev/ttyUSB0" 小电脑上是2--"/dev/ttyS1"
	{
		perror("open_port error");
		return -1;
	}
	//setOpt(fdSerial, 9600, 8, 'N', 1)
	if ((iSetOpt = setOpt(fdSerial, 9600, 8, 'N', 1))<0)
	{
		perror("set_opt error");
		return -1;
	}
	printf("Serial fdSerial=%d\n", fdSerial);
 
	tcflush(fdSerial, TCIOFLUSH);//清掉串口缓存
	fcntl(fdSerial, F_SETFL, 0);
 
 
	char buffRcvData[buffLen] = { 0 };
	unsigned int readDataNum = 0;
 
	buffRcvData[0] = 's';
	buffRcvData[1] = 't';
	buffRcvData[2] = 'a';
	buffRcvData[3] = 'r';
	buffRcvData[4] = 't';
	
	sendDataTty(fdSerial, buffRcvData, 5);
	while (1){
		readDataNum = readDataTty(fdSerial, buffRcvData, rcvTimeOut, buffLen);
		sendDataTty(fdSerial, buffRcvData, readDataNum);
	}
	
	
	return 1;
}
