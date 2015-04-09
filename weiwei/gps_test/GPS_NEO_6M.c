#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <time.h>
#include "nmea/nmea.h" 


#define GPS_MESG_LENGTH  500  
#define UART2_TTY   "/dev/ttyS2"


int  open_port(void)
{
	int fd = open(UART2_TTY,O_RDWR| O_NOCTTY| O_NDELAY);
	if(-1 == fd)
	{
		perror("open");
		return -1;
	}
	
	if(fcntl(fd, F_SETFL, 0) <0)  //���ô���Ϊ����״̬
	{
 		perror("fcntl");
		return -1;
	}

	return fd;
}

int set_port(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
 struct termios newtio, oldtio;
/*����������д��� �������ã�������������� �ŵȳ� �� ������صĳ� ����Ϣ*/
 if ( tcgetattr( fd, &oldtio) != 0)  { 
		perror("SetupSerial 1");
 return -1;
 }
 bzero( &newtio, sizeof( newtio ) );
/*����һ�� �����ַ���С*/
 newtio. c_cflag |= CLOCAL | CREAD; 
 newtio. c_cflag &= ~CSIZE; 
/*����ֹͣλ*/
 switch( nBits )
 {
	 case 7:
		newtio.c_cflag |= CS7;
	 break;
	 
	 case 8:
		newtio.c_cflag |= CS8;
	 break;
 }
/*������żУ��λ*/
 switch( nEvent )
 {
	 case 'O' : //����
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP) ;
	 break;
	 
	 case 'E' : //ż��
		newtio.c_iflag |= (INPCK | ISTRIP) ;
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
	 break;
	 
	 case 'N': //����żУ��λ
		newtio.c_cflag &= ~PARENB;
	 break;
 }
 /*���ò�����*/
 switch( nSpeed )  //set baud rate
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
		
		case 460800:
		 cfsetispeed(&newtio, B460800);
		 cfsetospeed(&newtio, B460800);
	  break;
		
		default:
		 cfsetispeed(&newtio, B9600);
		 cfsetospeed(&newtio, B9600);
	  break;
	}
 
	if( nStop == 1 )   //set  stop bit
 		newtio.c_cflag &= ~CSTOPB;
	else if ( nStop == 2 )
 		newtio.c_cflag |= CSTOPB;

 	newtio.c_cc[VTIME] = 0;  //set  wait time 
 	newtio.c_cc[VMIN] = 0;   //set  min rev bytes

	tcflush(fd, TCIFLUSH);     //deal the char those didn't receive

	if((tcsetattr(fd, TCSANOW, &newtio) ) !=0)  //activate the new configuration
  {
		perror("com set error");
		return -1;
  }
 	printf("set done! \n") ;
 	return 0;
}



int main(int argc,char *argv[])
{
	int fd,readBytes;
	char buff[GPS_MESG_LENGTH];
	char longitude[12];
	char latitude[12];
	char buffTemp[5];
	int tmp;

	nmeaINFO info;                  // nmeaЭ���������ṹ��  
	nmeaPARSER parser;              // nmeaЭ���������  

	nmea_zero_INFO(&info);          // ����Ĭ�ϵĽ������  
	nmea_parser_init(&parser);      // Ϊ������������ڴ�ռ� 
	printf("hi\n");	
	if(-1 ==(fd = open_port()))
	{
		perror("open port");
		return -1;
	}

	if(set_port(fd,9600,8,'N',1) < 0)
	{
		perror("set_opt");
		return -1;
	}

	while(1)
	{
		readBytes = read(fd,buff,GPS_MESG_LENGTH);
		if(readBytes < 0)
		{
			perror("read");
			return -1;
		}
	//	printf("Read Bytes = %d\n Message : %s\n",readBytes,buff);

  
    // ���ú������GPS��Ϣ���������ս��������info������  
    if((nmea_parse(&parser, buff, (int)strlen(buff), &info)) > 0)  
    {  
       // printf("longitude   %.5f\r\n",info.lon);  
       // printf("latitude    %.5f\r\n",info.lat);   
    }  

//longitude
    tmp = (int)(info.lon/100);  // ��
		
		sprintf(buffTemp,"%d",tmp); //��������ת�����ַ���
		strcat(longitude,buffTemp);
		strcat(longitude, "\"");	
    
		
    tmp = (int)(info.lon - tmp * 100);  //��
		sprintf(buffTemp,"%d",tmp);
		strcat(longitude,buffTemp);
		strcat(longitude, "'");

		sprintf(buffTemp,"%.2f",(info.lon - (int)info.lon) * 60); //��
    strcat(longitude,buffTemp);
		
	  printf("Longitude = %s\n",longitude);
		
//latitude
    tmp = (int)(info.lat/100);
		
		sprintf(buffTemp,"%d",tmp);
		strcat(latitude,buffTemp);
		strcat(latitude, "\"");		
    
		
    tmp = (int)(info.lat - tmp * 100); 
		sprintf(buffTemp,"%d",tmp);
		strcat(latitude,buffTemp);
		strcat(latitude, "'");

		sprintf(buffTemp,"%.2f",(info.lat - (int)info.lat) * 60);
    strcat(latitude,buffTemp);
		
	  printf("latitude = %s\n",latitude);

		
    printf("\n");
		bzero(longitude,sizeof(longitude));
		bzero(latitude,sizeof(latitude));
	  sleep(1);
	}
	nmea_parser_destroy(&parser);   // �ͷŽ���������ڴ�ռ� 
	close(fd);	
	return ;
}

