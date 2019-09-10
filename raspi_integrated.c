#include<stdio.h>
#include<unistd.h>
#include<sys/mman.h>
#include<fcntl.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

//for GPIOs
#define	GPIO_PERI_BASE_OLD	0x20000000
#define	GPIO_PERI_BASE_NEW	0x3F000000
#define PIN23_GPIO1  23
#define PIN25_GPIO2  25
#define PIN24_GPIO3  24
#define PIN17_GPIO4  17
#define PIN10_GPIO5  10
#define PIN9_GPIO6   9
#define PIN11_GPIO7  11
#define PIN7_GPIO8   7
#define PIN8_GPIO9   8
#define PIN20_GPIO10 20
#define PIN19_GPIO11 19
#define PIN21_GPIO12 21
#define TRUE 1
#define FALSE 0
#define BLOCK_SIZE 4*1024 


#define DISPLAY_STRING 1
char buf[10];
unsigned char result_buf[6500];
unsigned char temp[10];
unsigned int rdIdx;
unsigned short final_result[1254];
unsigned char pcnt=0;

char pin_arr[] = {PIN23_GPIO1,PIN25_GPIO2,PIN24_GPIO3,PIN17_GPIO4,PIN10_GPIO5,PIN9_GPIO6,PIN11_GPIO7,PIN7_GPIO8,PIN8_GPIO9,PIN20_GPIO10,PIN19_GPIO11,PIN21_GPIO12};

static volatile unsigned int GPIO_BASE;
int i,pin;
static volatile unsigned int piGpioBase = 0 ;
static unsigned int usingGpioMem    = FALSE ;
typedef uint32_t;
uint32_t *gpio;
void ascii_to_int(char*);
void display(unsigned short*);
void displaychar(unsigned char*);

void enableGPIO(char);
void disableGPIO(char);

int set_interface_attribs(int fd, int speed)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

	tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;         /* 8-bit characters */
	tty.c_cflag &= ~PARENB;     /* no parity bit */
	tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		printf("Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

void set_mincount(int fd, int mcount)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error tcgetattr: %s\n", strerror(errno));
		return;
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;
	tty.c_cc[VTIME] = 5;        /* half second timer */

	if (tcsetattr(fd, TCSANOW, &tty) < 0)
		printf("Error tcsetattr: %s\n", strerror(errno));
}


void main()
{
	//uart initialization
	char *portname = "/dev/ttyUSB0";
	int fd;
	int wlen;
	int rdlen;
	char ctr;
	unsigned int dptr = 0;
	
	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		printf("Error opening %s: %s\n", portname, strerror(errno));
		return -1;
	}
	/*baudrate 115200, 8 bits, no parity, 1 stop bit */
	set_interface_attribs(fd, B9600);

	//dup2(fdresult,1);
	while(1)
	{

			rdlen = read(fd, buf, 1);
			printf("%c",buf[0]);
	}
}
