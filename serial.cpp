#include <stdio.h>    // Standard input/output definitions
#include <stdlib.h>
#include <math.h>
#include <string.h>   // String function definitions
#include <unistd.h>   // for usleep()
#include <getopt.h>

#include "arduino-serial-lib.h"

#define PI 3.14159265359

void error(char* msg)
{
    fprintf(stderr, "%s\n",msg);
    exit(EXIT_FAILURE);
}

int servoWrite(int fd,int deg1,int deg2,int sp);
int coordinateWrite(int fd,double x,double y,int sp);
void error(char* msg);

int main(int argc, char *argv[])
{
  int fd = -1;
  char serialport[256] = "/dev/ttyACM0";
  int baudrate = 9600;  // default
  char quiet=0;
  char eolchar = '\n';
  int timeout = 5000;
  int rc,n;

  double x,y;
  double xc,yc,zc;

  fd = serialport_init(serialport, baudrate);
  if( fd==-1 ) error("couldn't open port");
  if(!quiet) printf("opened port %s\n",serialport);
  serialport_flush(fd);

  if( fd == -1 ) error("serial port not opened");

  int Speed=10;

  servoWrite(fd,250,0,Speed);

  coordinateWrite(fd,1,250,Speed);

  exit(EXIT_SUCCESS);
} // end main

int servoWrite(int fd,int deg1,int deg2,int sp){
  char buf[256];
  char quiet=0;
  int rc;

  snprintf(buf, 12, "H%02X%02X%02X", deg1, deg2, sp);

  if( !quiet ) printf("send string:%s\n", buf);
  rc = serialport_write(fd, buf);
  if(rc==-1) error("error writing");

  char eolchar = '\n';
  int timeout = 5000;

  while(1){
    if( fd == -1 ) error("serial port not opened");

    memset(buf,0,1);
    serialport_read_until(fd, buf, eolchar, 1, timeout);

    if( !quiet ) printf("read string:");

    if(!strcmp(buf, "I")){
      printf("Reach target\n");
    break;
    }
  }

}

int coordinateWrite(int fd,double x,double y,int sp){
  char buf[256];
  char quiet=0;
  int rc;

  double l1=127,l2=127;
  double A,B,C,D;
  int deg1,deg2;
  double th1,th2;

  //逆運動学
  A = pow(x,2)+pow(y,2)-pow(l1,2)-pow(l2,2);
  B = sqrt( 4 - ( pow(A,2)/(pow(l1,2)*pow(l2,2)) ));
  C = l1 + A/(2*l1);
  th1 = atan(y/x) - atan((l2*B)/(2*C));

  D = A/(2*l1*l2);
  th2 = acos(D);

  //printf("th1:%f th2:%f\n",th1,th2);
  deg1=215-th1*180/PI;
  deg2=125-th2*180/PI;
  printf("%.2f deg1:%d deg2:%d\n",y,deg1,deg2);

  if(deg1 > 250){
    deg1 = 250;
    printf("Limit over ");
  }else if(deg2 > 250){
    deg2 = 250;
    printf("Limit over ");
  }else if(deg1 < 0){
    deg1 = 0;
    printf("Limit over ");
  }else if(deg2 < 0){
    deg2 = 0;
    printf("Limit over ");
  }else{

  }

  snprintf(buf, 12, "H%02X%02X%02X", deg1, deg2, sp);
  if( !quiet ) printf("send string:%s\n", buf);
  rc = serialport_write(fd, buf);
  if(rc==-1) error("error writing");

  char eolchar = '\n';
  int timeout = 5000;

  //目標点到達時、Arduino側から文字”I”が送信
  while(1){
    if( fd == -1 ) error("serial port not opened");

    memset(buf,0,1);
    serialport_read_until(fd, buf, eolchar, 1, timeout);

    if( !quiet ) printf("read string:");

    if(!strcmp(buf, "I")){
      printf("Reach target\n");
    break;
    }
  }
}
