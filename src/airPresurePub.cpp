//
// Created by echo on 18-8-13.
//
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix标准函数定义*/
#include     <sys/types.h>  /**/
#include     <sys/stat.h>   /**/
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*PPSIX终端控制定义*/
#include     <errno.h>      /*错误号定义*/
using namespace std;
class airPressure{
public:
    airPressure(){};
    void setup(ros::NodeHandle node,ros::NodeHandle privateNode);
    void spin();
    void process();
    int serialProcess();
    void set_speed(int fd, int speed);
    int OpenDev(char *Dev);
private:
    ros::Publisher _pressurePub;
    int speed_arr[15] = { B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
                          B38400, B19200, B9600, B4800, B2400, B1200, B300};
    int name_arr[15] = { 115200, 38400, 19200, 9600, 4800, 2400, 1200, 300,
                         38400, 19200, 9600, 4800, 2400, 1200, 300};
    int fd;
    int nread;
    int buff[2000];
    char buff_char[2000];
    char *dev = "/dev/ttyUSB0";
};

void airPressure::setup(ros::NodeHandle node, ros::NodeHandle privateNode) {
    fd = OpenDev(dev);
    if (fd>0){
        set_speed(fd, 115200);
    }
    //_pressurePub =node.advertise<std_msgs::Bool> ("/g2o_finish", 5);
}

void airPressure::spin() {
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        process();
        //serialProcess();
        rate.sleep();
    }
}

void airPressure::process() {
    serialProcess();
}

void airPressure::set_speed(int fd, int speed) {
    int   i;
    int   status;
    struct termios   Opt;

    tcgetattr(fd, &Opt);
    for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);
            if (status != 0)
                perror("tcsetattr fd1");
            return;
        }
        tcflush(fd, TCIOFLUSH);

    }
}

int airPressure::OpenDev(char *Dev) {
    int fd = open(Dev, O_RDWR);
    if (-1 == fd)
    {
        perror("Can't Open Serial Port");
        return -1;
    }
    else
        return fd;
}

int airPressure::serialProcess() {


    if((nread = read(fd, buff, sizeof(buff)))>0)
    {
        string a;
        printf("recv_message: %s \n", buff);

        sprintf(buff_char,"%s",buff);
        a = buff_char;
        cout<<a<<endl;
        memset(buff,0,sizeof(buff));
        memset(buff_char,0,sizeof(buff_char));

    }
}
int main(int argc, char **argv){
    ros::init(argc, argv, "AirPressure");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");
    airPressure g2osaver;
    g2osaver.setup(node,privateNode);
    g2osaver.spin();

}
