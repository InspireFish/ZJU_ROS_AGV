#include <ros/ros.h>
#include <serial/serial.h>              //manipulation on SerialPort
// #include "agv_protocol.h"               //Frame Protocol
#include <iostream>                     //C++ operation
#include <math.h>                       //velocity transformation

#include <geometry_msgs/TwistStamped.h> //subscribe the CMD_VEL
using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef FRAME_SEND_LENGTH
#define FRAME_SEND_LENGTH 26
#endif

#ifndef FRAME_REC_LENGTH
#define FRAME_REC_LENGTH 34
#endif

float linear_vel_x, linear_vel_y, linear_vel_z, angular_vel_z;//send back velocity info to ROS topic net
serial::Serial sp;

//define the txFrame
typedef union
{
    struct _tx_frame
    {
        unsigned char       SOF;          //1byte
        unsigned char       CMType;       //1byte
        unsigned char       ID;           //1byte
        unsigned char       dataLength;   //1byte
        float               moveAcc;      //4byte
        float               rotateAcc;    //4byte
        float               moveSpeed;    //4byte
        float               moveAngle;    //4byte
        float               rotateSpeed;  //4byte
        unsigned short int  CRCCheck;     //2byte
    }txFrame;
    unsigned char buffer[FRAME_SEND_LENGTH];
}SerialTxFrame;

bool param_use_debug_cmd;               //control on/off: print cmd data on screen  

//////////////////////////////////
//      CRC16 Code Table        //
//      Initial value 0x2B3B    //
//      Polynomial 0x2333       //
//////////////////////////////////
static const unsigned short CRC16_List[256] =//CRC8 looking up table
{
	0x0000, 0x0E53, 0x1CA6, 0x12F5, 0x394C, 0x371F, 0x25EA, 0x2BB9,
	0x34FF, 0x3AAC, 0x2859, 0x260A, 0x0DB3, 0x03E0, 0x1115, 0x1F46,
	0x2F99, 0x21CA, 0x333F, 0x3D6C, 0x16D5, 0x1886, 0x0A73, 0x0420,
	0x1B66, 0x1535, 0x07C0, 0x0993, 0x222A, 0x2C79, 0x3E8C, 0x30DF,
	0x1955, 0x1706, 0x05F3, 0x0BA0, 0x2019, 0x2E4A, 0x3CBF, 0x32EC,
	0x2DAA, 0x23F9, 0x310C, 0x3F5F, 0x14E6, 0x1AB5, 0x0840, 0x0613,
	0x36CC, 0x389F, 0x2A6A, 0x2439, 0x0F80, 0x01D3, 0x1326, 0x1D75,
	0x0233, 0x0C60, 0x1E95, 0x10C6, 0x3B7F, 0x352C, 0x27D9, 0x298A,
	0x32AA, 0x3CF9, 0x2E0C, 0x205F, 0x0BE6, 0x05B5, 0x1740, 0x1913,
	0x0655, 0x0806, 0x1AF3, 0x14A0, 0x3F19, 0x314A, 0x23BF, 0x2DEC,
	0x1D33, 0x1360, 0x0195, 0x0FC6, 0x247F, 0x2A2C, 0x38D9, 0x368A,
	0x29CC, 0x279F, 0x356A, 0x3B39, 0x1080, 0x1ED3, 0x0C26, 0x0275,
	0x2BFF, 0x25AC, 0x3759, 0x390A, 0x12B3, 0x1CE0, 0x0E15, 0x0046,
	0x1F00, 0x1153, 0x03A6, 0x0DF5, 0x264C, 0x281F, 0x3AEA, 0x34B9,
	0x0466, 0x0A35, 0x18C0, 0x1693, 0x3D2A, 0x3379, 0x218C, 0x2FDF,
	0x3099, 0x3ECA, 0x2C3F, 0x226C, 0x09D5, 0x0786, 0x1573, 0x1B20,
	0x2333, 0x2D60, 0x3F95, 0x31C6, 0x1A7F, 0x142C, 0x06D9, 0x088A,
	0x17CC, 0x199F, 0x0B6A, 0x0539, 0x2E80, 0x20D3, 0x3226, 0x3C75,
	0x0CAA, 0x02F9, 0x100C, 0x1E5F, 0x35E6, 0x3BB5, 0x2940, 0x2713,
	0x3855, 0x3606, 0x24F3, 0x2AA0, 0x0119, 0x0F4A, 0x1DBF, 0x13EC,
	0x3A66, 0x3435, 0x26C0, 0x2893, 0x032A, 0x0D79, 0x1F8C, 0x11DF,
	0x0E99, 0x00CA, 0x123F, 0x1C6C, 0x37D5, 0x3986, 0x2B73, 0x2520,
	0x15FF, 0x1BAC, 0x0959, 0x070A, 0x2CB3, 0x22E0, 0x3015, 0x3E46,
	0x2100, 0x2F53, 0x3DA6, 0x33F5, 0x184C, 0x161F, 0x04EA, 0x0AB9,
	0x1199, 0x1FCA, 0x0D3F, 0x036C, 0x28D5, 0x2686, 0x3473, 0x3A20,
	0x2566, 0x2B35, 0x39C0, 0x3793, 0x1C2A, 0x1279, 0x008C, 0x0EDF,
	0x3E00, 0x3053, 0x22A6, 0x2CF5, 0x074C, 0x091F, 0x1BEA, 0x15B9,
	0x0AFF, 0x04AC, 0x1659, 0x180A, 0x33B3, 0x3DE0, 0x2F15, 0x2146,
	0x08CC, 0x069F, 0x146A, 0x1A39, 0x3180, 0x3FD3, 0x2D26, 0x2375,
	0x3C33, 0x3260, 0x2095, 0x2EC6, 0x057F, 0x0B2C, 0x19D9, 0x178A,
	0x2755, 0x2906, 0x3BF3, 0x35A0, 0x1E19, 0x104A, 0x02BF, 0x0CEC,
	0x13AA, 0x1DF9, 0x0F0C, 0x015F, 0x2AE6, 0x24B5, 0x3640, 0x3813
};

unsigned short int GetCrc_16(unsigned char*pData, int nLength, unsigned short int init)
{
    unsigned short int cRc_16 = init;
    pData++;                            //first digit not involved
    for(; nLength>0; nLength--)
    {
        cRc_16 = (cRc_16 >> 8) ^ CRC16_List[(cRc_16 ^*pData) & 0xff];
        pData++;
    }
    return cRc_16;
}

bool CMD_Send_Robot_move_CMD( serial::Serial& sp,float moveAcc,float moveAngle,float moveSpeed,float rotateAcc,float rotateSpeed)
{
    //Value assignment
    SerialTxFrame serialTxFrame;
    serialTxFrame.txFrame.CMType = 0x00;        //unsigned char
    serialTxFrame.txFrame.ID = 0x00;            //unsigned char
    serialTxFrame.txFrame.dataLength = 20;      //unsigned char

    serialTxFrame.txFrame.moveAcc = moveAcc;    //float
    serialTxFrame.txFrame.moveSpeed = moveSpeed;//float
    serialTxFrame.txFrame.moveAngle = moveAngle;//float
    serialTxFrame.txFrame.rotateAcc = rotateAcc;//float
    serialTxFrame.txFrame.rotateSpeed = rotateSpeed;//float
    
    //Calculate the CRC Value
    serialTxFrame.txFrame.CRCCheck = GetCrc_16(serialTxFrame.buffer,FRAME_SEND_LENGTH-3, 0x2B3B);

    //write into the Physical SerialPort
    sp.write(serialTxFrame.buffer , FRAME_SEND_LENGTH);
    return true;
}

/*------------------receive函数模板------------------
void serial_recv(uint8_t *recvBuffer, size_t recvSize)
{
    do
    {
        do
        {
            recvBuffer[0]=0;

        }
    }
    while (1);
    
}
------------------------------------------------------*/

void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
    float moveSpeed;
    float moveAngle;
    float rotateSpeed;

    if(param_use_debug_cmd)
    {
        ROS_INFO("cmd_vel Line[%1.2f,%1.2f,%1.2f], Ang[%1.2f,%1.2f,%1.2f]", 
                             msg->linear.x,  msg->linear.y,  msg->linear.z, 
                             msg->angular.x, msg->angular.y, msg->angular.z);
    }
    
    moveSpeed = sqrt(pow(msg->linear.x,2)+pow(msg->linear.y,2));//[-0.1, 0.1];(MAX Constrained by move_base yaml files)
    moveAngle = (float)atan2(msg->linear.y,msg->linear.x);//(-pi, pi](Constrained by move_base yaml files)
    rotateSpeed = msg->angular.z*180/M_PI;//from radian to [-30,30] degree;(MAX Constrained by move_base yaml files)
    linear_vel_x = msg->linear.x;           //used in main(), to send back velocity info to ROS topic net
    linear_vel_y = msg->linear.y;
    angular_vel_z = msg->angular.z;
    //cout<< "moveAngle:" <<moveAngle/M_PI*180<<endl;
    CMD_Send_Robot_move_CMD(sp, 0.5, moveAngle, moveSpeed, 45, rotateSpeed);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    //创建一个serial类并设置（1.创建timeout；2.设置要打开的串口名称；3.设置串口通信的波特率；4.串口设置timeout）
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort("/dev/agv");
    sp.setBaudrate(115200);
    sp.setTimeout(to);
    //Publisher and Subscriber definition
    ros::Subscriber sub_cmd_vel = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, callback_cmd_vel);
    ros::Publisher pub_raw_vel = n.advertise<geometry_msgs::TwistStamped>("raw_vel",1000);//topic_name: "raw_vel"
    //Global Variable
    geometry_msgs::TwistStamped pub_msg_vel;//send back velocity info to ROS topic net
   

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
        ROS_INFO_STREAM("/dev/agv is opened.");
    }
    else
    {
        return -1;
    }
    
    ros::Rate loop_rate(5);
    while(ros::ok())
    {
        /*------------------串口操作样例代码------------------
        //获取缓冲区内的Byte数
        size_t n = sp.available();
        if(n!=0)
        {
            //缓冲区内数据读出到内存
            uint8_t buffer[1024];
            n = sp.read(buffer, n);
            for(int i=0; i<n; i++)
            {
                //16进制的方式打印到屏幕 (1 Byte = 8 bit = 2位十六进制0xff)
                std::cout << std::hex << (buffer[i] & 0xff) << " ";
            }
            std::cout << std::endl;
            //把数据发送回去
            sp.write(buffer, n);
        }
        ------------------串口操作样例代码------------------*/

        //--------Get "cmd_vel" topic and Call "callback_cmd_vel": Send Command to AGV--------
        ros::spinOnce();

        //--------Publish "pub_raw_vel" to ROS: data directly from Command "cmd_vel";
        pub_msg_vel.header.stamp = ros::Time::now();
        pub_msg_vel.header.frame_id = "detec_vel";
        pub_msg_vel.twist.linear.x = linear_vel_x;
        pub_msg_vel.twist.linear.y = linear_vel_y;
        pub_msg_vel.twist.linear.z = 0;
        pub_msg_vel.twist.angular.x = 0;
        pub_msg_vel.twist.angular.y = 0;
        pub_msg_vel.twist.angular.z = angular_vel_z;
        pub_raw_vel.publish(pub_msg_vel);

        loop_rate.sleep();
    }
    //关闭串口
    sp.close();
    ROS_INFO("serial_trans turned down");
    return 0;
}