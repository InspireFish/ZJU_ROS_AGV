#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h> //subscribe the CMD_VEL
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>


#include <unistd.h>
#include <sys/ioctl.h>
using namespace std;
 
 
char GetInput()
{
    //fd_set 为long型数组
    //其每个元素都能和打开的文件句柄建立联系
    fd_set rfds;
    struct timeval tv;
    char c = '\0';
 
    //将rfds数组清零
    FD_ZERO(&rfds);
    //将rfds的第0位置为１，这样fd=1的文件描述符就添加到了rfds中
    //最初　rfds为00000000,添加后变为10000000
    FD_SET(0, &rfds);
    tv.tv_sec = 1;
    tv.tv_usec = 0; //设置等待超时时间
 
    //检测键盘是否有输入
    //由内核根据io状态修改rfds的内容，来判断执行了select的进程哪个句柄可读
    if (select(1, &rfds, NULL, NULL, &tv) > 0)
    {
        c = getchar();
        return c;
    }
 
    //没有数据返回n
    return '\0';
}

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
 
#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57

#define LOOP_OUT 0x6F

class KeyBoardControlNode
{
    public:
        double vel_x;
        double vel_y;
        double angular_z;
        geometry_msgs::Twist cmdvel_;//用于发布topic
        ros::Publisher pub_;//声明Publisher Obj
        

    KeyBoardControlNode(ros::NodeHandle nh)
    {
        vel_x = 0;
        vel_y = 0;
        angular_z = 0;
        pub_= nh.advertise<geometry_msgs::Twist>("cmd_vel",10);//创建publisher对象
    }
    ~KeyBoardControlNode() { }
    
    //接收键盘指令的method
    void keyboardLoop();
        
    //停止机器人的method
    void stopRobot()
    {
        cmdvel_.linear.x = 0.0;
        cmdvel_.linear.y = 0.0;
        cmdvel_.angular.z = 0.0;
        pub_.publish(cmdvel_);
    }
};


void KeyBoardControlNode::keyboardLoop()
{
    bool dirty = false;
    char c;
    
    cout<<"Reading From Keyboard"<<endl;

    ros::Rate loop_rate(1);//循环间隔 seconds
    for(;;)
    {
        //读入键盘输入（非阻塞）
        c = GetInput();

        if(c == LOOP_OUT)//按键盘”o“结束控制
        {
            break;
        }
        if (dirty == true)
        {
            stopRobot();
            dirty = false;
        }
        
        switch(c)
        {
            case KEYCODE_W:
                vel_x = 0;
                vel_y = 0.15;
                angular_z = 0;
                dirty = true;
                cout<<"moving forward"<<endl;
                break;
            case KEYCODE_S:
                vel_x = 0;
                vel_y = -0.15;
                angular_z = 0;
                dirty = true;
                cout<<"moving back"<<endl;
                break;
            case KEYCODE_A:
                vel_x = -0.15;
                vel_y = 0;
                angular_z = 0;
                dirty = true;
                cout<<"moving left"<<endl;
                break;
            case KEYCODE_D:
                vel_x = 0.15;
                vel_y = 0;
                angular_z = 0;
                dirty = true;
                cout<<"moving right"<<endl;
                break;
            case KEYCODE_A_CAP:
                vel_x = 0;
                vel_y = 0;
                angular_z = 0.15;
                dirty = true;
                cout<<"turning left"<<endl;
                break;
            case KEYCODE_D_CAP:
                vel_x = 0;
                vel_y = 0;
                angular_z = -0.15;
                dirty = true;
                cout<<"turning right"<<endl;
                break;
            default:
                vel_x = 0;
                vel_y = 0;
                angular_z = 0;
                dirty = false;
        }
        //发布vel_x; vel_y; angular_z;三个速度参数
        cmdvel_.linear.x = vel_x;
        cmdvel_.linear.y = vel_y;
        cmdvel_.angular.z = angular_z;
        //发布行动topic
        pub_.publish(cmdvel_);  //ros::Publisher pub_;
                                //geometry_msgs::Twist cmdvel_;
                                //ros::NodeHandle n_;
        loop_rate.sleep();
    }//end for-loop
}//end: method keyboardLoop




int main(int argc, char** argv)
{
    //关闭缓存区，使从终端接收一个字符不用按回车
   system("stty -icanon"); 

   ros::init(argc, argv, "keyboardControl");//创建节点
   ros::NodeHandle nh;//创建句柄

   KeyBoardControlNode control_obj(nh);

   control_obj.keyboardLoop();//循环接收、发送消息
   cout<< "keyboardControl to the end"<< endl;
    return(0);
}
