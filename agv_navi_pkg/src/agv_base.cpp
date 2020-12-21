#include <ros/ros.h>                    //basic ROS functions
#include <tf/transform_broadcaster.h>   //update TF
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
//using namespace std;

//global variables
double vel_x = 0;
double vel_y = 0;
double vel_ang_z = 0;
double vel_diff_time = 0;

//callback function to read out the velocity data from ROS net
void vel_callback(const geometry_msgs::TwistStamped & vel)
{
    static ros::Time last_vel_time = ros::Time::now();//static variable only initializes once 
    static ros::Time cur_vel_time;

    cur_vel_time = ros::Time::now();
    vel_diff_time = (cur_vel_time - last_vel_time).toSec();
    
    vel_x = vel.twist.linear.x;
    vel_y = vel.twist.linear.y;
    vel_ang_z = vel.twist.angular.z;
    last_vel_time = cur_vel_time;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "agv_base");//创建节点
    ros::NodeHandle n;
    
    ros::Subscriber sub_cmd_vel = n.subscribe("raw_vel", 50, vel_callback);//subscribe AGV's velocity
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);//publish AGV's displacement

    double pos_x = 0;
    double pos_y = 0;
    double dir_z = 0;

    ros::Rate loop_rate(50);//每秒发送1次
    while(n.ok())
    {
        ros::Time current_time = ros::Time::now();
        
    //------------update position of AGV------------
        //calculate angular displacement
        double delta_pos_x = vel_x * vel_diff_time; 
        double delta_pos_y = vel_y * vel_diff_time;
        double delta_ang_z = vel_ang_z * vel_diff_time;
        //calculate current position of the AGV
        pos_x += delta_pos_x;
        pos_y += delta_pos_y;
        dir_z += delta_ang_z;

    //------------update the tf topic------------
        //calculate AGV's heading in quarternion angle
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(delta_ang_z);
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = pos_x;
        odom_trans.transform.translation.y = pos_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        //publish AGV's tf using odom_trans object
        tf::TransformBroadcaster odom_broadcaster;
        odom_broadcaster.sendTransform(odom_trans);

    //------------update the topic "odom"------------
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = pos_x;
        odom.pose.pose.position.y = pos_y;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.angular.x = 0;
        odom.twist.twist.angular.y = 0;
        odom.twist.twist.angular.z = vel_ang_z;
        odom_pub.publish(odom);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
