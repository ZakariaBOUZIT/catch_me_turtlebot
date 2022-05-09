#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

ros::Subscriber scan_sub;
ros::Publisher vel_pub;

float scan_readings;
geometry_msgs::Twist move;

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    
    scan_readings = msg->ranges[300];
    ROS_INFO_STREAM("scan : " << scan_readings);

    if (scan_readings > 2){
        ROS_INFO_STREAM("Move forward");
        move.linear.x = 0.5;
        move.angular.z = 0;
        vel_pub.publish(move);
    }
    else{
        ROS_INFO_STREAM("Turn");
        move.linear.x = 0;
        move.angular.z = 0.5;
        vel_pub.publish(move);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "scan_move");
    ros::NodeHandle n;

    //scan subscriber
    scan_sub = n.subscribe("/scan", 2, scan_callback);

    //cmd_vel publisher
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 2);

    ros::Rate loop_rate(2);
    
    ros::spin();

    return 0;
}