#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist turtle_vel;

int main(int argc, char **argv){
    ros::init(argc, argv, "pose_talker");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    while(ros::ok()){
          turtle_vel.linear.x = 2;
          pub.publish(turtle_vel);
          ROS_INFO_STREAM("Publish velocity" << turtle_vel.linear.y);
          ros::spinOnce();
    }
    return 0;
}
