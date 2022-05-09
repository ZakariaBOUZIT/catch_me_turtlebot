#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


//Subscriber and Publisher
ros::Subscriber vel_turtlesim_sub;
ros::Publisher vel_turtlebot_pub;

geometry_msgs::Twist turtlebot_vel;

void vel_callback(const geometry_msgs::Twist msg){
    turtlebot_vel = msg;
    ROS_INFO_STREAM("turtlebot says : " << msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Turtlebot_move");
    ros::NodeHandle n;
    vel_turtlesim_sub = n.subscribe("turtle1/cmd_vel", 10, vel_callback);
    vel_turtlebot_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    ros::Rate loop_rate(10);
    while(ros::ok()){
        vel_turtlebot_pub.publish(turtlebot_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


