#include <ros/ros.h>
#include <turtlesim/Pose.h>

turtlesim::Pose turtle_pose;

void pose_callback(const turtlesim::Pose::ConstPtr pose_msg){
    ROS_INFO("listening to turtle pose :");
    turtle_pose = *pose_msg; 
    ROS_INFO_STREAM("x : " <<turtle_pose.x << "\ty : " <<turtle_pose.y << "\ttheta : " <<turtle_pose.theta);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "pose_listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("turtle1/pose", 10, pose_callback);
    ros::spin();
    return 0;
}
