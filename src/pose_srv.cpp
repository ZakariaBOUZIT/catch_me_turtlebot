#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>

ros::ServiceClient pose_client;
turtlesim::Spawn::Request pose_req; 
turtlesim::Spawn::Response pose_resp; 

void pose_callback_srv(turtlesim::Pose pose){
     pose_req.x = pose.x;
     pose_req.y = pose.y;
     pose_req.theta = pose.theta;
     pose_req.name = "A_turtle";
     ros::service::waitForService("/spawn", ros::Duration(5));
     if(pose_client.call(pose_req, pose_resp)){
        ROS_INFO_STREAM("received : " << pose_resp);
     }
     else{
        ROS_INFO_STREAM("not received");
     }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pose_listener");
    ros::NodeHandle n;
    pose_client = n.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Pose goal_pose;
    goal_pose.x = 1;
    goal_pose.y = 5;
    pose_callback_srv(goal_pose);
    return 0;
}


