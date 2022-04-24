#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <vector>
#include <math.h>

turtlesim::Pose turtle_pose;

//Subscriber and Publisher
ros::Subscriber pose_sub;
ros::Publisher vel_pub;

//Services
ros::ServiceClient spawn_client;            // to add a spawn service
ros::ServiceClient kill_client;             // to add a kill service

//Create the request and response objects of spawn
turtlesim::Spawn::Request spawn_req;        // need a req. and resp. for spawn
turtlesim::Spawn::Response spawn_resp;

//Create the request and response objects of kill
turtlesim::Kill::Request kill_req;          // need a req. and resp. for kill
turtlesim::Kill::Response kill_resp;

//Methods declaration
void pose_callback(const turtlesim::Pose::ConstPtr &pose_msg);
void move_to(turtlesim::Pose goal_pose, double tolerance);
double PID_distance(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt);
double PID_angle(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt);
void spawnTurtle(turtlesim::Pose leader_pose);
void killTurtle(void);



int main(int argc, char **argv){
    ros::init(argc, argv, "move");
    ros::NodeHandle n;
    pose_sub = n.subscribe("turtle1/pose", 10, pose_callback);
    vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    // make the service to the spawn client
    spawn_client = n.serviceClient<turtlesim::Spawn>("/spawn");
    // make the service to the kill client
    kill_client = n.serviceClient<turtlesim::Kill>("/kill");
    
    turtlesim::Pose goal_pose;
    
    while(1){
	    goal_pose.x = random() % 12;
	    goal_pose.y = random() % 12;
	    
	    ROS_INFO_STREAM("going to : " << "x = " <<goal_pose.x << "\t y = "<< goal_pose.y );
	    
	    spawnTurtle(goal_pose); 
	    move_to(goal_pose, 0.1);
	    killTurtle();        
    }
    return 0;
}


//Methods definition
void pose_callback(const turtlesim::Pose::ConstPtr &pose_msg){
    turtle_pose = *pose_msg; 
}

void move_to(turtlesim::Pose goal_pose, double tolerance){
     geometry_msgs::Twist turtle_vel;
     ros::Rate loop_rate(10);
     
     std::vector<double> KpKiKd_distance{1.01, 0.2, 0.001};
     std::vector<double> KpKiKd_angle{3.5, 0.05, 0.05};
     double d;
     double dt;
     
     ros::Time start = ros::Time::now();
     ros::Time end = ros::Time::now() + ros::Duration(0.1);
     
     do{
           end = ros::Time::now();
           dt = end.toSec() - start.toSec();
           d = PID_distance(KpKiKd_distance, goal_pose, turtle_pose, 1.0/10);
           start = ros::Time::now();
           turtle_vel.linear.x = d;
           turtle_vel.angular.z = PID_angle(KpKiKd_angle, goal_pose, turtle_pose, 1.0/10);
           vel_pub.publish(turtle_vel);
           ros::spinOnce();
           loop_rate.sleep();
     }while(d > tolerance);
     
     turtle_vel.linear.x = 0;
     turtle_vel.angular.z = 0;
     vel_pub.publish(turtle_vel);
}

double PID_distance(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt) {
    turtlesim::Pose e;
    static double prev_error;
    static double derror;
    
    double Kd = Ks.back(); Ks.pop_back();
    double Ki = Ks.back(); Ks.pop_back();
    double Kp = Ks.back(); Ks.pop_back();

    e.x = setpoint_pose.x - turtle_pose.x;
    e.y = setpoint_pose.y - turtle_pose.y;
    double error = sqrt(pow(e.x,2) + pow(e.y,2));

    derror = error - prev_error;    
    double u = Kp*error + Ki*error*dt + Kd*derror/dt;
    prev_error = error;
    return u;
}

double PID_angle(std::vector<double> Ks, turtlesim::Pose setpoint_pose, turtlesim::Pose turtle_pose, double dt) {
    turtlesim::Pose e;
    static double prev_error;
    static double derror;

    double Kd = Ks.back(); Ks.pop_back();
    double Ki = Ks.back(); Ks.pop_back();
    double Kp = Ks.back(); Ks.pop_back();
    
    double error = atan2(setpoint_pose.y - turtle_pose.y, setpoint_pose.x - turtle_pose.x) - turtle_pose.theta;
    derror = error - prev_error;
    double u = Kp*error + Ki*error*dt + Kd*derror/dt;
    
    prev_error = error;
    
    u = u >  4*M_PI? 4*M_PI:u;
    u = u < -4*M_PI?-4*M_PI:u;
    return u;
}

void spawnTurtle(turtlesim::Pose leader_pose) {
   spawn_req.x = leader_pose.x;
   spawn_req.y = leader_pose.y;
   spawn_req.theta = leader_pose.theta;
   spawn_req.name = "Leader";
   ros::service::waitForService("/spawn", ros::Duration(5));
   bool success = spawn_client.call(spawn_req, spawn_resp);
   if(success){
       ROS_INFO_STREAM("Reborn like fenix turtle named " << spawn_resp.name);
   }else{
       ROS_ERROR_STREAM("Failed to spawn turtle named " << spawn_resp.name);
   }
}

void killTurtle(void) {
   kill_req.name = spawn_resp.name;
   ros::service::waitForService("/kill", ros::Duration(5));
   bool success = kill_client.call(kill_req, kill_resp);
   if(success){
       ROS_INFO_STREAM("Killed beloved turtle named " << spawn_resp.name);
   }else{
       ROS_ERROR_STREAM("Failed to kill turtle named " << spawn_resp.name);
   } 
}


