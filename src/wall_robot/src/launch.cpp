#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h> 
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <flatland_msgs/DeleteModel.h>
#include <flatland_msgs/SpawnModel.h>
#include <flatland_msgs/DeleteModel.h>
#include <flatland_msgs/MoveModel.h>
#include <flatland_msgs/SpawnModel.h>
#include <flatland_server/timekeeper.h>
//namespace fs = boost::filesystem;

float sens_front,sens_right_back, sens_right_front;
double odom_x, odom_y;
auto lap_start=std::chrono::system_clock::now();
bool model_exists;

void OdometryMsgRecived ( const nav_msgs::Odometry& odom_msg) {
     
     odom_x=odom_msg.pose.pose.position.x;
     odom_y=odom_msg.pose.pose.position.y;
     
     std::ofstream travaled_path;
     travaled_path.open("travel2.csv",std::ios_base::app);
     
     travaled_path << odom_x << ";" << odom_y << "\n";
     travaled_path.close(); 

     //ROS_INFO_STREAM("odom:"<< odom_x <<" "<< odom_y);
     
}
//travaled_path.open();

     
void RelaunchModel(){   
    srand((unsigned) time(NULL));

    flatland_msgs::SpawnModel srv_spawn;
    srv_spawn.request.ns = "wall_r";
    srv_spawn.request.name = "turtlebot";
    srv_spawn.request.yaml_path = ("../yamls/turtlebot.model.yaml"); //robot_yaml.string();
    srv_spawn.request.pose.x = -4+(double)rand()/RAND_MAX*(2+4);
    srv_spawn.request.pose.y = -3+(double)rand()/RAND_MAX*(3.5+3);
    srv_spawn.request.pose.theta = (double)rand()/RAND_MAX*(M_1_PI);
    ros::ServiceClient client;
    ros::NodeHandle nh;
    client = nh.serviceClient<flatland_msgs::SpawnModel>("spawn_model");
    
    
    ros::service::waitForService("spawn_model", 1000);
        
        while(!client.call(srv_spawn)){
            //rate.sleep();
            ROS_INFO_STREAM("wait for spawn");
        }
        lap_start=std::chrono::system_clock::now();
        ROS_INFO_STREAM("Model opened");
    model_exists=true;
}   
void DeleteModel(){
    flatland_msgs::DeleteModel srv;
    srv.request.name = "turtlebot";
    ros::NodeHandle nh;
    ros::ServiceClient client;
    client = nh.serviceClient<flatland_msgs::DeleteModel>("delete_model");

    ros::service::waitForService("delete_model", 1000);
        
    while(!client.call(srv)){
        //rate.sleep();
        ROS_INFO_STREAM("wait for delete");
    }
    //model deleted, saving and printing data
    auto lap_end=std::chrono::system_clock::now();
    std::chrono::duration<double> elapse_time=lap_end.time_since_epoch()-lap_start.time_since_epoch();
    ROS_INFO_STREAM("Model deleted, time: "<< elapse_time.count()<<" s");

    model_exists=false;
}
int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "wall_follower_launcher");
     ros::NodeHandle nh;


     ros::Subscriber odom_sub=nh.subscribe("/wall_r//odometry/ground_truth",1000,OdometryMsgRecived);

     //Sets the loop to publish at a rate of 10Hz
     ros::Rate rate(1);
    
       while(ros::ok()) {

          //reaching the endpoint
          if (odom_y<-9.5 && model_exists){
                odom_y=0;
               
               
               //spawning new model
               

          }
          //Delays untill it is time to send another message
          //rate.sleep();
          ros::spinOnce();
        }
}
