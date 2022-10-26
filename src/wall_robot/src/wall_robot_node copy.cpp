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

float ldat[3];
float ldat_old[3];
bool data_rcvd;
float alpha,dt;
int range=20;
float threashold=2;
float sens_front,sens_right_back, sens_right_front;
double odom_x, odom_y;
void LaserMsgRecived ( const sensor_msgs::LaserScan& laser_msg) {
     
     for (size_t i=0;i<laser_msg.ranges.size();i++){
          ldat_old[i]=ldat[i];
          ldat[i]=laser_msg.ranges.operator[](i); 
          //ROS_INFO_STREAM("data["<<i<<"]"<< ldat[i]);
     }
     data_rcvd=1;
     dt=laser_msg.scan_time;
     sens_front=ldat[5];
     sens_right_back=ldat[0];
     sens_right_front=ldat[2];
     ROS_INFO_STREAM("laser:"<< sens_right_back <<" "<< sens_right_front << " " << sens_front);
     
}

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

     

     

int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "wall_follower_robot");
     ros::NodeHandle nh;
     auto lap_start=std::chrono::system_clock::now();
     srand((unsigned) time(NULL));
     //Ceates the publisher, and tells it to publish
     //to the cmd_vel topic, with a queue size of 100
     ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/wall_r/cmd_vel", 100);
     //read laser data 
     ros::Subscriber laser_sub=nh.subscribe("/wall_r/scan",1000,LaserMsgRecived);
     ros::Subscriber odom_sub=nh.subscribe("/wall_r//odometry/ground_truth",1000,OdometryMsgRecived);


     //Sets the loop to publish at a rate of 10Hz
     ros::Rate rate(1);
     //Declares the message to be sent
               geometry_msgs::Twist msg;
       int ifcase;  
         
       while(ros::ok()) {
          alpha=atan2(sens_right_back-ldat_old[0],msg.linear.x*dt);
          
          //0 = -120
          //1 = -60
          //2 = 0
          ifcase=0;
          double w=2;
          double v=1; //0.1
          if (isnan(sens_right_back) && isnan(sens_right_front) && isnan(sens_front)){
               msg.angular.z=w;
               msg.linear.x=0;
               ifcase=1;
          }
          else if (sens_front>threashold && ((isnan(sens_right_back) && isnan(sens_right_front)) || (sens_right_back>threashold && sens_right_front>threashold ))) {
               msg.angular.z=0;
               msg.linear.x=v; //1
               ifcase=2;
          }
          else if (sens_front<threashold ){ //&& ((isnan(sens_right_back) && isnan(sens_right_front))|| (sens_right_back>2 && sens_right_front>2 ))) {
               msg.angular.z=w;
               msg.linear.x=0.5*v; //0.1;
               ifcase=3;
          }

          else if (( !isnan(sens_right_back) && isnan(sens_right_front))) {
               msg.angular.z=-w;
               msg.linear.x= v; //0.2;
               ifcase=4;
          }
          else if (( isnan(sens_right_back) && !isnan(sens_right_front))) {
               msg.angular.z=w;
               msg.linear.x=v ;//0.1;
               ifcase=5;
          }
          else if (( !isnan(sens_right_back) && !isnan(sens_right_front))) {
               msg.linear.x=v ;//0.5;
               
               if (sens_right_back>threashold && sens_right_front>threashold){
                    msg.angular.z=-w;
                    msg.linear.x= v ; //0.1;
                    ifcase=6;
               }
               else {
                    float k=4;
                    msg.angular.z=(sens_right_back-sens_right_front)*k*(+threashold-std::min(sens_right_back,sens_right_front));
                    if (msg.angular.z>5) msg.angular.z=5; //max angular velocity
                    ifcase=7;
               }
          }
          

          //Publish the message
          if (data_rcvd==1){
           ROS_INFO_STREAM("move z:"<< msg.angular.z <<" x: "<< msg.linear.x<< " case: "<< ifcase);
           pub.publish(msg);
           data_rcvd=0;
          } 

          //reaching the endpoint
          if (odom_y<-9.5){
               odom_y=0;
               flatland_msgs::DeleteModel srv;
               srv.request.name = "turtlebot";
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
               
               //spawning new model
               flatland_msgs::SpawnModel srv_spawn;

               srv_spawn.request.name = "turtlebot";
               srv_spawn.request.ns = "wall_r";
               srv_spawn.request.yaml_path = ("../yamls/turtlebot.model.yaml"); //robot_yaml.string();
               srv_spawn.request.pose.x = -4+(double)rand()/RAND_MAX*(2+4);
               srv_spawn.request.pose.y = -3+(double)rand()/RAND_MAX*(3.5+3);
               srv_spawn.request.pose.theta = (double)rand()/RAND_MAX*(M_1_PI);

               client = nh.serviceClient<flatland_msgs::SpawnModel>("spawn_model");
               //srv_spawn.response.success;
               ros::service::waitForService("spawn_model", 1000);
               
               while(!client.call(srv_spawn)){
                    //rate.sleep();
                    ROS_INFO_STREAM("wait for spawn");
               }
               lap_start=std::chrono::system_clock::now();
               ROS_INFO_STREAM("Model opened");

          }
          //Delays untill it is time to send another message
          //rate.sleep();
          ros::spinOnce();
        }
}
