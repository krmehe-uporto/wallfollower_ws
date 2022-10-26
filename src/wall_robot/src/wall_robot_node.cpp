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
float threashold=1;
float sens_front,sens_right_back, sens_right_front;
double odom_x, odom_y;
auto traveled_path_filename=std::chrono::system_clock::now();
auto lap_start=std::chrono::system_clock::now();
auto lap_end=std::chrono::system_clock::now();

void LaserMsgRecived ( const sensor_msgs::LaserScan& laser_msg) {
     
     for (size_t i=0;i<laser_msg.ranges.size();i++){
          ldat_old[i]=ldat[i];
          ldat[i]=laser_msg.ranges.operator[](i); 
          //ROS_INFO_STREAM("data["<<i<<"]"<< ldat[i]);
     }
     //5 = -112.5 deg
     //0 = -67.5 deg 
     //2 = 0 deg
     
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
     // creating travaled path file nam based on lap start time
     std::ofstream travaled_path;
     std::time_t lap_start_t = std::chrono::system_clock::to_time_t(lap_start);
     std::stringstream datetime;
     datetime << std::put_time(std::localtime(&lap_start_t), "%Y-%m-%d %X");

     travaled_path.open(datetime.str()+".csv",std::ios_base::app);
     // printing odometry data to file
     travaled_path << odom_x << ";" << odom_y << "\n";

     travaled_path.close(); 

     //ROS_INFO_STREAM("odom:"<< odom_x <<" "<< odom_y);
     
}
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
     std::time_t lap_start_t = std::chrono::system_clock::to_time_t(lap_start);

     ROS_INFO_STREAM("Model opened at " << std::ctime(&lap_start_t));
}   
void DeleteModel(){  
     lap_end=std::chrono::system_clock::now();
     //calculating elapsed time between lap start and finish
     std::chrono::duration<double> elapse_time=lap_end.time_since_epoch()-lap_start.time_since_epoch();
     std::time_t lap_start_t = std::chrono::system_clock::to_time_t(lap_start);
     //getting the file name based on lap start time
     std::stringstream datetime;
     datetime << std::put_time(std::localtime(&lap_start_t), "%Y-%m-%d %X");
     std::ofstream travaled_path;

     travaled_path.open(datetime.str()+".csv",std::ios_base::app);
     //saving lap time at then end of the traveled path csv file
     travaled_path << "Duration of lap " << elapse_time.count()<<" s";

     travaled_path.close(); 

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
     //model deleted, printing data
     
     ROS_INFO_STREAM("Model deleted, lap time: "<< elapse_time.count()<<" s");
}
     

     

int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "wall_follower_robot");
     ros::NodeHandle nh;
     srand((unsigned) time(NULL));
     //delete the existing model from the map and launch a new one in a random position
     DeleteModel();
     RelaunchModel();
     //Ceate the publisher for velocity commands
     ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/wall_r/cmd_vel", 100);
     //read laser data 
     ros::Subscriber laser_sub=nh.subscribe("/wall_r/scan",1000,LaserMsgRecived);
     //read odometry daat
     ros::Subscriber odom_sub=nh.subscribe("/wall_r/odometry/ground_truth",1000,OdometryMsgRecived);


     //Sets the loop to publish at a rate of 10Hz
     ros::Rate rate(10);
     //Declares the message to be sent
               geometry_msgs::Twist msg;
       int ifcase;  
         
       while(ros::ok()) {
          alpha=atan2(sens_right_back-ldat_old[0],msg.linear.x*dt);
          
          
          ifcase=0;
          double w=2;
          double v=1; //0.1
          if (isnan(sens_right_back) && isnan(sens_right_front) && isnan(sens_front)){
               msg.angular.z=w;
               msg.linear.x=0;
               ifcase=1;
          }
          else if (sens_front>2*threashold && ((isnan(sens_right_back) && isnan(sens_right_front)) || (sens_right_back>1.5*threashold && sens_right_front>1.5*threashold ))) {
               msg.angular.z=0;
               msg.linear.x=v; //1
               ifcase=2;
          }
          else if (sens_front<2*threashold ){ 
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
                    float avg=(sens_right_back+sens_right_front)/2;
                    msg.angular.z=(sens_right_back-sens_right_front)*k+5*(+threashold-avg);

                    if (msg.angular.z>5) msg.angular.z=5; //max angular velocity
                    if (msg.angular.z<-5) msg.angular.z=-5; //max angular velocity

                    ifcase=7;
               }
          }
          

          //Publish the message
          //if (data_rcvd==1){
           ROS_INFO_STREAM("move z:"<< msg.angular.z <<" x: "<< msg.linear.x<< " case: "<< ifcase);
           pub.publish(msg);
           data_rcvd=0;
          //} 

          //reaching the endpoint
          if (odom_y<-9.5){
               odom_y=0;
               //stop the robot
               msg.angular.z=0;
               msg.linear.x=0; 
               pub.publish(msg);
               ROS_INFO_STREAM("Target reached");
               //unsubscribe from the topics so no new data comes in
               odom_sub.shutdown();
               laser_sub.shutdown();
               pub.shutdown();
               //relaunch subscribes to flush the alreay buffered data
               pub=nh.advertise<geometry_msgs::Twist>("/wall_r/cmd_vel", 100);
               laser_sub=nh.subscribe("/wall_r/scan",1000,LaserMsgRecived);
               odom_sub=nh.subscribe("/wall_r//odometry/ground_truth",1000,OdometryMsgRecived);
              
               odom_sub.shutdown();
               laser_sub.shutdown();
               pub.shutdown();
               
               DeleteModel(); 
               //wait 2 seconds to give time to cancel the 
               sleep(2);
               //spawning new model
               RelaunchModel();
               //resubscribe to topics

               pub=nh.advertise<geometry_msgs::Twist>("/wall_r/cmd_vel", 100);
               laser_sub=nh.subscribe("/wall_r/scan",1000,LaserMsgRecived);
               odom_sub=nh.subscribe("/wall_r//odometry/ground_truth",1000,OdometryMsgRecived);
               
               

          }
          //Delays for the loop to not run too fast
          rate.sleep();
          ros::spinOnce();
        }
}
