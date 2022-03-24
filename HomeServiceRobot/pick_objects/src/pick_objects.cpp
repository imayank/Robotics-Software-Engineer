#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include<iostream>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher state_pub;
//MoveBaseClient ac("move_base", true);
void publish_state(std::string state){
  
  std_msgs::String msg;
  msg.data = state;
  
  if(state.compare("PICK")==0){
    state_pub.publish(msg);
    ros::spinOnce();
  }
  else{
    if(state.compare("TRANSIT")==0){
      state_pub.publish(msg);
      ros::spinOnce();
      ros::Duration(5).sleep();
    }
    else{
      if(state.compare("DROP")==0){
        state_pub.publish(msg);
        ros::spinOnce();
      }
    }
  }
  
}

void navigate(MoveBaseClient& ac,double x, double y, std::string state){
  
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal x=%f, y=%f",x, y);
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Reached the location");
    publish_state(state);
    //ROS_INFO("DONE");
  }
  else
    ROS_INFO("FAILED");
  
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle nh;
  std::string state;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  state_pub = nh.advertise<std_msgs::String>("nav_state",100);
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  double x, y;
  
  if(nh.getParam("pick_objects/pick_x",x) && nh.getParam("pick_objects/pick_y",y)){
    state = "PICK";
    publish_state(state);
    state="TRANSIT";
    navigate(ac, x, y,state);
    ROS_INFO("Reached at PICK-UP");
  }
  else{
    ROS_INFO("x=%f, y=%f,",x,y);
    ROS_INFO("pickup Param Error");
  }
  
  if(nh.getParam("pick_objects/drop_x",x) && nh.getParam("pick_objects/drop_y",y)){
    state="DROP";
    //publish_state(state);
    navigate(ac, x, y, state);
    ROS_INFO("Reached at DROP-OFF");
  }
  else
    ROS_INFO("drop Param Error");
  
  ros::Duration(240).sleep();

  return 0;
}