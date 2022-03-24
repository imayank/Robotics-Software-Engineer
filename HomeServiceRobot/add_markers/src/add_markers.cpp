#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <iostream>

ros::Publisher marker_pub;
ros::Subscriber state_sub;
visualization_msgs::Marker marker;
uint32_t shape = visualization_msgs::Marker::CUBE;


void hide_marker(){
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
}

void display_marker(double x, double y){
  
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
  
}

void displayCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  
  if(msg->data.compare("PICK")==0){
    double x,y;
    if(ros::param::get("add_markers/pick_x",x) && ros::param::get("add_markers/pick_y",y))
    {
      display_marker(x,y);
    }
    else
      ROS_INFO_STREAM("Failed to diplay at pickup");
    
  }
  else{
    if(msg->data.compare("TRANSIT")==0){
      hide_marker();
    }
    else{
      if(msg->data.compare("DROP")==0){
        double x,y;
        if(ros::param::get("add_markers/drop_x",x) && ros::param::get("add_markers/drop_y",y))
          display_marker(x,y);
        else
          ROS_INFO_STREAM("FAILED to diplay at drop");
      }
    }
  }
}



int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Duration d(5);
  
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
  
  
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  state_sub = n.subscribe("nav_state", 1000, displayCallback);

  ros::spin();
  return 0;
}