#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_msgs/LookupTransformAction.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(5);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("pr2_marker", 1);
  actionlib::SimpleActionClient<tf2_msgs::LookupTransformAction> tf_client("tf2_buffer_server", true);
  ROS_INFO("connecting to tf2_buffer_server...");
  tf_client.waitForServer();
  ROS_INFO("connected to tf2_buffer_server");

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    shape = visualization_msgs::Marker::CUBE;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    tf2_msgs::LookupTransformGoal tf_goal;
    tf_goal.target_frame = "map";
    tf_goal.source_frame = "odom_combined";
    tf_goal.timeout = ros::Duration(5.0);
    tf_client.sendGoal(tf_goal);
    tf_client.waitForResult();
    tf2_msgs::LookupTransformResult::ConstPtr resultptr = tf_client.getResult();

    marker.pose.position.x = resultptr->transform.transform.translation.x;
    marker.pose.position.y = resultptr->transform.transform.translation.y;
    marker.pose.position.z = resultptr->transform.transform.translation.z + 1.4/2.0;
    marker.pose.orientation.x = resultptr->transform.transform.rotation.x;
    marker.pose.orientation.y = resultptr->transform.transform.rotation.y;
    marker.pose.orientation.z = resultptr->transform.transform.rotation.z;
    marker.pose.orientation.w = resultptr->transform.transform.rotation.w;

    std::cout << marker.pose << "\n";

    marker.scale.x = 0.65;
    marker.scale.y = 0.65;
    marker.scale.z = 1.40;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.33;

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);
    r.sleep();

    if (!ros::ok()){
      return 0;
    }
  }
}