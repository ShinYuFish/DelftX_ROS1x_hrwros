#include <ros/ros.h>
#include "hrwros_gazebo/urdf_creator.h"
#include <gazebo_msgs/SpawnModel.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "urdf_creator_test");
  ros::NodeHandle nh, pnh("~");

  std::string reference_frame;
  pnh.param<std::string>("reference_frame", reference_frame, "world");

  std::string object_name;
  pnh.param<std::string>("object_name", object_name, "test");

  std::string mesh_resource;
  pnh.param<std::string>("mesh_resource", mesh_resource, "package://hrwros_gazebo/meshes/conveyor_objects/gear.stl");

  std::vector<double> initial_position;
  pnh.param<std::vector<double>>("initial_position", initial_position, {1.2, 1.0, 1.0});

  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_urdf_model");
  if(!client.waitForExistence(ros::Duration(10.0f)))
  {
    ROS_ERROR("Timeout waiting for service");
    return -1;
  }

  std::string parsed_xml = hrwros::simulation::createObjectURDF("gear", mesh_resource);

  gazebo_msgs::SpawnModel srv;
  srv.request.model_name = object_name;
  srv.request.model_xml = parsed_xml;
  srv.request.initial_pose.orientation.w = 1.0;
  srv.request.initial_pose.position.x = initial_position[0];
  srv.request.initial_pose.position.y = initial_position[1];
  srv.request.initial_pose.position.z = initial_position[2];
  srv.request.reference_frame = reference_frame;
  srv.request.robot_namespace = "hrwros";

  if(!client.call(srv))
  {
    ROS_ERROR("Failed to call service");
    return -1;
  }
  else
  {
    if(srv.response.success)
    {
      ROS_INFO("%s", srv.response.status_message.c_str());
    }
    else
    {
      ROS_ERROR("%s", srv.response.status_message.c_str());
      return -1;
    }
  }

  return 0;
}
