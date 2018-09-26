#include "hrwros_gazebo/conveyor_spawner.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "conveyor_object_spawner");
  ros::NodeHandle pnh("~"), nh;

  XmlRpc::XmlRpcValue spawner_params;
  if(!pnh.getParam("spawner", spawner_params))
  {
    ROS_ERROR("Failed to get spawner parameters");
    return -1;
  }

  hrwros::simulation::ConveyorSpawner spawner (nh);
  if(!spawner.init(spawner_params))
  {
    ROS_ERROR("Failed to initialize spawner");
    return -2;
  }

  spawner.run();

  return 0;
}
