#include <boost/filesystem.hpp>
#include <gazebo_msgs/SpawnModel.h>
#include "hrwros_gazebo/conveyor_spawner.h"
#include "hrwros_gazebo/urdf_creator.h"
#include <random>
#include <ros/package.h>
#include <XmlRpcException.h>
#include <tf/transform_datatypes.h>

const static std::string GAZEBO_SPAWN_SERVICE = "gazebo/spawn_urdf_model";
const static std::string START_SPAWN_SERVICE = "start_spawn";
const static std::string STOP_SPAWN_SERVICE = "stop_spawn";
const static std::string SPAWNED_PART_TOPIC = "spawned_part";
const static double SRV_TIMEOUT = 10.0f;

namespace hrwros
{
namespace simulation
{

ConveyorSpawner::ConveyorSpawner(ros::NodeHandle& nh)
  : nh_(nh)
{
}

bool ConveyorSpawner::init(XmlRpc::XmlRpcValue& p)
{
  // Load the parameters
  if(!loadSpawnParameters(p, params_))
  {
    return false;
  }

  // Connect to ROS topics/services/actions/etc.
  if(!connectToROS())
  {
    return false;
  }

  // Initialize the randomization engine
  srand(params_.randomization_seed);

  start_server_ = nh_.advertiseService(START_SPAWN_SERVICE, &ConveyorSpawner::start, this);
  stop_server_ = nh_.advertiseService(STOP_SPAWN_SERVICE, &ConveyorSpawner::stop, this);
  timer_ = nh_.createTimer(ros::Duration(params_.spawn_period), &ConveyorSpawner::spawnObject, this, false, false);
  pub_ = nh_.advertise<std_msgs::Header>(SPAWNED_PART_TOPIC, 10, true);

  return true;
}

void ConveyorSpawner::run()
{
  ros::spin();
}


bool ConveyorSpawner::loadSpawnParameters(const XmlRpc::XmlRpcValue& p,
                                          SpawnParameters& spawn_params) const
{
  XmlRpc::XmlRpcValue params = p;
  try
  {
    // Get the top-level parameters
    // Get the reference frame
    XmlRpc::XmlRpcValue& frame = params["reference_frame"];
    spawn_params.reference_frame = static_cast<std::string>(frame);

    // Get the spawn timing
    XmlRpc::XmlRpcValue& period = params["spawn_period"];
    spawn_params.spawn_period = static_cast<double>(period);

    // Get the randomization seed
    XmlRpc::XmlRpcValue& seed = params["randomization_seed"];
    spawn_params.randomization_seed = static_cast<int>(seed);

    // Get the spawned objects
    XmlRpc::XmlRpcValue& objects = params["objects"];
    for(int i = 0; i < objects.size(); ++i)
    {
      XmlRpc::XmlRpcValue& obj = objects[i];
      ObjectParameters object_params;
      if(!loadObjectParameters(obj, object_params))
      {
        return false;
      }

      // Add the object's parameters to the list
      spawn_params.objects.push_back(object_params);
    }
  }
  catch(XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR("Exception in loading spawner parameters:\n%s'", ex.getMessage().c_str());
    return false;
  }

  return true;
}

bool ConveyorSpawner::loadObjectParameters(const XmlRpc::XmlRpcValue& object,
                                           ObjectParameters& object_params) const
{
  XmlRpc::XmlRpcValue obj = object;

  try
  {
    // Get the name
    XmlRpc::XmlRpcValue& name = obj["name"];
    object_params.name = static_cast<std::string>(name);

    // Get the relative URDF path file
    XmlRpc::XmlRpcValue& filepath = obj["mesh_resource"];
    object_params.mesh_resource = static_cast<std::string>(filepath);

    // Get the initial pose
    XmlRpc::XmlRpcValue& initial_pose = obj["initial_pose"];

    XmlRpc::XmlRpcValue& position = initial_pose["position"];
    object_params.initial_pose.position.x = static_cast<double>(position[0]);
    object_params.initial_pose.position.y = static_cast<double>(position[1]);
    object_params.initial_pose.position.z = static_cast<double>(position[2]);

    XmlRpc::XmlRpcValue& orientation = initial_pose["orientation"];
    object_params.initial_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(static_cast<double>(orientation[0]),
                                                                               static_cast<double>(orientation[1]),
                                                                               static_cast<double>(orientation[2]));

    // Get the lateral placement variance
    XmlRpc::XmlRpcValue& lpv = obj["lateral_placement_variance"];
    object_params.lateral_placement_variance = static_cast<double>(lpv);

    // Get the yaw placement_variance
    XmlRpc::XmlRpcValue& ypv = obj["yaw_placement_variance"];
    object_params.yaw_placement_variance = static_cast<double>(ypv) * M_PI / 180.0f;

    // Get the spawn timing variance
    XmlRpc::XmlRpcValue& stv = obj["spawn_timing_variance"];
    object_params.spawn_timing_variance = static_cast<double>(stv);
  }
  catch(const XmlRpc::XmlRpcException& ex)
  {
    ROS_ERROR("Exception in loading object parameters:\n%s", ex.getMessage().c_str());
    return false;
  }

  return true;
}

bool ConveyorSpawner::connectToROS()
{
  spawn_client_ = nh_.serviceClient<gazebo_msgs::SpawnModel>(GAZEBO_SPAWN_SERVICE);
  if(!spawn_client_.waitForExistence(ros::Duration(SRV_TIMEOUT)))
  {
    ROS_ERROR("Timeout waiting for '%s' service", spawn_client_.getService().c_str());
    return false;
  }

  return true;
}

bool ConveyorSpawner::start(std_srvs::EmptyRequest& req,
           std_srvs::EmptyResponse& res)
{
  ROS_INFO("Starting conveyor spawner...");
  timer_.start();
  return true;
}

bool ConveyorSpawner::stop(std_srvs::EmptyRequest& req,
          std_srvs::EmptyResponse& res)
{
  ROS_INFO("Stopping conveyor spawner...");
  timer_.stop();
  return true;
}

void ConveyorSpawner::spawnObject(const ros::TimerEvent& e)
{
  ++object_counter_;

  // Randomize the model to be spawned
  int idx = rand() % params_.objects.size();
  auto obj = params_.objects.begin();
  std::advance(obj, idx);

  // Populate the spawn service request
  gazebo_msgs::SpawnModel srv;
  srv.request.reference_frame = params_.reference_frame;
  srv.request.robot_namespace = "hrwros";
  srv.request.model_xml = createObjectURDF(obj->name, obj->mesh_resource);
  srv.request.initial_pose = obj->initial_pose;
  srv.request.model_name = "object_" + std::to_string(object_counter_);

  // Randomize the object's lateral spawn position
  // Create a new pseudo-random number between 0 and 1
  double r_lpv = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);

  double& lpv = obj->lateral_placement_variance;
  double lpv_delta = -lpv + 2.0*r_lpv*lpv;
  srv.request.initial_pose.position.y += lpv_delta;

  // Randomize the object's spawn yaw angle
  // Create a new psuedo-random number between 0 and 1
  double r_ypv = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);

  double& ypv = obj->yaw_placement_variance;
  double ypv_delta = -ypv + 2.0*r_ypv*ypv;
  tf::Quaternion q;
  tf::quaternionMsgToTF(srv.request.initial_pose.orientation, q);
  tf::Quaternion dq = tf::createQuaternionFromRPY(0.0, 0.0, ypv_delta);
  q *= dq;
  tf::quaternionTFToMsg(q, srv.request.initial_pose.orientation);

  // Randomize the delay in spawning the object
  // Create a new psuedo-random number between 0 and 1
  double r_tv = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);

  ros::Duration time_var (r_tv * obj->spawn_timing_variance);
  time_var.sleep();

  // Call the spawn service
  if(!spawn_client_.call(srv))
  {
    ROS_ERROR("Failed to call '%s' service", spawn_client_.getService().c_str());
    --object_counter_;
    return;
  }
  else
  {
    if(!srv.response.success)
    {
      ROS_ERROR("%s", static_cast<std::string>(srv.response.status_message).c_str());
      --object_counter_;
      return;
    }
    else
    {
      // Publish which part was just spawned onto the conveyor
      std_msgs::Header msg;
      msg.frame_id = obj->name;
      msg.stamp = ros::Time::now();
      msg.seq = object_counter_;
      pub_.publish(msg);
    }
  }
}

} // namespace simulation
} // namespace hrwros
