#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "gazebo_ros_always_attached.h"
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(GazeboRosAlwaysAttached)

  // Constructor
  GazeboRosAlwaysAttached::GazeboRosAlwaysAttached()
  {
  }

  // Destructor
  GazeboRosAlwaysAttached::~GazeboRosAlwaysAttached()
  {
  }

  void GazeboRosAlwaysAttached::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->world_ = _world;
    this->physics_ = this->world_->Physics();

    if(_sdf->HasElement("model_1"))
    {
      joint_.model1 = _sdf->GetElement("model_1")->Get<std::string>();
    }
    else
    {
      ROS_ERROR_STREAM("Parameter model_1 is missing, and it is required in order to attach models");
      return;
    }
    if(_sdf->HasElement("link_1"))
    {
      joint_.link1 = _sdf->GetElement("link_1")->Get<std::string>();
    }
    else
    {
      ROS_ERROR_STREAM("Parameter link_1 is missing, and it is required in order to attach models");
      return;
    }
    if(_sdf->HasElement("model_2"))
    {
      joint_.model2 = _sdf->GetElement("model_2")->Get<std::string>();
    }
    else
    {
      ROS_ERROR_STREAM("Parameter model_2 is missing, and it is required in order to attach models");
      return;
    }
    if(_sdf->HasElement("link_2"))
    {
      joint_.link2 = _sdf->GetElement("link_2")->Get<std::string>();
    }
    else
    {
      ROS_ERROR_STREAM("Parameter link_1 is missing, and it is required in order to attach models");
      return;
    }

    std::cout << "ASSUMES WE HAVE ALL THESE VALUES" << std::endl;

      joint_.l2_world_pose.push_back(_sdf->GetElement("link2_world_pose_x")->Get<double>());
      joint_.l2_world_pose.push_back(_sdf->GetElement("link2_world_pose_y")->Get<double>());
      joint_.l2_world_pose.push_back(_sdf->GetElement("link2_world_pose_z")->Get<double>());
      joint_.l2_world_pose.push_back(_sdf->GetElement("link2_world_pose_qx")->Get<double>());
      joint_.l2_world_pose.push_back(_sdf->GetElement("link2_world_pose_qy")->Get<double>());
      joint_.l2_world_pose.push_back(_sdf->GetElement("link2_world_pose_qz")->Get<double>());
      joint_.l2_world_pose.push_back(_sdf->GetElement("link2_world_pose_qw")->Get<double>());
      std::cout << "PRINTING VALUES" << std::endl;
      for(auto a : joint_.l2_world_pose)
        std::cout << a << std::endl;
      if(joint_.l2_world_pose.size() != 7)
      {
            ROS_ERROR_STREAM("World pose must be 7 values: x y z qx qy qz qw");
      }

    attached_ = false;
    timer_ = nh_.createTimer(ros::Duration(2), &GazeboRosAlwaysAttached::tryAttach, this);
  }

  void GazeboRosAlwaysAttached::tryAttach(const ros::TimerEvent& _e)
  {
    if(!attached_)
    {
      // look for any previous instance of the joint first.
      // if we try to create a joint in between two links
      // more than once (even deleting any reference to the first one)
      // gazebo hangs/crashes
      ROS_DEBUG_STREAM("Creating new joint.");

      ROS_DEBUG_STREAM("Getting BasePtr of " << joint_.model1);
      physics::BasePtr b1 = world_->ModelByName(joint_.model1);

      if (b1 == NULL)
      {
        ROS_ERROR_STREAM_ONCE("GazeboRosAlwaysAttached plugin: " << joint_.model1 << " model was not found, ensure the model exists, or remove the plugin.");
        return;
      }
      ROS_DEBUG_STREAM("Getting BasePtr of " << joint_.model2);
      physics::BasePtr b2 = world_->ModelByName(joint_.model2);
      if (b2 == NULL)
      {
        ROS_ERROR_STREAM_ONCE("GazeboRosAlwaysAttached plugin: " << joint_.model2 << " model was not found, ensure the model exists, or remove the plugin.");
        return;
      }

      ROS_DEBUG_STREAM("Casting into ModelPtr");
      physics::ModelPtr m1(dynamic_cast<physics::Model*>(b1.get()));
      joint_.m1 = m1;
      physics::ModelPtr m2(dynamic_cast<physics::Model*>(b2.get()));
      joint_.m2 = m2;

      ROS_DEBUG_STREAM("Getting link: '" << joint_.link1 << "' from model: '" << joint_.model1 << "'");
      physics::LinkPtr l1 = m1->GetLink(joint_.link1);
      if (l1 == NULL)
      {
        ROS_ERROR_STREAM_ONCE("GazeboRosAlwaysAttached plugin: " << joint_.link1 << " link was not found, ensure the link exists, or remove the plugin.");
        return;
      }
      if (l1->GetInertial() == NULL)
      {
          ROS_ERROR_STREAM("link1 inertia is NULL!");
      }
      else
          ROS_DEBUG_STREAM("link1 inertia is not NULL, for example, mass is: " << l1->GetInertial()->Mass());
      joint_.l1 = l1;

      ROS_DEBUG_STREAM("Getting link: '" << joint_.link2 << "' from model: '" << joint_.model2 << "'");
      physics::LinkPtr l2 = m2->GetLink(joint_.link2);
      if (l2 == NULL)
      {
        ROS_ERROR_STREAM_ONCE("GazeboRosAlwaysAttached plugin: " << joint_.link2 << " link was not found, ensure the link exists, or remove the plugin.");
        return;
      }
      if (l2->GetInertial() == NULL)
      {
          ROS_ERROR_STREAM("link2 inertia is NULL!");
      }
      else
          ROS_DEBUG_STREAM("link2 inertia is not NULL, for example, mass is: " << l2->GetInertial()->Mass());
      joint_.l2 = l2;

      ROS_INFO_STREAM("Links are: "  << joint_.l1->GetName() << " and " << joint_.l2->GetName());

      ROS_INFO_STREAM("Creating fixed joint on model: '" << joint_.model1 << "'");
      //      (0.176, 0.457, 0.027, 0.500, -0.500, -0.500, 0.500)
      // joint_.m2->SetLinkWorldPose(ignition::math::Pose3d(0.176, 0.457, 0.027, 0.500, -0.500, -0.500, 0.500), joint_.l2);
      joint_.m2->SetLinkWorldPose(ignition::math::Pose3d(joint_.l2_world_pose.at(0), joint_.l2_world_pose.at(1), joint_.l2_world_pose.at(2), joint_.l2_world_pose.at(3), joint_.l2_world_pose.at(4), joint_.l2_world_pose.at(5), joint_.l2_world_pose.at(6)), joint_.l2);
      joint_.joint = physics_->CreateJoint("fixed", joint_.m1);
      // joint_.m1->CreateJoint("name", "revolute", joint_.l1, joint_.l2);
      ROS_INFO_STREAM("Attach");
     //  joint_.l1->AddChildJoint(joint_.joint);
      joint_.joint->Attach(joint_.l1, joint_.l2);
      // joint_.l2->SetRelativePose(ignition::math::Pose3d());
      // ROS_DEBUG_STREAM("Loading links");
      // joint_.joint->LoadJoints(); // (joint_.l1, joint_.l2, ignition::math::Pose3d());
      ROS_INFO_STREAM("SetModel");
      joint_.joint->SetModel(joint_.m2);

      /*
       * If SetModel is not done we get:
       * ***** Internal Program Error - assertion (this->GetParentModel() != __null)
       failed in void gazebo::physics::Entity::PublishPose():
       /tmp/buildd/gazebo2-2.2.3/gazebo/physics/Entity.cc(225):
       An entity without a parent model should not happen

       * If SetModel is given the same model than CreateJoint given
       * Gazebo crashes with
       * ***** Internal Program Error - assertion (self->inertial != __null)
       failed in static void gazebo::physics::ODELink::MoveCallback(dBodyID):
       /tmp/buildd/gazebo2-2.2.3/gazebo/physics/ode/ODELink.cc(183): Inertial pointer is NULL
       */

      ROS_INFO_STREAM("SetHightstop");
      joint_.joint->SetUpperLimit(0, 0);
      ROS_INFO_STREAM("SetLowStop");
      joint_.joint->SetLowerLimit(0, 0);
      ROS_INFO_STREAM("Init");
      joint_.joint->Init();
      ROS_INFO_STREAM("Attach finished.");
      attached_ = true;
    }
  }

}
