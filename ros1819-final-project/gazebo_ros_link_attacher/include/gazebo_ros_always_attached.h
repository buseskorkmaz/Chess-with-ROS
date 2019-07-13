/*
 * Desc: Gazebo ros always attached plugin.
 * Author: Carlos J. Rosales Gallegos (carlos@beta-robots.com)
 * Date: 03/05/2019
 */

#ifndef GAZEBO_ROS_ALWAYS_ATTACHED_HH
#define GAZEBO_ROS_ALWAYS_ATTACHED_HH

#include <ros/ros.h>

#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include <gazebo/physics/physics.hh>
#include "gazebo/physics/PhysicsTypes.hh"
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"

namespace gazebo
{

   class GazeboRosAlwaysAttached : public WorldPlugin
   {
      public:
        /// \brief Constructor
        GazeboRosAlwaysAttached();

        /// \brief Destructor
        virtual ~GazeboRosAlwaysAttached();

        /// \brief Load the controller
        void Load( physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/ );

        /// \brief Internal representation of a fixed joint
        struct fixedJoint{
            std::string model1;
            physics::ModelPtr m1;
            std::string link1;
            physics::LinkPtr l1;
            std::string model2;
            physics::ModelPtr m2;
            std::string link2;
            physics::LinkPtr l2;
            physics::JointPtr joint;
	    std::vector<double> l2_world_pose;
        };

   private:
        ros::NodeHandle nh_;
        ros::Timer timer_;
        bool attached_;
        void tryAttach(const ros::TimerEvent& _e);

        fixedJoint joint_;

        /// \brief The physics engine.
        physics::PhysicsEnginePtr physics_;

        /// \brief Pointer to the world.
        physics::WorldPtr world_;

   };

}

#endif
