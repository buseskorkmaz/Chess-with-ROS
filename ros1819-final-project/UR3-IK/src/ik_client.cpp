#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sstream>

#include <ur3ik/UR3IK.h>


int main( int argc, char** argv )
{
    ros::init(argc, argv, "ik_client_node");
    ros::NodeHandle node;

    ROS_INFO("**** starting ik client node ****");

    ros::service::waitForService("/UR3IK");
    ros::ServiceClient ur3ik_client = node.serviceClient<ur3ik::UR3IK>("/UR3IK");
    ur3ik::UR3IK ur3ik_srv;
      
    ur3ik_srv.request.theta_ref.resize(6);
    ur3ik_srv.request.theta_min.resize(6);
    ur3ik_srv.request.theta_max.resize(6);
    for(int i=0; i<6; i++) {
        ur3ik_srv.request.theta_ref[i] = 0.0;
        ur3ik_srv.request.theta_min[i] = -M_PI;
        ur3ik_srv.request.theta_max[i] = M_PI;
    }
    ur3ik_srv.request.pose.position.x = 0.422;
    ur3ik_srv.request.pose.position.y = -0.039;
    ur3ik_srv.request.pose.position.z = 0.100;
    ur3ik_srv.request.pose.orientation.x = 0.707;
    ur3ik_srv.request.pose.orientation.y = -0.707;
    ur3ik_srv.request.pose.orientation.z = 0.0;
    ur3ik_srv.request.pose.orientation.w = 0.0;
        
    
    ROS_INFO_STREAM("Robot Pose: [" << 
        ur3ik_srv.request.pose.position.x << ", " <<
        ur3ik_srv.request.pose.position.y << ", " <<
        ur3ik_srv.request.pose.position.z << ", " << 
        ur3ik_srv.request.pose.orientation.x << ", " << 
        ur3ik_srv.request.pose.orientation.y << ", " <<
        ur3ik_srv.request.pose.orientation.z << ", " <<
        ur3ik_srv.request.pose.orientation.w << "]");
    
    ur3ik_client.call(ur3ik_srv);
    
    std::stringstream sstr;
    if(ur3ik_srv.response.status)
    {
        sstr<<"The computed ik is:"<<std::endl;
        for(int i=0; i<ur3ik_srv.response.ik_solution.size(); i++)
        {
            sstr << "[";
            for(int j=0; j<5; j++)
            {
                sstr << ur3ik_srv.response.ik_solution[i].ik[j] <<", ";            
            }
            sstr << ur3ik_srv.response.ik_solution[i].ik[5] << "]" << std::endl;
        }
        ROS_INFO_STREAM(sstr.str());
    }
    else{
        ROS_INFO("Not able to compute the ik");
    }
     
}
