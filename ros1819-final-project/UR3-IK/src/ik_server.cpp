#include "ros/ros.h"
#include <ur3.h>

#include "ur3ik/UR3IK.h"




bool srvURIK(ur3ik::UR3IK::Request &req, ur3ik::UR3IK::Response &res)
{
    //TRANSFORMATION BETWEEN ROBOT WORLD AND SHOULDER
    //    Eigen::Matrix4d T;
    //    T << -0.57156, -0.104847, 0.813834,   0.05355,
    //         0.616978, -0.708787, 0.341993,    0.0725,
    //         0.540978,  0.697587, 0.469803,   0.41492,
    //              0.0,       0.0,      0.0,       1.0;

    //TRANSFORMATION BETWEEN ROBOT WORLD AND TCP
    Eigen::AffineCompact3d transform;
    transform = Eigen::Translation3d(req.pose.position.x, req.pose.position.y, req.pose.position.z) *
            Eigen::Quaterniond(req.pose.orientation.w,req.pose.orientation.x,
                               req.pose.orientation.y,req.pose.orientation.z);

    UR3::Configuration _theta_ref = {req.theta_ref[0], req.theta_ref[1], req.theta_ref[2],
                                     req.theta_ref[3], req.theta_ref[4], req.theta_ref[5]};
    UR3::Configuration _theta_min = {req.theta_min[0], req.theta_min[1], req.theta_min[2],
                                     req.theta_min[3], req.theta_min[4], req.theta_min[5]};
    UR3::Configuration _theta_max = {req.theta_max[0], req.theta_max[1], req.theta_max[2],
                                     req.theta_max[3], req.theta_max[4], req.theta_max[5]};

    std::vector <UR3::Configuration> _theta;


    //TRANSFORMATION BETWEEN THE SHOULDER AND THE TCP
    //   Pose = T.inverse() * Pose;

    // Eigen::VectorXd init_q(Eigen::VectorXd::Zero(7));
    if (  UR3::solveIK(transform, _theta, _theta_ref, _theta_min, _theta_max) )
    {
        ROS_INFO("IK has been computed successfully.\n");
   
        res.ik_solution.resize(_theta.size());
        for(unsigned int i=0; i<_theta.size(); i++){
            for(unsigned int j=0; j<6; j++)
                res.ik_solution[i].ik.push_back(_theta[i][j]);
        }
        
        std::stringstream sstr;
        sstr<<"The UR-IK solution is: " <<std::endl;
        for(int i=0; i<_theta.size(); i++)
        {
            sstr << "[";
            for(int j=0; j<5; j++)
            {
                sstr << res.ik_solution[i].ik[j] <<", ";            
            }
            sstr << res.ik_solution[i].ik[5] << "]" << std::endl;
        }
        
        ROS_INFO_STREAM(sstr.str());
        res.status = true;
        return true;
    }

    else
    {
        ROS_INFO("The UR-IK has not been computed.\n");
        res.status = false;
        return false;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "UR3IK_node");
    ros::NodeHandle n;

    ROS_INFO("Starting UR3 inverse kinematic service");

    ros::ServiceServer service1 = n.advertiseService("/UR3IK", srvURIK);
    ros::spin();

    return 0;
}
