#include <iostream>
#include <ros/ros.h>
//#include <ros/package.h>
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <geometry_msgs/Pose.h>
//#include <sensor_msgs/JointState.h>

#include <chesslab_setup/setrobconf.h>
//#include <chesslab_setup/setobjpose.h>
//#include <chesslab_setup/attachobs2robot.h>
//#include <chesslab_setup/dettachobs.h>
//#include <chesslab_setup/ffplan.h>
//#include <chesslab_setup/ik.h>
#include <std_srvs/Empty.h>

int main( int argc, char** argv )
{
    ros::init(argc, argv, "demo_collision");
    ros::NodeHandle node;

    ROS_INFO("**** starting chesslab_setup client node to test the motion of the robot and the collision-check ****");

    //Set the initial scene
    ros::service::waitForService("/chesslab_setup/resetscene");
    ros::ServiceClient resetscene_client = node.serviceClient<std_srvs::Empty>("/chesslab_setup/resetscene");
    std_srvs::Empty resetscene_srv;
    resetscene_client.call(resetscene_srv);

    ////////////////////////////////////////////////////////
    //Wait for user to press a key
    std::cout<<"\nPRESS A KEY TO START..."<<std::endl;
    std::cin.get();

    //Move robots
    ros::service::waitForService("/chesslab_setup/setrobconf");
    ros::ServiceClient setrobconf_client = node.serviceClient<chesslab_setup::setrobconf>("/chesslab_setup/setrobconf");
    chesslab_setup::setrobconf setrobconf_srv;

    //collision robA with kingB:
    //rosservice call /chesslab_setup/setrobconf "conf: [1.15,-1.2,1.4,1.5,1.5,0.,0.5, 0.9, -1.8, 1.7, -1.6, -1.5, 0., 0.]"
    setrobconf_srv.request.conf.resize(14);
    setrobconf_srv.request.conf[0]  =  1.15;
    setrobconf_srv.request.conf[1]  = -1.2;
    setrobconf_srv.request.conf[2]  =  1.4;
    setrobconf_srv.request.conf[3]  =  1.5;
    setrobconf_srv.request.conf[4]  =  1.5;
    setrobconf_srv.request.conf[5]  =  0.0;
    setrobconf_srv.request.conf[6]  =  0.5;
    setrobconf_srv.request.conf[7]  =  0.9;
    setrobconf_srv.request.conf[8]  = -1.8;
    setrobconf_srv.request.conf[9]  =  1.7;
    setrobconf_srv.request.conf[10] = -1.6;
    setrobconf_srv.request.conf[11] = -1.5;
    setrobconf_srv.request.conf[12] =  0.0;
    setrobconf_srv.request.conf[13] =  0.0;

    std::cout << "Moving robots to: [" <<
        setrobconf_srv.request.conf[0] << ", " <<
        setrobconf_srv.request.conf[1] << ", " <<
        setrobconf_srv.request.conf[2] << ", " <<
        setrobconf_srv.request.conf[3] << ", " <<
        setrobconf_srv.request.conf[4] << ", " <<
        setrobconf_srv.request.conf[5] << ", " <<
        setrobconf_srv.request.conf[6] << ", " <<
        setrobconf_srv.request.conf[7] << ", " <<
        setrobconf_srv.request.conf[8] << ", " <<
        setrobconf_srv.request.conf[9] << ", " <<
        setrobconf_srv.request.conf[10] << ", " <<
        setrobconf_srv.request.conf[11] << ", " <<
        setrobconf_srv.request.conf[12] << ", " <<
        setrobconf_srv.request.conf[13] << "]" << std::endl;

    setrobconf_client.call(setrobconf_srv);

    if(setrobconf_srv.response.incollision == true)
    {
        std::cout << "The configuration is not collision-free"<< std::endl;
        std::cout << setrobconf_srv.response.msg;
        std::cout << "The collided obstacle aruco ID is " << setrobconf_srv.response.obj << std::endl;
    }
    else{
        std::cout << setrobconf_srv.response.msg;
    }

}
