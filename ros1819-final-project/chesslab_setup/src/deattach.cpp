#include <iostream>
#include <ros/ros.h>
//#include <ros/package.h>
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <geometry_msgs/Pose.h>
//#include <sensor_msgs/JointState.h>

#include <chesslab_setup/setrobconf.h>
//#include <chesslab_setup/setobjpose.h>
#include <chesslab_setup/attachobs2robot.h>
#include <chesslab_setup/dettachobs.h>
//#include <chesslab_setup/ffplan.h>
//#include <chesslab_setup/ik.h>
#include <std_srvs/Empty.h>



int main( int argc, char** argv )
{
    ros::init(argc, argv, "demo_dettach");
    ros::NodeHandle node;

    ROS_INFO("**** starting chesslab_setup client node to test the attach/dettach services ****");
    
    //Set the initial scene
    ros::service::waitForService("/chesslab_setup/resetscene");
    ros::ServiceClient resetscene_client = node.serviceClient<std_srvs::Empty>("/chesslab_setup/resetscene");
    std_srvs::Empty resetscene_srv;
    resetscene_client.call(resetscene_srv);
    
    ////////////////////////////////////////////////////////
    //Wait for user to press a key
    std::cout<<"\nPRESS A KEY TO START..."<<std::endl;
    std::cin.get();

    int id;
    std::cout <<"\nGIVE ID OF THE PIECE WILL BE DEATTACHED" << std::endl;
    std::cin >> id;
    
    ROS_INFO("**** Move Robot and attach piece ****");
    
    ros::service::waitForService("/chesslab_setup/setrobconf");
    ros::ServiceClient setrobconf_client = node.serviceClient<chesslab_setup::setrobconf>("/chesslab_setup/setrobconf");
    chesslab_setup::setrobconf setrobconf_srv;
    
    //rosservice call /chesslab_setup/setrobconf "conf: [1.02,-1.435,1.635,1.4,1.5,1.0,0.59, 0.9, -1.8, 1.7, -1.6, -1.5, 0.0, 0.]"
    setrobconf_srv.request.conf.resize(14);
    setrobconf_srv.request.conf[0]  =  1.02;
    setrobconf_srv.request.conf[1]  = -1.435;
    setrobconf_srv.request.conf[2]  =  1.635;
    setrobconf_srv.request.conf[3]  =  1.4;
    setrobconf_srv.request.conf[4]  =  1.5;
    setrobconf_srv.request.conf[5]  =  1.0;
    setrobconf_srv.request.conf[6]  =  0.59;
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
    
    //call the attach service
    ros::service::waitForService("/chesslab_setup/attachobs2robot");
    ros::ServiceClient attachobs2robot_client = node.serviceClient<chesslab_setup::attachobs2robot>("/chesslab_setup/attachobs2robot");
    chesslab_setup::attachobs2robot attachobs2robot_srv;
    
    if(id <= 201 && id >= 216){
        attachobs2robot_srv.request.robotName =  "team_A";
    }
    else{
        attachobs2robot_srv.request.robotName =  "team_B";
    }
    attachobs2robot_srv.request.objarucoid =  id;
    //The following delay is necessary to let the robot reach the final configuration before calling the attach (the exact value not analyzed)
    ros::Duration(2).sleep();
    attachobs2robot_client.call(attachobs2robot_srv);
    
    ////////////////////////////////////////////////////////
    //Wait for user to press a key
    std::cout<<"\nPRESS A KEY TO CONTINUE..."<<std::endl;
    std::cin.get();
    
    ROS_INFO("**** Move Robot with attached piece, deattaches it and moves without it****");
    
    //rosservice call /chesslab_setup/setrobconf "conf: [1.02,-0.935,0.935,1.6,1.5,1.0,0.59, 0.9, -1.8, 1.7, -1.6, -1.5, 0.0, 0.]"
    setrobconf_srv.request.conf.resize(14);
    setrobconf_srv.request.conf[0]  =  1.02;
    setrobconf_srv.request.conf[1]  = -0.935;
    setrobconf_srv.request.conf[2]  =  0.935;
    setrobconf_srv.request.conf[3]  =  1.6;
    setrobconf_srv.request.conf[4]  =  1.5;
    setrobconf_srv.request.conf[5]  =  1.0;
    setrobconf_srv.request.conf[6]  =  0.59;
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
    
    
    //call the dettach service
    ros::service::waitForService("/chesslab_setup/dettachobs");
    ros::ServiceClient dettachobs_client = node.serviceClient<chesslab_setup::dettachobs>("/chesslab_setup/dettachobs");
    chesslab_setup::dettachobs dettachobs_srv;
    
    dettachobs_srv.request.objarucoid =  216;
    if(id <= 201 && id >= 216){
        attachobs2robot_srv.request.robotName =  "team_A";
    }
    else{
        attachobs2robot_srv.request.robotName =  "team_B";
    }
    //The following delay is necessary to let the robot reach the final configuration before calling the attach (the exact value not analyzed)
    ros::Duration(2).sleep();
    dettachobs_client.call(dettachobs_srv);
    
    //move again
    //rosservice call /chesslab_setup/setrobconf "conf: [1.02,-1.55,1.535,1.4,1.5,1.0,0.59, 0.9, -1.8, 1.7, -1.6, -1.5, 0.0, 0.]"
    setrobconf_srv.request.conf.resize(14);
    setrobconf_srv.request.conf[0]  =  1.02;
    setrobconf_srv.request.conf[1]  = -1.55;
    setrobconf_srv.request.conf[2]  =  1.535;
    setrobconf_srv.request.conf[3]  =  1.4;
    setrobconf_srv.request.conf[4]  =  1.5;
    setrobconf_srv.request.conf[5]  =  1.0;
    setrobconf_srv.request.conf[6]  =  0.59;
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
}
