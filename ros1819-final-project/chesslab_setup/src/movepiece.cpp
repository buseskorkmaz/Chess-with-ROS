//#include <iostream>
//#include <ros/ros.h>
//#include <ros/package.h>
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <geometry_msgs/Pose.h>
//#include <sensor_msgs/JointState.h>

//#include <chesslab_setup/setrobconf.h>
#include <chesslab_setup/setobjpose.h>
#include <chesslab_setup/shareID.h>
//#include <chesslab_setup/attachobs2robot.h>
//#include <chesslab_setup/dettachobs.h>
//#include <chesslab_setup/ffplan.h>
//#include <chesslab_setup/ik.h>
//#include <std_srvs/Empty.h>
//#include "ObjectInfo.h"
#include <map> 
#include <sstream>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

#include <ff/Plan.h>
#include <ur3ik/UR3IK.h>

#include <kautham/OpenProblem.h>
#include <kautham/CheckCollision.h>
#include <kautham/SetRobotsConfig.h>

#include <kautham/ObsPos.h>
#include <kautham/AttachObstacle2RobotLink.h>
#include <kautham/DetachObstacle.h>

/*
#include <kautham/SetQuery.h>
#include <kautham/GetPath.h>
#include <kautham/ObsPos.h>
#include <kautham/SetRobotsConfig.h>
#include <kautham/RemoveObstacle.h>
#include <kautham/SetInit.h>
#include <kautham/SetRobControls.h>
#include <kautham/FindIK.h>
*/

#include "chesslab_setup/setrobconf.h"
#include "chesslab_setup/setobjpose.h"
#include "chesslab_setup/attachobs2robot.h"
#include "chesslab_setup/dettachobs.h"
#include "chesslab_setup/ffplan.h"
#include "chesslab_setup/ik.h"

#include "object_info.hpp"

using namespace std;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "demo_movepiece");
    ros::NodeHandle node;

    ROS_INFO("**** starting chesslab_setup client node to test the motion of a chess piece ****");
    
    SetChessWorld();

    //Set the initial scene
    /*
    ros::service::waitForService("/chesslab_setup/resetscene");
    ros::ServiceClient resetscene_client = node.serviceClient<std_srvs::Empty>("/chesslab_setup/resetscene");
    std_srvs::Empty resetscene_srv;
    resetscene_client.call(resetscene_srv);
    */

    ////////////////////////////////////////////////////////
    //Wait for user to press a key
    std::cout<<"\nPRESS A KEY TO START..."<<std::endl;
    std::cin.get();
    
    //Move piece
    ros::service::waitForService("/chesslab_setup/setobjpose");
    ros::ServiceClient setobjpose_client = node.serviceClient<chesslab_setup::setobjpose>("/chesslab_setup/setobjpose");
    chesslab_setup::setobjpose setobjpose_srv;
    
    //servera bilgi gönder 
    ros::service::waitForService("/chesslab_setup/shareID");
    ros::ServiceClient shareID_client = node.serviceClient<chesslab_setup::shareID>("/chesslab_setup/shareID");
    chesslab_setup::shareID shareID_srv;

    
    int id;
    string posit;
    ////////////////////////////////////////////////////////
    //Wait for user give piece and goal position
    std::cout <<"\nPlEASE GIVE INFORMATION OF THE PIECE AND DESIRED POSITION IN THE FOLLOWING FORMAT: ArucoID(210) POS(A7) ..."<<std::endl;
    std::cin >> id >> posit ;

    shareID_srv.request.objarucoid = id;
    shareID_client.call(shareID_srv);
    
    if(shareID_srv.response.found){

        Cell will_move_cell(mymap[posit].getx(), mymap[posit].gety());
        setobjpose_srv.request.objid = shareID_srv.response.objArucoID;
        setobjpose_srv.request.p.position.x = will_move_cell.getx();
        setobjpose_srv.request.p.position.y = will_move_cell.gety();
        setobjpose_srv.request.p.position.z = shareID_srv.response.pos.position.z;
        setobjpose_srv.request.p.orientation.x = shareID_srv.response.pos.orientation.x;
        setobjpose_srv.request.p.orientation.y = shareID_srv.response.pos.orientation.y;
        setobjpose_srv.request.p.orientation.z = shareID_srv.response.pos.orientation.z;
        setobjpose_srv.request.p.orientation.w = shareID_srv.response.pos.orientation.w;
    
        std::cout << "Moving piece " << setobjpose_srv.request.objid <<" to: [" << 
            setobjpose_srv.request.p.position.x << ", " <<
            setobjpose_srv.request.p.position.y << ", " <<
            setobjpose_srv.request.p.position.z << ", " << 
            setobjpose_srv.request.p.orientation.x << ", " << 
            setobjpose_srv.request.p.orientation.y << ", " <<
            setobjpose_srv.request.p.orientation.z << ", " <<
            setobjpose_srv.request.p.orientation.w << "]" << std::endl;
    
        setobjpose_client.call(setobjpose_srv);
    }
}
