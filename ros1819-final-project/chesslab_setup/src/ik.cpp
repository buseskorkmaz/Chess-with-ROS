#include <iostream>
#include <ros/ros.h>
#include <iterator>
//#include <ros/package.h>
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <geometry_msgs/Pose.h>
//#include <sensor_msgs/JointState.h>

//#include <chesslab_setup/setrobconf.h>
//#include <chesslab_setup/setobjpose.h>
//#include <chesslab_setup/attachobs2robot.h>
//#include <chesslab_setup/dettachobs.h>
//#include <chesslab_setup/ffplan.h>
#include <chesslab_setup/ik.h>
#include "chesslab_setup/getobstaclepos.h"
#include <chesslab_setup/setrobconf.h>

#include "object_info.hpp"
#include <map>
#include <stdio.h> 
#include <string>

using namespace std;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "demo_ik");
    ros::NodeHandle node;
    SetChessWorld();
    ROS_INFO("**** starting chesslab_setup client node to thest the ik service****");    
    
    //call the inversekin service
    ros::service::waitForService("/chesslab_setup/inversekin");
    ros::ServiceClient inversekin_client = node.serviceClient<chesslab_setup::ik>("/chesslab_setup/inversekin");
    chesslab_setup::ik inversekin_srv;
   
    ros::service::waitForService("chesslab_setup/getobstaclepos");
    ros::ServiceClient getobstaclepos_client = node.serviceClient<chesslab_setup::getobstaclepos>("/chesslab_setup/getobstaclepos");
    chesslab_setup::getobstaclepos getobstaclepos_srv;

    ros::service::waitForService("/chesslab_setup/setrobconf");
    ros::ServiceClient setrobconf_client = node.serviceClient<chesslab_setup::setrobconf>("/chesslab_setup/setrobconf");
    chesslab_setup::setrobconf setrobconf_srv;
   
    char team;
    std::string goal;    
    int id;
    cout << "PLEASE GIVE KauthamID OF THE PIECE AND GOAL POSITION WITH CELL CODE(A8): " << std::endl;
    cin >> id >> goal;
    if (id <= 16 && id >=1)
        team = 'a';
    else
        team = 'b';
    

    getobstaclepos_srv.request.objkauthamid = id;
    getobstaclepos_client.call(getobstaclepos_srv);
    std::map<string,Cell>::iterator itmap;
    double x, y;

    for (itmap = mymap.begin(); itmap != mymap.end(); ++itmap){
        if(itmap->first == goal){
            x = itmap->second.getx();
            y = itmap->second.gety();
            break;
        }
    }
     
    inversekin_srv.request.pose.position.x = x;
    inversekin_srv.request.pose.position.y = y;
    inversekin_srv.request.pose.position.z = getobstaclepos_srv.response.pos.position.z;
    inversekin_srv.request.pose.orientation.x = getobstaclepos_srv.response.pos.orientation.x;
    inversekin_srv.request.pose.orientation.y = getobstaclepos_srv.response.pos.orientation.y;
    inversekin_srv.request.pose.orientation.z = getobstaclepos_srv.response.pos.orientation.z;
    inversekin_srv.request.pose.orientation.w = getobstaclepos_srv.response.pos.orientation.w;
    
    inversekin_client.call(inversekin_srv);
    
    ROS_INFO_STREAM("Robot Pose: [" << 
        inversekin_srv.request.pose.position.x << ", " <<
        inversekin_srv.request.pose.position.y << ", " <<
        inversekin_srv.request.pose.position.z << ", " << 
        inversekin_srv.request.pose.orientation.x << ", " << 
        inversekin_srv.request.pose.orientation.y << ", " <<
        inversekin_srv.request.pose.orientation.z << ", " <<
        inversekin_srv.request.pose.orientation.w << "]");
    
    std::stringstream sstr;
    if(inversekin_srv.response.status)
    {
        sstr<<"The computed ik is:"<<std::endl;
        for(int i=0; i<inversekin_srv.response.ik_solution.size(); i++)
        {
            sstr << "[";
            for(int j=0; j<5; j++)
            {
                sstr << inversekin_srv.response.ik_solution[i].ik[j] <<", ";            
            }
            sstr << inversekin_srv.response.ik_solution[i].ik[5] << "]" << std::endl;
        }
        ROS_INFO_STREAM(sstr.str());
        if (team == 'a'){
            setrobconf_srv.request.conf.resize(6);
            for(int j=0; j<6; j++){
                setrobconf_srv.request.conf[j] = inversekin_srv.response.ik_solution[0].ik[j];          
            }
            setrobconf_client.call(setrobconf_srv);
        }
        else{
            setrobconf_srv.request.conf.resize(6);
            for(int j=0; j<6; j++){
                setrobconf_srv.request.conf[j+6] = inversekin_srv.response.ik_solution[0].ik[j];          
            }
            setrobconf_client.call(setrobconf_srv);    
        }
    }
    else{
        ROS_INFO("Not able to compute the ik");
    }
}
