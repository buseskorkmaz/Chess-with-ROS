#include <map> 
#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdlib>

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
#include "chesslab_setup/shareID.h"
#include "chesslab_setup/getobstaclepos.h"
#include "chesslab_setup/checkrule.h"
#include "chesslab_setup/checkmate.h"
#include "chesslab_setup/checkcastle.h"

#include "object_info.hpp"


using namespace std;

//! Function that wraps the call to the kautham service that opens a problem
bool kauthamOpenProblem( string modelFolder, string problemFile )
{
    ros::NodeHandle n;
    ros::service::waitForService("/kautham_node/OpenProblem");

    kautham::OpenProblem kthopenproblem_srv;
    std::string model = modelFolder;
    ros::ServiceClient kthopenproblem_client = n.serviceClient<kautham::OpenProblem>("/kautham_node/OpenProblem");
    kthopenproblem_srv.request.problem = problemFile;
    kthopenproblem_srv.request.dir.resize(1);
    kthopenproblem_srv.request.dir[0] = model;
    kthopenproblem_client.call(kthopenproblem_srv);
    if (kthopenproblem_srv.response.response == true) {
        ROS_INFO( "Kautham Problem opened correctly" );        
        std::cout <<"\nPlEASE GIVE INFORMATION OF THE PIECE AND DESIRED POSITION IN THE FOLLOWING FORMAT: ArucoID(210) POS(A7) ..."<<std::endl;

    } else {
        ROS_ERROR( "ERROR Opening Kautham Problem" );
        ROS_ERROR( "models folder: %s", kthopenproblem_srv.request.dir[0].c_str() );
        ROS_ERROR( "problem file: %s", kthopenproblem_srv.request.problem.c_str() );
    }
}




//! Function that wraps the call to the kautham service that checks for collisions
bool kauthamCheckCollisionRob(std::vector<float> conf, int *collObj, std::string *msg)
{
    ros::NodeHandle node;
    ros::service::waitForService("/kautham_node/CheckCollisionRob");
    ros::ServiceClient check_collision_obstacles_client = node.serviceClient<kautham::CheckCollision>("/kautham_node/CheckCollisionRob");

    kautham::CheckCollision check_collision_obstacles_srv;
    check_collision_obstacles_srv.request.config = conf;

    check_collision_obstacles_client.call(check_collision_obstacles_srv);


    *collObj = check_collision_obstacles_srv.response.collObj;
    *msg = check_collision_obstacles_srv.response.msg;
    
    if(!check_collision_obstacles_srv.response.collisionFree)
    {
        //std::cout<<" The current configuration is not collision free! "<<std::endl;
        return true;
    }
    else
    {
        //std::cout<<" The current configuration is collision free! "<<std::endl;
        return false;
    }
}



//! Function that wraps the call to the kautham service that moves an obstacle
bool kauthamSetObstaclePos(int index,  std::vector <float> pos)
{
    // activate the setObstaclePos service
    ros::NodeHandle node;
    ros::service::waitForService("/kautham_node/SetObstaclePos");
    ros::ServiceClient set_obstacle_client = node.serviceClient<kautham::ObsPos>("/kautham_node/SetObstaclePos");

    kautham::ObsPos set_obstacle_srv;

    set_obstacle_srv.request.index = index;
    set_obstacle_srv.request.setPos = pos;

    //call kautham service to set the obstacle pos
    set_obstacle_client.call(set_obstacle_srv);

    // Evaluate the set obstacle service and perform rest of process
    if (set_obstacle_srv.response.response == false)
    {
        ROS_ERROR("SetObstaclePos service has not been performed. ");
        return false;
    }
    else
    {
        ROS_INFO("SetObstaclePos service has been performed !");
        return true;
    }
}

//! Function that wraps the call to the kautham service that attaches an obstacle to a robot link
bool kauthamAttachObs(int robotindex, int linkindex, int obsindex)
{
    // activate the setObstaclePos service
    ros::NodeHandle node;
    ros::service::waitForService("/kautham_node/AttachObstacle2RobotLink");
    ros::ServiceClient attach_client = node.serviceClient<kautham::AttachObstacle2RobotLink>("/kautham_node/AttachObstacle2RobotLink");

    kautham::AttachObstacle2RobotLink attach_srv;

    attach_srv.request.robot = robotindex;
    attach_srv.request.link = linkindex;
    attach_srv.request.obs = obsindex;

    //call kautham service to set the robot pose
    attach_client.call(attach_srv);

    if (attach_srv.response.response == false)
    {
        ROS_ERROR("AttachObstacle2RobotLink service has not been performed. ");
        return false;
    }
    else
    {
        ROS_INFO("AttachObstacle2RobotLink service has been performed !");
        return true;
    }
}

//! Function that wraps the call to the kautham service that dettaches an obstacle from a robot 
bool kauthamDettachObs(int obsindex)
{
    // activate the setObstaclePos service
    ros::NodeHandle node;
    ros::service::waitForService("/kautham_node/DetachObstacle");
    ros::ServiceClient dettach_client = node.serviceClient<kautham::DetachObstacle>("/kautham_node/DetachObstacle");

    kautham::DetachObstacle dettach_srv;

    dettach_srv.request.obs = obsindex;

    //call kautham service to set the robot pose
    dettach_client.call(dettach_srv);

    if (dettach_srv.response.response == false)
    {
        ROS_ERROR("DettachObstacle service has not been performed. ");
        return false;
    }
    else
    {
        ROS_INFO("DettachObstacle service has been performed !");
        return true;
    }
}



//! Service that sets the robot config in rviz and checks for collision using kautham
bool setrobconf(chesslab_setup::setrobconf::Request  &req,
         chesslab_setup::setrobconf::Response &res)
{
  //set conf to visualize in rviz
  conf[0] = req.conf[0];
  conf[1] = req.conf[1];
  conf[2] = req.conf[2];
  conf[3] = req.conf[3];
  conf[4] = req.conf[4];
  conf[5] = req.conf[5];
  conf[6] = req.conf[6];
  conf[7] = req.conf[7];
  conf[8] = req.conf[8];
  conf[9] = req.conf[9];
  conf[10] = req.conf[10];
  conf[11] = req.conf[11];
  conf[12] = req.conf[12];
  conf[13] = req.conf[13];
  ROS_INFO("New robot-A configuration set to = (%f, %f, %f, %f, %f, %f, %f)", req.conf[0],req.conf[1],req.conf[2],req.conf[3],req.conf[4],req.conf[5],req.conf[6]);
  ROS_INFO("New robot-B configuration set to = (%f, %f, %f, %f, %f, %f, %f)", req.conf[7],req.conf[8],req.conf[9],req.conf[10],req.conf[11],req.conf[12],req.conf[13]);

  //move robot in kautham and collisioncheck
  //need to convert to normalized controls
  std::vector<float> controls(14);
  for(int i=0; i<6; i++){
      //<limit effort="54.0" lower="-3.141592654" upper="3.141592654" velocity="3.2"/>
      controls[i] = (conf[i]+3.141592654)/6.283185308;
  }
  //<limit effort="60" lower="0.0" upper="0.872664444444" velocity="1.91986177778"/>
  controls[6] = (conf[6]-0.0)/0.872664444444;
  for(int i=7; i<13; i++){
      controls[i] = (conf[i]+3.141592654)/6.283185308;
  }
  controls[13] = (conf[13]-0.0)/0.872664444444; 
  ROS_INFO("New robot-A controls set to = (%f, %f, %f, %f, %f, %f, %f)", controls[0],controls[1],controls[2],controls[3],controls[4],controls[5],controls[6]);
  ROS_INFO("New robot-B controls set to = (%f, %f, %f, %f, %f, %f, %f)", controls[7],controls[8],controls[9],controls[10],controls[11],controls[12],controls[13]);
  int collObj;
  std::string msg;
  if(kauthamCheckCollisionRob(controls, &collObj, &msg)){
      res.obj = kautham2aruco_map.find(collObj)->second;
      /*
      for(int i=0; i<objects.size(); i++) {
       ROS_DEBUG("object[%d].ID=%d collObj=%d",i,objects[i].getKauthamID(),collObj);
       if(objects[i].getKauthamID() == collObj){
           res.obj = objects[i].getArucoID();
           break;
       }
      }
      */
      
      //ROS_INFO("Collision with object %d",res.obj);
      res.msg = msg;
      res.incollision = true;
      ROS_INFO_STREAM(res.msg);
  }
  else{
      res.obj = -1;
      //ROS_INFO("Collision free configuration");
      res.incollision = false;
      res.msg = msg;
      ROS_INFO_STREAM(res.msg);
  }
  return true;
}


//! Service that sets the pose of an object in rviz and in kautham
bool setobjpose(chesslab_setup::setobjpose::Request  &req,
         chesslab_setup::setobjpose::Response &res)
{
  int i = aruco2rviz_map.find(req.objid)->second;
  objects[i].setx(req.p.position.x);
  objects[i].sety(req.p.position.y);
  objects[i].setz(req.p.position.z);
  objects[i].setqx(req.p.orientation.x);
  objects[i].setqy(req.p.orientation.y);
  objects[i].setqz(req.p.orientation.z);
  objects[i].setqw(req.p.orientation.w);
    
  /*
  int i;
  for(i=0; i<objects.size(); i++) {
   //ROS_DEBUG("object[%d].ID=%d req.objid=%d",i,objects[i].getArucoID(),req.objid);
   if(objects[i].getArucoID() == req.objid){
     objects[i].setx(req.p.position.x);
     objects[i].sety(req.p.position.y);
     objects[i].setz(req.p.position.z);
     objects[i].setqx(req.p.orientation.x);
     objects[i].setqy(req.p.orientation.y);
     objects[i].setqz(req.p.orientation.z);
     objects[i].setqw(req.p.orientation.w);
     break;
   }
  }
  */
  ROS_INFO("Object[%d] with id %d and kautham id %d set to pose = (%f, %f, %f, %f, %f, %f, %f)", i, req.objid, objects[i].getKauthamID(), req.p.position.x, req.p.position.y, req.p.position.z, req.p.orientation.x, req.p.orientation.y, req.p.orientation.z, req.p.orientation.w );

  //move object in kautham
  int collObj;
  std::string msg;
  int index = aruco2kautham_map.find(req.objid)->second; //objects[i].getKauthamID();
  std::vector <float> pos(7);
  pos[0] = req.p.position.x;
  pos[1] = req.p.position.y;
  pos[2] = req.p.position.z;
  pos[3] = req.p.orientation.x;
  pos[4] = req.p.orientation.y;
  pos[5] = req.p.orientation.z;
  pos[6] = req.p.orientation.w;

  kauthamSetObstaclePos(index, pos);
  return true;
}



//! Service that attachs an obstacle to the gripper_right pad. 
bool attachobs2robot(chesslab_setup::attachobs2robot::Request  &req,
         chesslab_setup::attachobs2robot::Response &res)
{
  ROS_INFO("Attach obstacle with aruco mark %d to gripper right pad of robot %s", req.objarucoid, req.robotName.c_str());

  //Sets the transform that attaches the object to the robot gripper in rviz
  //it sets the frame of team_A__gripper_right_pad (or  team_B__gripper_right_pad) as the reference frame for the object
  //then computes the from this reference frame to the object reference frame
  
  //transform from chessboard to object
  int i = aruco2rviz_map.find(req.objarucoid)->second;
  tf2::Transform chess2obj;
  tf2::fromMsg(objects[i].getPose(), chess2obj);
    
  //transform from chessboard to robot gripper pad
  geometry_msgs::TransformStamped transformStamped;
  std::string rightpadframe = req.robotName+"_gripper_right_pad"; //team_A__gripper_right_pad or team_B_gripper_right_pad
  ROS_INFO("frame name = %s",rightpadframe.c_str());
  try{
      transformStamped = tfBuffer.lookupTransform("chess_frame", rightpadframe, ros::Time(0), ros::Duration(0.1));
  }
  catch (tf2::TransformException ex ){
    ROS_ERROR("%s",ex.what());
  }
  tf2::Stamped< tf2::Transform >  chess2pad;
  tf2::fromMsg(transformStamped, chess2pad);
  //std::cout<<"chess2pad!!!!!!! = "<<chess2pad.frame_id_<<std::endl;
  
  //transform from robot gripper pad to object
  tf2::Transform pad2chess = chess2pad.inverse();
  tf2::Transform pad2obj = pad2chess * chess2obj;
  
  //set the transform from right pad frame to oject
  objects[i].set_frame_id(rightpadframe);
  objects[i].setx(pad2obj.getOrigin().getX());
  objects[i].sety(pad2obj.getOrigin().getY());
  objects[i].setz(pad2obj.getOrigin().getZ());
  objects[i].setqx(pad2obj.getRotation().getX());
  objects[i].setqy(pad2obj.getRotation().getY());
  objects[i].setqz(pad2obj.getRotation().getZ());
  objects[i].setqw(pad2obj.getRotation().getW());
 
  /*
  std::cout<<"chess2pad = "<<std::endl;
  std::cout<<" x = "<<chess2pad.getOrigin().getX()<< std::endl;
  std::cout<<" y = "<<chess2pad.getOrigin().getY()<< std::endl;
  std::cout<<" z = "<<chess2pad.getOrigin().getZ()<< std::endl;
  std::cout<<" qx = "<<chess2pad.getRotation().getX()<< std::endl;
  std::cout<<" qy = "<<chess2pad.getRotation().getY()<< std::endl;
  std::cout<<" qz = "<<chess2pad.getRotation().getZ()<< std::endl;
  std::cout<<" qw = "<<chess2pad.getRotation().getW()<< std::endl;
  std::cout<<"pad2chess = "<<std::endl;
  std::cout<<" x = "<<pad2chess.getOrigin().getX()<< std::endl;
  std::cout<<" y = "<<pad2chess.getOrigin().getY()<< std::endl;
  std::cout<<" z = "<<pad2chess.getOrigin().getZ()<< std::endl;
  std::cout<<" qx = "<<pad2chess.getRotation().getX()<< std::endl;
  std::cout<<" qy = "<<pad2chess.getRotation().getY()<< std::endl;
  std::cout<<" qz = "<<pad2chess.getRotation().getZ()<< std::endl;
  std::cout<<" qw = "<<pad2chess.getRotation().getW()<< std::endl;
  std::cout<<"chess2obj = "<<std::endl;
  std::cout<<" x = "<<chess2obj.getOrigin().getX()<< std::endl;
  std::cout<<" y = "<<chess2obj.getOrigin().getY()<< std::endl;
  std::cout<<" z = "<<chess2obj.getOrigin().getZ()<< std::endl;
  std::cout<<" qx = "<<chess2obj.getRotation().getX()<< std::endl;
  std::cout<<" qy = "<<chess2obj.getRotation().getY()<< std::endl;
  std::cout<<" qz = "<<chess2obj.getRotation().getZ()<< std::endl;
  std::cout<<" qw = "<<chess2obj.getRotation().getW()<< std::endl;
  
  std::cout<<"pad2obj = "<<std::endl;
  std::cout<<" x = "<<pad2obj.getOrigin().getX()<< std::endl;
  std::cout<<" y = "<<pad2obj.getOrigin().getY()<< std::endl;
  std::cout<<" z = "<<pad2obj.getOrigin().getZ()<< std::endl;
  std::cout<<" qx = "<<pad2obj.getRotation().getX()<< std::endl;
  std::cout<<" qy = "<<pad2obj.getRotation().getY()<< std::endl;
  std::cout<<" qz = "<<pad2obj.getRotation().getZ()<< std::endl;
  std::cout<<" qw = "<<pad2obj.getRotation().getW()<< std::endl;
  */
  
  //attaches the object in kautham
  int robotindex;
  if(req.robotName=="team_A") robotindex = 0;
  else robotindex = 1;
  //std::cout<<"Robot index = "<<robotindex<<std::endl;
  int linkindex = 14; //this is the kautham index corresponding to the *gripper_right_pad*
  int obsindex = aruco2kautham_map.find(req.objarucoid)->second; //objects[i].getKauthamID();
  kauthamAttachObs(robotindex, linkindex, obsindex);
  
  //sets global flag on attached obstactes
  attached[robotindex] = req.objarucoid;
  
  
  std::cout<<"attached[0]="<<attached[0]<<std::endl;
  std::cout<<"attached[1]="<<attached[1]<<std::endl;
  
  return true;
}



//! Service that dettaches an obstacle from a given robot. 
bool dettachobs(chesslab_setup::dettachobs::Request  &req,
         chesslab_setup::dettachobs::Response &res)
{
  ROS_INFO("Dettach obstacle with aruco mark %d from robot %s", req.objarucoid, req.robotName.c_str());

  //Resets the transform that locates the object w.r.t. the chess frame 
  
  //transform from robot gripper pad to object (the object is attached so the object pose is w.r.t the gripper pad frame)
  int i = aruco2rviz_map.find(req.objarucoid)->second;
  tf2::Transform pad2obj;
  tf2::fromMsg(objects[i].getPose(), pad2obj);
    
  //transform from chessboard to robot gripper pad
  geometry_msgs::TransformStamped transformStamped;
  std::string rightpadframe = req.robotName+"_gripper_right_pad"; //team_A__gripper_right_pad or team_B_gripper_right_pad
  ROS_INFO("frame name = %s",rightpadframe.c_str());
  try{
      transformStamped = tfBuffer.lookupTransform("chess_frame", rightpadframe, ros::Time(0), ros::Duration(0.1));
  }
  catch (tf2::TransformException ex ){
    ROS_ERROR("%s",ex.what());
  }
  tf2::Stamped< tf2::Transform >  chess2pad;
  tf2::fromMsg(transformStamped, chess2pad);
  
  //transform from chessboard to object
  tf2::Transform chess2obj = chess2pad * pad2obj;
  
  //set the transform from right pad frame to oject
  objects[i].set_frame_id("/chess_frame");
  objects[i].setx(chess2obj.getOrigin().getX());
  objects[i].sety(chess2obj.getOrigin().getY());
  objects[i].setz(chess2obj.getOrigin().getZ());
  objects[i].setqx(chess2obj.getRotation().getX());
  objects[i].setqy(chess2obj.getRotation().getY());
  objects[i].setqz(chess2obj.getRotation().getZ());
  objects[i].setqw(chess2obj.getRotation().getW());
 
  //dettaches the object in kautham
  int obsindex = aruco2kautham_map.find(req.objarucoid)->second; 
  kauthamDettachObs(obsindex);
  
  
  //resets global flag on attached obstactes
  std::cout<<"Current attached vector"<<std::endl;
  std::cout<<"attached[0]="<<attached[0]<<std::endl;
  std::cout<<"attached[1]="<<attached[1]<<std::endl;
  int robotindex;
  if(req.robotName=="team_A") robotindex = 0;
  else robotindex = 1;
  attached[robotindex] = -1;
  std::cout<<"resetting attached vector"<<std::endl;
  std::cout<<"attached[0]="<<attached[0]<<std::endl;
  std::cout<<"attached[1]="<<attached[1]<<std::endl;
  
  return true;
}




//! Function that wraps the call to the ur ik service 
bool findIK(std::vector<std::vector<double>> &iksolution, geometry_msgs::Pose ur3pose)
{
    ros::NodeHandle node;
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
    ur3ik_srv.request.pose = ur3pose;
        
    
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
        iksolution.resize(ur3ik_srv.response.ik_solution.size());
        sstr<<"The computed ik is:"<<std::endl;
        for(int i=0; i<ur3ik_srv.response.ik_solution.size(); i++)
        {
            for(int j=0; j<6; j++)
                iksolution[i].push_back(ur3ik_srv.response.ik_solution[i].ik[j]);
            
            sstr << "[";
            for(int j=0; j<5; j++)
            {
                sstr << ur3ik_srv.response.ik_solution[i].ik[j] <<", ";            
            }
            sstr << ur3ik_srv.response.ik_solution[i].ik[5] << "]" << std::endl;
        }
        ROS_INFO_STREAM(sstr.str());
        return true;
    }
    else{
        ROS_INFO("Not able to compute the ik");
        return false;
    }
}


//! Service that calls the ik for the ur3 robot
bool inversekin(chesslab_setup::ik::Request  &req,
         chesslab_setup::ik::Response &res)
{
    std::vector<std::vector<double>> iksolution;
    if(findIK(iksolution, req.pose))
    {
        res.ik_solution.resize(iksolution.size());
        for(int i=0; i<iksolution.size(); i++)
            for(int j=0; j<6; j++)
                res.ik_solution[i].ik.push_back(iksolution[i][j]); 
        res.status = true;
    }
    else
    {
        res.status = false;
    }
}


//! Function that wraps the call to the FF service that computes a plan
bool taskPlan(std::vector <std::string> &plan, 
              std::vector< std::pair<std::string,std::string>> &init, 
              std::vector< std::pair<std::string,std::string>> &goal, 
              std::vector< std::string > &initfree,
              std::vector< std::string > &goalfree,
              std::string deadpiece="")
{
    ros::NodeHandle node;
    ros::service::waitForService("/FFPlan");
    ros::ServiceClient ff_client = node.serviceClient<ff::Plan>("/FFPlan");
    ff::Plan ff_srv;
    
    //Writing the problem file
    std::stringstream sstr_comment;
    std::stringstream sstr_intro;
    std::stringstream sstr_init;
    std::stringstream sstr_goal;
    
    sstr_comment << "; This is an automated generated file.\n\
    ; put in .gitignore not to be under version control\n";
    
    //Introductory part needed for all chess world problems
    sstr_intro << "(define (problem chessworld)\n\
        (:domain chessworld)\n\
        \n\
        (:objects \n\
            BLACK_PAWN_1 BLACK_PAWN_2 BLACK_PAWN_3 BLACK_PAWN_4 BLACK_PAWN_5 BLACK_PAWN_6 BLACK_PAWN_7 BLACK_PAWN_8 \n\
            WHITE_PAWN_1 WHITE_PAWN_2 WHITE_PAWN_3 WHITE_PAWN_4 WHITE_PAWN_5 WHITE_PAWN_6 WHITE_PAWN_7 WHITE_PAWN_8 \n\
            BLACK_KNIGHT_1 BLACK_KNIGHT_2 BLACK_HORSE_1 BLACK_HORSE_2 BLACK_TOWER_1 BLACK_TOWER_2 \n\
            WHITE_KNIGHT_1 WHITE_KNIGHT_2 WHITE_HORSE_1 WHITE_HORSE_2 WHITE_TOWER_1 WHITE_TOWER_2 \n\
            BLACK_KING BLACK_QUEEN \n\
            WHITE_KING WHITE_QUEEN \n\
            A1 A2 A3 A4 A5 A6 A7 A8  \n\
            B1 B2 B3 B4 B5 B6 B7 B8  \n\
            C1 C2 C3 C4 C5 C6 C7 C8  \n\
            D1 D2 D3 D4 D5 D6 D7 D8  \n\
            E1 E2 E3 E4 E5 E6 E7 E8  \n\
            F1 F2 F3 F4 F5 F6 F7 F8  \n\
            G1 G2 G3 G4 G5 G6 G7 G8  \n\
            H1 H2 H3 H4 H5 H6 H7 H8  \n\
            SAFE_REGION \n\
         )\n";
    
     //Setting the init state
     std::stringstream sstr_initmap;
     for(int i=0; i<init.size() ;i++)
     {
        sstr_initmap << "(on "+ init[i].first + " " +  init[i].second + ") \n";
     }
     std::stringstream sstr_initfree;
     for(int i=0; i<initfree.size() ;i++)
     {
        sstr_initfree << "(freecell "+ initfree[i] + ") \n";
     }
     sstr_init << "(:init \n\
        (arm-empty) \n\
        (freecell SAFE_REGION) \n" << sstr_initfree.str() << "\n" << sstr_initmap.str() << ")\n";
     ROS_INFO_STREAM("Init set to " << sstr_init.str() << "\n freecells:" << sstr_initfree.str());
        
        
    //Setting the goal state
    std::stringstream sstr_goalmap;
    for(int i=0; i<goal.size() ;i++)
    {
        sstr_goalmap << "(on "+ goal[i].first + " " + goal[i].second +") \n";
    }
    std::stringstream sstr_goalfree;
    for(int i=0; i<goalfree.size() ;i++)
    {
       sstr_goalfree << "(freecell "+ goalfree[i] + ") \n";
    }

    sstr_goal << "(:goal \n\
        (and " << sstr_goalmap.str() << "\n" << sstr_goalfree.str();
        
    if(deadpiece!="") 
        sstr_goal << " (deadpiece " << deadpiece << ")))\n";
    else
        sstr_goal <<  "))\n";
    
     ROS_INFO_STREAM("Goal set to " << sstr_goal.str());
    
    //Writing it to a file
    std::string ff_path = ros::package::getPath("ff");
    std::string problemFile = ff_path + "/ff-domains/my_chess_world_problem";
    std::ofstream myfile;
    myfile.open (problemFile, std::ofstream::out);
    myfile << sstr_comment.str();
    myfile << sstr_intro.str();
    myfile << sstr_init.str();
    myfile << sstr_goal.str();
    myfile << ")"; //closing the problem
    myfile.close();
    
    //Loading the requestfor the ff service: the problem file and the domain file
    //std::string problemFile = ff_path + "/ff-domains/chess_world_2";
    std::string domainFile = ff_path + "/ff-domains/chess_world.pddl";
      
    ff_srv.request.problem = problemFile;
    ff_srv.request.domain = domainFile;
        
    ROS_INFO_STREAM("Domain: " << domainFile);
    ROS_INFO_STREAM("Problem: " << problemFile);
    
    //Calling the service
    ff_client.call(ff_srv);
    
    //printing the response
    std::stringstream sstr;
    if(ff_srv.response.response)
    {
        sstr<<"The computed ff plan is:"<<std::endl;
        for(int i=0; i<ff_srv.response.plan.size(); i++)
        {
            sstr << ff_srv.response.plan[i] <<std::endl;
            plan.push_back(ff_srv.response.plan[i]);
        }
        ROS_INFO_STREAM(sstr.str());
        return true;
    }
    else{
        ROS_INFO("Not able to compute ff plan");
        return false;
    }
}


//! Service that plans a movement using the FF task planner 
bool planmovement(chesslab_setup::ffplan::Request  &req,
         chesslab_setup::ffplan::Response &res)
{
  ROS_INFO("Planning movements using FF");

  std::vector <std::string> ffplan;
  std::vector< std::pair<std::string,std::string> > init; //stores piecelabel and cell where it is located for the initial state
  std::vector< std::pair<std::string,std::string> > goal; //stores piecelabel and cell where it is located for the initial state
  std::string deadpiece;
  std::vector< std::string > initfree; //stores labels of free cells in init state
  std::vector< std::string > goalfree; //stores labels of free cells in goal state
  
  for(int i=0; i<req.init.objarucoid.size();i++)
    init.push_back( std::pair<std::string,std::string>(aruco2ff_map.find(req.init.objarucoid[i])->second, req.init.occupiedcells[i]));
     
  for(int i=0; i<req.goal.objarucoid.size();i++)
    goal.push_back( std::pair<std::string,std::string>(aruco2ff_map.find(req.goal.objarucoid[i])->second, req.goal.occupiedcells[i]));

  for(int i=0; i<req.init.freecells.size();i++)
    initfree.push_back( req.init.freecells[i]) ;
  
  for(int i=0; i<req.goal.freecells.size();i++)
    goalfree.push_back( req.goal.freecells[i]) ;

  deadpiece = aruco2ff_map.find(req.killed)->second;
  
  for(int i=0; i<init.size();i++)
      std::cout<<"---- "<<init[i].first<<" "<<init[i].second<<std::endl;  
  for(int i=0; i<goal.size();i++)
      std::cout<<"---- "<<goal[i].first<<" "<<goal[i].second<<std::endl;
  
  
  if(taskPlan(ffplan, init, goal, initfree, goalfree, deadpiece))
  {
      res.plan = ffplan;
      res.response = true;
  }
  else {
      res.response = false;
  }
  return true;
}

//! Service that sets the initial set-up
bool resetscene(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
  ros::NodeHandle node;
  ROS_INFO("Setting the initial scene");
  
  //dettaches the object in kautham, if any was attached
  std::cout<<"attached[0]="<<attached[0]<<std::endl;
  std::cout<<"attached[1]="<<attached[1]<<std::endl;
  for(int i=0; i<2; i++)
  {
    if(attached[i]!=-1)
    {
        kauthamDettachObs(aruco2kautham_map.find(attached[i])->second);
        attached[i] = -1;
    }
  }
  
  
  //Reset the pieces locations - this must be done after dettaching
  SetChessWorld();
  //Set pieces in Kautham
  int index;
  std::vector <float> pos(7);
  for(int i=0; i<objects.size(); i++)
  {
        index = objects[i].getKauthamID(); 
        pos[0] = objects[i].getPose().position.x;
        pos[1] = objects[i].getPose().position.y;
        pos[2] = objects[i].getPose().position.z;
        pos[3] = objects[i].getPose().orientation.x;
        pos[4] = objects[i].getPose().orientation.y;
        pos[5] = objects[i].getPose().orientation.z;
        pos[6] = objects[i].getPose().orientation.w;
        kauthamSetObstaclePos(index, pos);
  }

  
  //Reset the robots in rviz
  for(int i=0; i<14; i++)
    conf[i]  =  0.0;
  
  //Reset the robots in Kautham
  //need to convert to normalized controls
  std::vector<float> controls(14);
  for(int i=0; i<6; i++){
      //<limit effort="54.0" lower="-3.141592654" upper="3.141592654" velocity="3.2"/>
      controls[i] = (conf[i]+3.141592654)/6.283185308;
  }
  //<limit effort="60" lower="0.0" upper="0.872664444444" velocity="1.91986177778"/>
  controls[6] = (conf[6]-0.0)/0.872664444444;
  for(int i=7; i<13; i++){
      controls[i] = (conf[i]+3.141592654)/6.283185308;
  }
  controls[13] = (conf[13]-0.0)/0.872664444444; 
  int collObj;
  std::string msg;
  kauthamCheckCollisionRob(controls, &collObj, &msg);
  ROS_INFO_STREAM(msg);   
  
  return true;
}

bool shareID(chesslab_setup::shareID::Request  &req, chesslab_setup::shareID::Response &res){
    bool found = false;
    vector<ObjectInfo>::iterator it;  // declare an iterator to a vector of strings
    // start at from the beginning and keep iterating over the vector until you find the given piece
        
    if(!req.posit.empty()){
        std::map<string,Cell>::iterator itmap;
        double x, y;
        res.free = true;
        for (itmap = mymap.begin(); itmap != mymap.end(); ++itmap){
            if(itmap->first == req.posit){
                x = itmap->second.getx();
                y = itmap->second.gety();
                found = true;
                break;
            }
        }
        if (!found) 
            std::cout << "\nPLEASE GIVE A VALID POSITION" << std::endl;
        else{
            for(it = objects.begin(); it != objects.end(); ++it){
                if((it->getx() == x) && (it->gety() == y)){
                    res.objArucoID = it->getArucoID();
                    res.free = false;
                    break;
                }
            }
        }            
    }     
    else{  
        int i = req.objarucoid;
        geometry_msgs::Pose pos;
        for(it = objects.begin(); it != objects.end(); it++){
            if(it->getArucoID() == i) {
                found = true;
                break;
            }
        }
        if (!found) 
            std::cout << "\nPLEASE GIVE A VALID PIECE" << std::endl;
        else{
            res.objkauthamid = it->getKauthamID();
            res.objArucoID = it->getArucoID();
            res.pos.position.x = it->getx();
            res.pos.position.y = it->gety();
            res.pos.position.z = it->getz();
            res.pos.orientation.x = it->getqx();
            res.pos.orientation.y = it->getqy();
            res.pos.orientation.z = it->getqz();
            res.pos.orientation.w = it->getqw();
        //gereksiz olabilir
        }
    }
    res.found = found;
    return found;
}

bool getobstaclepos(chesslab_setup::getobstaclepos::Request &req, chesslab_setup::getobstaclepos::Response &res){
    int i = req.objkauthamid;
    
    std::vector<ObjectInfo>::iterator it;
    bool found = false;
    
    for(it = objects.begin(); it != objects.end(); it++){
        if(it->getKauthamID() == i) {
            found = true;
            break;
        }
    }
    if (!found) 
        std::cout << "\nPLEASE GIVE A VALID PIECE" << std::endl;
    else{
        ObjectInfo will_move_obj(*it);
        res.pos = will_move_obj.getPose();
    }
    return found;
}

bool checkRule(chesslab_setup::checkrule::Request &req, chesslab_setup::checkrule::Response &res){
    int id;
    std::string initial_pos = req.initial_pos;
    std::string goal_pos = req.goal_pos;
    std::map<string,Cell>::iterator itmap;
    char goal_cell_letter;
    char goal_cell_number;
    char initial_cell_letter;
    char initial_cell_number;
    int len = 2;
    bool check = false;

    id = req.id;
    initial_pos = req.initial_pos;
    goal_pos = req.goal_pos;

    goal_cell_letter = goal_pos[0];
    goal_cell_number = goal_pos[1];

    initial_cell_letter = initial_pos[0];
    initial_cell_number = initial_pos[1];
   

    //PİYONLAR BASTA 2 KUTU DA GIDEBİLİR

    if (id  >= 201 && id <= 208){ //BLACK PAWNS
        if((initial_cell_number - goal_cell_number == 1) && ((abs((int)initial_cell_letter - int(goal_cell_letter) == -1)) || abs(((int)initial_cell_letter - (int)goal_cell_letter) == 0))){
            res.check = true;
        }
    } 
    else if (id >= 301 && id <= 308){//WHITE PAWNS
        if((initial_cell_number - goal_cell_number == -1) && ((abs((int)initial_cell_letter - int(goal_cell_letter)) == 1) || (int)initial_cell_letter - (int)goal_cell_letter == 0)){
            res.check = true;
        }
    }
    else if((id == 209) || (id == 210) || (id == 309) || (id == 310)){ //TOWERS
        if((initial_cell_number - goal_cell_number == 0) || ((int)initial_cell_letter - (int)goal_cell_letter == 0)){
            res.check = true;
            if(id == 209 || id == 210)
                castleoperation.seta(false);
            else
                castleoperation.setb(false);
        }
    }
    else if((id == 213) || (id == 214) || (id == 313) || (id == 314)){ //KNIGHTS
        if((initial_cell_number - goal_cell_number != 0 )&& ((int)initial_cell_letter - (int)goal_cell_letter != 0))
            res.check = true;
    }
    else if(id == 215 || id == 315)//QUEENS
        res.check = true;
    else if(id == 316 || id == 216){//KINGS
        if((abs(initial_cell_number- goal_cell_number) == 1) || (abs((int)initial_cell_letter - (int)goal_cell_letter == 1)))
            res.check = true;
    }
    else if((id == 211) || (id == 212) || (id == 311) || (id == 312)){//HORSES
        if(((abs(initial_cell_number - goal_cell_number) == 2) && (abs((int)initial_cell_letter - (int)goal_cell_letter) == 1)) ||
            ((abs(initial_cell_number - goal_cell_number) == 1) && (abs((int)initial_cell_letter - (int)goal_cell_letter) == 2)))
            res.check = true;
    }
    else
        std::cout << "\nPLEASE GIVE A VALID PIECE" << std::endl;
    if(!res.check)
        std::cout << "\nTHE MOVEMENT IS NOT ALLOWED!" << std::endl;
}

bool diagonal_check(double x, double y, char team){
    std::vector<ObjectInfo>::iterator it;
    bool kill = false;
    double v = x; 
    double z = y;
    if(team == 'A'){
        while( v <= 0.175 && z <= 0.175 && !kill ){
            for(it = objects.begin(); it != objects.end(); ++it){
                if(it->getx() == x && it->gety() == y){
                    if(it->getArucoID() >= 201 && it->getArucoID() <= 216)
                        return false;
                    else if(((it->getArucoID() >= 301 && it->getArucoID() <= 308) || it->getArucoID() == 316) && abs(x-v) == 0.050 && abs(y-z) == 0.050)
                        kill = true;
                    else if(it->getArucoID() == 315 || it->getArucoID() == 313 || it->getArucoID() == 314)
                        kill = true;
                    else
                        kill = false;
                }
                else
                    kill = false;
            }
            v += 0.050;
            z += 0.050;
        }
        while( v >= -0.175 && z >= -0.175 && !kill ){
            for(it = objects.begin(); it != objects.end(); ++it){
                if(it->getx() == x && it->gety() == y){
                    if(it->getArucoID() >= 201 && it->getArucoID() <= 216)
                        return false;
                    else if(((it->getArucoID() >= 301 && it->getArucoID() <= 308) || it->getArucoID ()== 316) && abs(x-v) == 0.050 && abs(y-z) == 0.050)
                        kill = true;
                    else if(it->getArucoID() == 315 || it->getArucoID() == 313 || it->getArucoID() == 314)
                        kill = true;
                    else
                        kill = false;
                }
                else
                    kill = false;
            }
            v -= 0.050;
            z -= 0.050;
        }
    }
    else if(team == 'B'){
        while( v <= 0.175 && z <= 0.175 && !kill ){
            for(it = objects.begin(); it != objects.end(); ++it){
                if(it->getx() == x && it->gety() == y){
                    if(it->getArucoID() >= 301 && it->getArucoID() <= 316)
                        return false;
                    else if(((it->getArucoID() >= 201 && it->getArucoID() <= 208) || it->getArucoID() == 216) && abs(x-v) == 0.050 && abs(y-z) == 0.050)
                        kill = true;
                    else if(it->getArucoID() == 215 || it->getArucoID() == 213 || it->getArucoID() == 214)
                        kill = true;
                    else
                        kill = false;
                }
                else
                    kill = false;
            }
            v += 0.050;
            z += 0.050;
        }
        while( v >= -0.175 && z >= -0.175 && !kill ){
            for(it = objects.begin(); it != objects.end(); ++it){
                if(it->getx() == x && it->gety() == y){
                    if(it->getArucoID() >= 301 && it->getArucoID() <= 316)
                        return false;
                    else if(((it->getArucoID() >= 201 && it->getArucoID() <= 208) || it->getArucoID() == 216) && abs(x-v) == 0.050 && abs(y-z) == 0.050)
                        kill = true;
                    else if(it->getArucoID() == 215 || it->getArucoID() == 213 || it->getArucoID() == 214)
                        kill = true;
                    else
                        kill = false;
                }
                else
                    kill = false;
            }
            v -= 0.050;
            z -= 0.050;
        }
    }
    else
        std::cout << "\nPLEASE INDICATE YOUR TEAM CORRECTLY" << std::endl;
    return kill;
    
}

bool horizontal_check(double x, double y, char team){
    std::vector<ObjectInfo>::iterator it;
    bool kill = false;
    double j = y;
    if(team == 'A'){
        while(j <= 0.175 && !kill ){
            for(it = objects.begin(); it != objects.end(); ++it){
                if(it->getx() == x && it->gety() == j && it->getArucoID() >= 201 && it->getArucoID() <= 216)
                    return false;
                else if(it->getx() == x && it->gety() == j && (abs(j-y) == 0.050) && it->getArucoID() == 316)
                    kill = true;
                else if(it->getx() == x && it->gety() == j && (it->getArucoID() == 309 || it->getArucoID() == 310 || it->getArucoID() == 315))
                    kill = true;
                else
                    kill = false;
            }
            j += 0.050;
        }
        while(j >= -0.175 && !kill ){
            for(it = objects.begin(); it != objects.end(); ++it){
                if(it->getx() == x && it->gety() == j && it->getArucoID() >= 201 && it->getArucoID() <= 216)
                    return false;
                else if(it->getx() == x && it->gety() == j && (abs(j-y) == 0.050) && it->getArucoID() == 316)
                    kill = true;
                else if(it->getx() == x && it->gety() == j && (it->getArucoID() == 309 || it->getArucoID() == 310 || it->getArucoID() == 315))
                    kill = true;
                else
                    kill = false;
            }
            j -= 0.050;
        }
        //checking horses
        for (it= objects.begin(); it != objects.end(); ++it){
            if((it->getArucoID() == 311 || it->getArucoID() == 312) && ((abs(x-it->getx() == 0.100) && abs(y-it->gety()) == 0.050)) || (abs(x-it->getx() == 0.100) && abs(y-it->gety() == 0.050)))
                kill = true;
        }
    }
   else if(team == 'B'){
        while(j <= 0.175 && !kill ){
            for(it = objects.begin(); it != objects.end(); ++it){
                if(it->getx() == x && it->gety() == j && it->getArucoID() >= 301 && it->getArucoID() <= 316)
                    return false;
                else if(it->getx() == x && it->gety() == j && (abs(j-y) == 0.050) && it->getArucoID() == 216)
                    kill = true;
                else if(it->getx() == x && it->gety() == j && (it->getArucoID() == 209 || it->getArucoID() == 210 || it->getArucoID() == 215))
                    kill = true;
                else
                    kill = false;
            }
            j += 0.050;
        }
        while(j >= -0.175 && !kill ){
            for(it = objects.begin(); it != objects.end(); ++it){
                if(it->getx() == x && it->gety() == j && it->getArucoID() >= 301 && it->getArucoID() <= 316)
                    return false;
                else if(it->getx() == x && it->gety() == j && (abs(j-y) == 0.050) && it->getArucoID() == 216)
                    kill = true;
                else if(it->getx() == x && it->gety() == j && (it->getArucoID() == 209 || it->getArucoID() == 210 || it->getArucoID() == 215))
                    kill = true;
                else
                    kill = false;
            }
            j -= 0.050;
        }
        for (it= objects.begin(); it != objects.end(); ++it){
            if((it->getArucoID() == 211 || it->getArucoID() == 212) && ((abs(x-it->getx() == 0.100) && abs(y-it->gety()) == 0.050)) || (abs(x-it->getx() == 0.100) && abs(y-it->gety() == 0.050)))
                kill = true;
        }
    }
    else
        std::cout << "\nPLEASE INDICATE YOUR TEAM CORRECTLY" << std::endl;
    return kill;
}

bool vertical_check(double x, double y, char team){
    std::vector<ObjectInfo>::iterator it;
    bool kill = false;
    double j = x;
    if(team == 'A'){
        while(j <= 0.175 && !kill ){
            for(it = objects.begin(); it != objects.end(); ++it){
                if(it->getx() == j && it->gety() == y && it->getArucoID() >= 201 && it->getArucoID() <= 216)
                    return false;
                else if(it->getx() == j && it->gety() == y && (abs(j-y) == 0.050) && it->getArucoID() == 316)
                    kill = true;
                else if(it->getx() == j && it->gety() == y && (it->getArucoID() == 309 || it->getArucoID() == 310 || it->getArucoID() == 315))
                    kill = true;
                else
                    kill = false;
            }
            j += 0.050;
        }
        while(j >= -0.175 && !kill ){
            for(it = objects.begin(); it != objects.end(); ++it){
                if(it->getx() == j && it->gety() == y && it->getArucoID() >= 201 && it->getArucoID() <= 216)
                    return false;
                else if(it->getx() == j && it->gety() == y && (abs(j-y) == 0.050) && it->getArucoID() == 316)
                    kill = true;
                else if(it->getx() == j && it->gety() == y && (it->getArucoID() == 309 || it->getArucoID() == 310 || it->getArucoID() == 315))
                    kill = true;
                else
                    kill = false;
            }
            j -= 0.050;
        }
    }
   else if(team == 'B'){
        while(j <= 0.175 && !kill ){
            for(it = objects.begin(); it != objects.end(); ++it){
                if(it->getx() == j && it->gety() == y && it->getArucoID() >= 301 && it->getArucoID() <= 316)
                    return false;
                else if(it->getx() == j && it->gety() == y && (abs(j-y) == 0.050) && it->getArucoID() == 216)
                    kill = true;
                else if(it->getx() == j && it->gety() == y && (it->getArucoID() == 209 || it->getArucoID() == 210 || it->getArucoID() == 215))
                    kill = true;
                else
                    kill = false;
            }
            j += 0.050;
        }
        while(j >= -0.175 && !kill ){
            for(it = objects.begin(); it != objects.end(); ++it){
                if(it->getx() == j && it->gety() == y && it->getArucoID() >= 301 && it->getArucoID() <= 316)
                    return false;
                else if(it->getx() == j && it->gety() == y && (abs(j-y) == 0.050) && it->getArucoID() == 216)
                    kill = true;
                else if(it->getx() == j && it->gety() == y && (it->getArucoID() == 209 || it->getArucoID() == 210 || it->getArucoID() == 215))
                    kill = true;
                else
                    kill = false;
            }
            j -= 0.050;
        }
    }
    else
        std::cout << "\nPLEASE INDICATE YOUR TEAM CORRECTLY" << std::endl;
    
    return kill;
}

bool checkmate(chesslab_setup::checkmate::Request &req, chesslab_setup::checkmate::Response &res){
    int king_id = req.id;
    char team;
    double x, y;
    std::vector<ObjectInfo>::iterator it;
    for(it = objects.begin(); it != objects.end(); ++it){
        if(it->getArucoID() == king_id){
            x = it->getx();
            y = it->gety();
            break;
        }
    }
    if(king_id == 316){
        team = 'B';
        if(diagonal_check(x, y,team) || horizontal_check(x,y,team) || vertical_check(x, y, team)){
            cm.setb(true);
            castleoperation.setb(false);
            res.mate = true;
            return true;
        }
    }
    else if(king_id == 216){
        team = 'A';
        if(diagonal_check(x, y,team) || horizontal_check(x, y, team) || vertical_check(x, y, team)){
            cm.seta(false);
            castleoperation.seta(false);
            res.mate = true;
            return true;
        }
    }
    else
        std::cout << "\nPLEASE INDICATE YOUR TEAM CORRECTLY" << std::endl;
    res.mate = false;
    return false;    
}

bool checkcastle(chesslab_setup::checkcastle::Request &req, chesslab_setup::checkcastle::Response &res){
    int king_id = req.id;
    char team;
    double x, y;
    std::vector<ObjectInfo>::iterator it;
    double v, z;
    for(it = objects.begin(); it != objects.end(); ++it){
        if(it->getArucoID() == king_id){
            x = it->getx();
            y = it->gety();
            break;
        }
    }
    if(king_id == 316){
        team = 'B';
        if(diagonal_check(x, y,team) || horizontal_check(x,y,team) || vertical_check(x, y, team) || diagonal_check(x, y + 0.050,team) || horizontal_check(x,y + 0.050,team) || vertical_check(x, y + 0.050, team) || diagonal_check(x, y + 0.100,team) || horizontal_check(x,y + 0.100,team) || vertical_check(x, y + 0.100, team)){
            castleoperation.setb(false);
            res.check = false;
            return true;
        }
    }
    else if(king_id == 216){
        team = 'A';
        if(diagonal_check(x, y,team) || horizontal_check(x,y,team) || vertical_check(x, y, team) || diagonal_check(x, y - 0.050,team) || horizontal_check(x,y - 0.050,team) || vertical_check(x, y - 0.050, team) || diagonal_check(x, y - 0.100,team) || horizontal_check(x,y - 0.100,team) || vertical_check(x, y - 0.100, team)){
            castleoperation.seta(false);
            res.check = false;
            return true;
        }
    }
    else
        std::cout << "\nPLEASE INDICATE YOUR TEAM CORRECTLY" << std::endl;
    res.check = true;
    return false;
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "chesslab_setup_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    ROS_INFO("**** starting chesslab_setup node ****");

    ROS_INFO("**** starting kautham setup ****");
    string kautham_path = ros::package::getPath("kautham");
    string modelFolder = kautham_path + "/demos/models/";
    string problemFile = kautham_path + "/demos/OMPL_geo_demos/chess/OMPL_RRTconnect_chess_ur3_gripper_2.xml";

    kauthamOpenProblem(modelFolder, problemFile);

    ROS_INFO("**** starting rviz setup ****");
    SetChessWorld();
    // message declarations
    //teamB
    sensor_msgs::JointState joint_state_teamA_arm;
    joint_state_teamA_arm.name.resize(6);
    joint_state_teamA_arm.position.resize(6);
    joint_state_teamA_arm.position[0] = 0;
    joint_state_teamA_arm.position[1] = 0;
    joint_state_teamA_arm.position[2] = 0;
    joint_state_teamA_arm.position[3] = 0;
    joint_state_teamA_arm.position[4] = 0;
    joint_state_teamA_arm.position[5] = 0;
    
    sensor_msgs::JointState joint_state_teamA_gripper;
    joint_state_teamA_gripper.name.resize(1);
    joint_state_teamA_gripper.position.resize(1);
    joint_state_teamA_gripper.position[0] = 0;
    //teamB
    sensor_msgs::JointState joint_state_teamB_arm;
    joint_state_teamB_arm.name.resize(6);
    joint_state_teamB_arm.position.resize(6);
    joint_state_teamB_arm.position[0] = 0;
    joint_state_teamB_arm.position[1] = 0;
    joint_state_teamB_arm.position[2] = 0;
    joint_state_teamB_arm.position[3] = 0;
    joint_state_teamB_arm.position[4] = 0;
    joint_state_teamB_arm.position[5] = 0;
    
    sensor_msgs::JointState joint_state_teamB_gripper;
    joint_state_teamB_gripper.name.resize(1);
    joint_state_teamB_gripper.position.resize(1);
    joint_state_teamB_gripper.position[0] = 0;

    
    
    //The node advertises the marker poses
    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 1 );
    
    //The node advertises the joint values for teamA robot
    ros::Publisher joint_pub_teamA_arm = n.advertise<sensor_msgs::JointState>("/team_A_arm/joint_states", 1);
    
    //The node advertises the joint values for teamA gripper
    ros::Publisher joint_pub_teamA_gripper = n.advertise<sensor_msgs::JointState>("/team_A_gripper/joint_states", 1);
    
    //The node advertises the joint values for teamB robot
    ros::Publisher joint_pub_teamB_arm = n.advertise<sensor_msgs::JointState>("/team_B_arm/joint_states", 1);
    
    //The node advertises the joint values for teamB gripper
    ros::Publisher joint_pub_teamB_gripper = n.advertise<sensor_msgs::JointState>("/team_B_gripper/joint_states", 1);

    //The node provides a service to set a robot configuration
    ros::ServiceServer serviceRobConf = n.advertiseService("chesslab_setup/setrobconf", setrobconf);
    
    //The node provides a service to set the pose of a piece
    ros::ServiceServer serviceObjPose = n.advertiseService("chesslab_setup/setobjpose", setobjpose);
    
    //The node provides a service to attach a piece to the robot
    ros::ServiceServer serviceAttachObs2Rob = n.advertiseService("chesslab_setup/attachobs2robot", attachobs2robot);
    
    //The node provides a service to dettach a piece from the robot
    ros::ServiceServer serviceDettachObs = n.advertiseService("chesslab_setup/dettachobs", dettachobs);
    
    //The node provides a service to plan a chess movement
    ros::ServiceServer servicePlanmovement = n.advertiseService("chesslab_setup/planmovement", planmovement);

    //The node provides a service to compute the ur3ik
    ros::ServiceServer serviceIK = n.advertiseService("chesslab_setup/inversekin", inversekin);
    
    //The node provides a service to reset the scene to the inital configuration
    ros::ServiceServer serviceResetscene = n.advertiseService("chesslab_setup/resetscene", resetscene);

    //The node provides a service to share the ArucoID of the will be moved piece
    ros::ServiceServer serviceShareID = n.advertiseService("chesslab_setup/shareID", shareID);

    //The node provides a service to get obstacle position
    ros::ServiceServer servicegetobstaclepos = n.advertiseService("chesslab_setup/getobstaclepos",getobstaclepos);

    //The node provides a service to check rule of chess
    ros::ServiceServer servcieCheckRule = n.advertiseService("chesslab_setup/checkrule", checkRule);

    //The node provides a service to check checkmate situation
    ros::ServiceServer serviceCheckMate = n.advertiseService("chesslab_setup/checkmate", checkmate);
    
    //The node provides a service to check castle operation
    ros::ServiceServer serviceCheckCastle = n.advertiseService("chesslab_setup/checkcastle", checkcastle);

    tf2_ros::TransformListener tfListener(tfBuffer);
    std::vector <float> pos;
    int i;
    while (ros::ok())
    {
        i = 0;
        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray markers;


        //PIECES SETUP
        ROS_DEBUG("LOADING OBJECTS");
        ROS_DEBUG("%d OBJECTs",(int)objects.size());

        for(i=0; i<objects.size(); i++) {
            //marker.header.frame_id = "/chess_frame";
            marker.header.frame_id = objects[i].get_frame_id();
            marker.header.stamp = ros::Time();
            marker.ns = "chess_pieces";
            marker.id = objects[i].getArucoID();
            marker.mesh_resource = objects[i].getObjPath();

            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose = objects[i].getPose();
            
            marker.scale.x = objects[i].getscale_x();
            marker.scale.y = objects[i].getscale_y();
            marker.scale.z = objects[i].getscale_z();
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = objects[i].getr();
            marker.color.g = objects[i].getg();
            marker.color.b = objects[i].getb();
            marker.mesh_use_embedded_materials = objects[i].get_use_embeded_materials();//if true the r,g,b peviously defined are overriden


            ROS_DEBUG("OBJECT: [%d]", marker.id);
            ROS_DEBUG("X value is: [%f]", marker.pose.position.x);
            ROS_DEBUG("Y value is: [%f]", marker.pose.position.y);
            ROS_DEBUG("Z value is: [%f]", marker.pose.position.z);
            ROS_DEBUG("ORI X value is: [%f]", marker.pose.orientation.x);
            ROS_DEBUG("ORI Y value is: [%f]", marker.pose.orientation.y);
            ROS_DEBUG("ORI Z value is: [%f]", marker.pose.orientation.z);
            ROS_DEBUG("ORI W value is: [%f]", marker.pose.orientation.w);
            
            markers.markers.push_back(marker);
        }
        vis_pub.publish( markers );


        //ROBOT SETUP
        ROS_DEBUG("SETTING ROBOT");
        //update joint_state
        //teamA
        joint_state_teamA_arm.position[0] = conf[0];
        joint_state_teamA_arm.position[1] = conf[1];
        joint_state_teamA_arm.position[2] = conf[2];
        joint_state_teamA_arm.position[3] = conf[3];
        joint_state_teamA_arm.position[4] = conf[4];
        joint_state_teamA_arm.position[5] = conf[5];
        joint_state_teamA_gripper.position[0] = conf[6];
        //teamB
        joint_state_teamB_arm.position[0] = conf[7];
        joint_state_teamB_arm.position[1] = conf[8];
        joint_state_teamB_arm.position[2] = conf[9];
        joint_state_teamB_arm.position[3] = conf[10];
        joint_state_teamB_arm.position[4] = conf[11];
        joint_state_teamB_arm.position[5] = conf[12];
        joint_state_teamB_gripper.position[0] = conf[13];

        //teamA
        joint_state_teamA_arm.header.stamp = ros::Time::now();
        joint_state_teamA_arm.name[0] ="team_A_shoulder_pan_joint";
        joint_state_teamA_arm.name[1] ="team_A_shoulder_lift_joint";
        joint_state_teamA_arm.name[2] ="team_A_elbow_joint";
        joint_state_teamA_arm.name[3] ="team_A_wrist_1_joint";
        joint_state_teamA_arm.name[4] ="team_A_wrist_2_joint";
        joint_state_teamA_arm.name[5] ="team_A_wrist_3_joint";
        joint_state_teamA_gripper.header.stamp = joint_state_teamA_arm.header.stamp;
        joint_state_teamA_gripper.name[0] ="team_A_gripper_right_driver_joint";
        //teamB
        joint_state_teamB_arm.header.stamp = joint_state_teamA_arm.header.stamp;
        joint_state_teamB_arm.name[0] ="team_B_shoulder_pan_joint";
        joint_state_teamB_arm.name[1] ="team_B_shoulder_lift_joint";
        joint_state_teamB_arm.name[2] ="team_B_elbow_joint";
        joint_state_teamB_arm.name[3] ="team_B_wrist_1_joint";
        joint_state_teamB_arm.name[4] ="team_B_wrist_2_joint";
        joint_state_teamB_arm.name[5] ="team_B_wrist_3_joint";
        joint_state_teamB_gripper.header.stamp = joint_state_teamA_arm.header.stamp;
        joint_state_teamB_gripper.name[0] ="team_B_gripper_right_driver_joint";

        //send the joint state
        joint_pub_teamA_arm.publish(joint_state_teamA_arm);
        joint_pub_teamA_gripper.publish(joint_state_teamA_gripper);
        joint_pub_teamB_arm.publish(joint_state_teamB_arm);
        joint_pub_teamB_gripper.publish(joint_state_teamB_gripper);

        //ROS_INFO_STREAM("joints A: "<<joint_state_teamA_arm.position[0]<<"," \
                                    <<joint_state_teamA_arm.position[1]<<"," \
                                    <<joint_state_teamA_arm.position[2]<<"," \
                                    <<joint_state_teamA_arm.position[3]<<"," \
                                    <<joint_state_teamA_arm.position[4]<<"," \
                                    <<joint_state_teamA_arm.position[5]);

        //ROS_INFO_STREAM("joints B: "<<joint_state_teamB_arm.position[0]<<"," \
                                    <<joint_state_teamB_arm.position[1]<<"," \
                                    <<joint_state_teamB_arm.position[2]<<"," \
                                    <<joint_state_teamB_arm.position[3]<<"," \
                                    <<joint_state_teamB_arm.position[4]<<"," \
                                    <<joint_state_teamB_arm.position[5]);

       
        ros::spinOnce();
        loop_rate.sleep();
    }
}
