#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <iostream>
#include <fstream>

#include <ff/Plan.h>


int main( int argc, char** argv )
{
    ros::init(argc, argv, "ff_client_node");
    ros::NodeHandle node;

    ROS_INFO("**** starting ff client node ****");

    ros::service::waitForService("/FFPlan");
    ros::ServiceClient ff_client = node.serviceClient<ff::Plan>("/FFPlan");
    ff::Plan ff_srv;
    
    //Writing the problem file
    std::stringstream sstr_intro;
    std::stringstream sstr_init;
    std::stringstream sstr_goal;
    
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
    
     //Seeting the init state
     sstr_init << "(:init \n\
        (arm-empty) \n\
        (freecell SAFE_REGION) \n\
        (on BLACK_PAWN_1 A4) \n\
        (on WHITE_PAWN_1 B5)\n\
    )\n";

    //Setting the goal state
    sstr_goal << "(:goal \n\
        (and (on BLACK_PAWN_1 B5) (deadpiece WHITE_PAWN_1)) \n\
    )\n";
    
    //Writing it to a file
    std::string ff_path = ros::package::getPath("ff");
    std::string problemFile = ff_path + "/ff-domains/my_chess_world_problem";
    std::ofstream myfile;
    myfile.open (problemFile, std::ofstream::out);
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
        sstr<<"The computed plan is:"<<std::endl;
        for(int i=0; i<ff_srv.response.plan.size(); i++)
        {
            sstr << ff_srv.response.plan[i] <<std::endl;
        }
        ROS_INFO_STREAM(sstr.str());
    }
    else{
        ROS_INFO("Not able to compute plan");
    }
     
}
