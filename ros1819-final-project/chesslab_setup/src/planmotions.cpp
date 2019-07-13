#include <iostream>
#include <ros/ros.h>
#include <string>
//#include <ros/package.h>
#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
//#include <sensor_msgs/JointState.h>

//#include <chesslab_setup/setrobconf.h>
//#include <chesslab_setup/setobjpose.h>
//#include <chesslab_setup/attachobs2robot.h>
//#include <chesslab_setup/dettachobs.h>
#include <chesslab_setup/ffplan.h>
#include <chesslab_setup/shareID.h>
#include "chesslab_setup/getobstaclepos.h"
#include "chesslab_setup/checkrule.h"
#include "chesslab_setup/checkmate.h"
#include "chesslab_setup/checkcastle.h"
//#include <chesslab_setup/ik.h>


#include "object_info.hpp"

#include <iterator>
int main( int argc, char** argv )
{
    ros::init(argc, argv, "demo_planmotions");
    ros::NodeHandle node;

    SetChessWorld();

    ROS_INFO("**** starting chesslab_setup client node to test the planning of motions using the FF task planner ****");
    
    //call the planmovement service
    ros::service::waitForService("/chesslab_setup/planmovement");
    ros::ServiceClient planmovement_client = node.serviceClient<chesslab_setup::ffplan>("/chesslab_setup/planmovement");
    chesslab_setup::ffplan planmovement_srv;
       
    /* MOVE ACTION */
    std::cout<<"\n*** MOVE ACTION ***"<<std::endl;

    ros::service::waitForService("/chesslab_setup/shareID");
    ros::ServiceClient shareID_client = node.serviceClient<chesslab_setup::shareID>("chesslab_setup/shareID");
    chesslab_setup::shareID shareID_srv;

    ros::service::waitForService("/chesslab_setup/getobstaclepos");
    ros::ServiceClient getobstaclepos_client = node.serviceClient<chesslab_setup::getobstaclepos>("chesslab_setup/getobstaclepos");
    chesslab_setup::getobstaclepos getobstaclepos_srv;

    ros::service::waitForService("/chesslab_setup/checkrule");
    ros::ServiceClient checkrule_client = node.serviceClient<chesslab_setup::checkrule>("chesslab_setup/checkrule");
    chesslab_setup::checkrule checkrule_srv;

    
    ros::service::waitForService("/chesslab_setup/checkmate");
    ros::ServiceClient checkmate_client = node.serviceClient<chesslab_setup::checkmate>("chesslab_setup/checkmate");
    chesslab_setup::checkmate checkmate_srv;

    ros::service::waitForService("/chesslab_setup/checkcastle");
    ros::ServiceClient checkcastle_client = node.serviceClient<chesslab_setup::checkcastle>("chesslab_setup/checkcastle");
    chesslab_setup::checkcastle checkcastle_srv;
    

    int id, goal_id, rook_id, king_id;
    std::string c;
    int choice;
    std::string goal;
    bool castle = true;
    char team;
    std::map<std::string,Cell>::iterator it;
    bool found = false;
    ////////////////////////////////////////////////////////
    //Wait for user give piece and goal position
    std::cout <<"\n PLEASE CHOOSE WHAT DO YOU WANT : "; 
    std::cout <<"\n 0)LEARN THE POSITION OF THE PIECE WITH ArucoID ";
    std::cout <<"\n 1)MOVE THE PIECE  ";
    std::cout <<"\n 2)MOVE AND KILL" ;
    std::cout <<"\n 3)CASTLE OPERATION" << std::endl;
    std::cin >> choice;

    switch (choice)
    {
        case 0:
            std::cout <<"\nPlEASE GIVE INFORMATION OF THE PIECE IN THE FOLLOWING FORMAT: ArucoID(210) ..."<<std::endl;
            std::cin >> id;
            break;
        case 1:
            std::cout <<"\nPlEASE GIVE INFORMATION OF THE PIECE AND DESIRED POSITION IN THE FOLLOWING FORMAT: ArucoID(210) POS(A7) ..."<<std::endl;
            std::cin >> id >> goal ;
            break;
        case 2:
            std::cout <<"\nPLEASE GIVE INFORMATION OF THE PIECE THAT YOU WANT TO MOVE AND KILL IN THE FOLLOWING FORMAT: ArucoID(210) ArucoID(310)" << std::endl;
            std::cin >> id >> goal_id;
            break;

        case 3:
            std::cout << "\nPLEASE CHOOSE YOUR TEAM. A OR B?" << std::endl;
            std::cin >> team;
            if (team == 'a' || team == 'A'){
                team = 'A';
                rook_id = 209;
                king_id = 216;
            }
            else if (team == 'b' || team == 'B'){
                team = 'B';
                rook_id = 310;
                king_id = 316;
            }
            else{
                std::cout << "\nPLEASE INDICATE YOUR TEAM CORRECTLY" << std::endl;
            }
            break;

        default:
                std::cout <<"\n PLEASE CHOOSE WHAT DO YOU WANT : "; 
                std::cout <<"\n 0)LEARN THE POSITION OF THE PIECE WITH ArucoID ";
                std::cout <<"\n 1)MOVE THE PIECE  ";
                std::cout <<"\n 2)MOVE AND KILL" ;
                std::cout <<"\n 3)CASTLE OPERATION" << std::endl;
                break;
    }
    
    if(choice != 3){
        shareID_srv.request.objarucoid = id;
        shareID_client.call(shareID_srv);
        
        int i = shareID_srv.response.objkauthamid;
        
        getobstaclepos_srv.request.objkauthamid = i;
        getobstaclepos_client.call(getobstaclepos_srv);
        
        std::vector<double> pos(7);
        
        pos[0] = getobstaclepos_srv.response.pos.position.x;
        pos[1] = getobstaclepos_srv.response.pos.position.y;
        pos[2] = getobstaclepos_srv.response.pos.position.z;
        pos[3] = getobstaclepos_srv.response.pos.orientation.x;
        pos[4] = getobstaclepos_srv.response.pos.orientation.y;
        pos[5] = getobstaclepos_srv.response.pos.orientation.z;
        pos[6] = getobstaclepos_srv.response.pos.orientation.w;

        for (it= mymap.begin(); it!= mymap.end(); ++it){
            if (pos[0] == it->second.getx() && pos[1] == it->second.gety()){    
                c = it->first;
                found = true;
            }
        }
        if(found)
            cout << "\nThe cell of the given piece is " << c << endl;
        else
            cout << "\nCouldn't find the piece " << endl;
    }
    if (choice == 1){
        shareID_srv.request.posit = goal;
        shareID_client.call(shareID_srv); 

        checkrule_srv.request.initial_pos = c;
        checkrule_srv.request.goal_pos = goal;
        checkrule_srv.request.id = id;
        checkrule_client.call(checkrule_srv);
        if (checkrule_srv.response.check){
            if(id == 209 || id == 210)
                castleoperation.seta(false);
            else if(id == 309 || id == 310)
                castleoperation.setb(false);        
            if (!shareID_srv.response.free)
                goal_id = shareID_srv.response.objArucoID;
            else
                goal_id = id;
            
            planmovement_srv.request.init.objarucoid.push_back(id);//BLACK_PAWN_1 in A4
            planmovement_srv.request.init.occupiedcells.push_back(c);
            planmovement_srv.request.init.freecells.push_back(goal); //bu tasi at demekmis
            planmovement_srv.request.goal.objarucoid.push_back(goal_id);//BLACK_PAWN_1 in B5
            planmovement_srv.request.goal.occupiedcells.push_back(goal);

            planmovement_client.call(planmovement_srv);
                
            std::cout<<"Initial state:"<<std::endl;
            for(int i=0; i<planmovement_srv.request.init.objarucoid.size(); i++)
            {
                std::cout<<planmovement_srv.request.init.objarucoid[i]<<" " <<planmovement_srv.request.init.occupiedcells[i]<<std::endl;
            }
            std::cout<<"Goal state:"<<std::endl;
            for(int i=0; i<planmovement_srv.request.goal.objarucoid.size(); i++)
            {
                std::cout<<planmovement_srv.request.goal.objarucoid[i]<<" " <<planmovement_srv.request.goal.occupiedcells[i]<<std::endl;
            }
            std::cout<<"Plan:"<<std::endl;
            for(int i=0; i<planmovement_srv.response.plan.size(); i++)
            {
                std::cout<<planmovement_srv.response.plan[i]<<std::endl;
            }
        }
    }

    else if(!shareID_srv.response.free && choice == 2){
                    
        /* KILL ACTION*/
        shareID_srv.request.objarucoid = goal_id;
        shareID_client.call(shareID_srv);

        if(id == 209 || id == 210)
            castleoperation.seta(false);
        else if(id == 309 || id == 310)
            castleoperation.setb(false);       
    
        int v = shareID_srv.response.objkauthamid;
    
        getobstaclepos_srv.request.objkauthamid = v;
        getobstaclepos_client.call(getobstaclepos_srv);
    
        std::vector<double> position(2);
    
        position[0] = getobstaclepos_srv.response.pos.position.x;
        position[1] = getobstaclepos_srv.response.pos.position.y;

        checkrule_srv.request.initial_pos = c;
        checkrule_srv.request.goal_pos = goal;
        checkrule_srv.request.id = id;
        checkrule_client.call(checkrule_srv);

        found = false;
        for (it= mymap.begin(); it!= mymap.end(); ++it){
            if (position[0] == it->second.getx() && position[1] == it->second.gety()){    
                goal = it->first;
                found = true;
            }
        }
        if(found)
            cout << "\nThe cell of the will be killed piece is " << goal << endl;
        else
            cout << "\nCouldn't find the piece " << endl;
    
                
        std::cout<<"\n*** KILL ACTION ***"<<std::endl;
        planmovement_srv.request.init.objarucoid.push_back(id);//BLACK_PAWN_1 in A4
        planmovement_srv.request.init.occupiedcells.push_back(c);
        planmovement_srv.request.init.objarucoid.push_back(goal_id);//WHITE_PAWN_1 in B5
        planmovement_srv.request.init.occupiedcells.push_back(goal);
        planmovement_srv.request.goal.objarucoid.push_back(goal_id);//BLACK_PAWN_1 in B5
        planmovement_srv.request.goal.occupiedcells.push_back(goal);
        planmovement_srv.request.killed = goal_id;//WHITE_PAWN_1 KILLED


        planmovement_client.call(planmovement_srv);

        std::cout<<"Initial state:"<<std::endl;
        for(int i=0; i<planmovement_srv.request.init.objarucoid.size(); i++)
        {
            std::cout<<planmovement_srv.request.init.objarucoid[i]<<" " <<planmovement_srv.request.init.occupiedcells[i]<<std::endl;
        }
        std::cout<<"Goal state:"<<std::endl;
        for(int i=0; i<planmovement_srv.request.goal.objarucoid.size(); i++)
        {
            std::cout<<planmovement_srv.request.goal.objarucoid[i]<<" " <<planmovement_srv.request.goal.occupiedcells[i]<<std::endl;
        }
        std::cout<<"Plan:"<<std::endl;
        for(int i=0; i<planmovement_srv.response.plan.size(); i++)
        {
            std::cout<<planmovement_srv.response.plan[i]<<std::endl;
        }
    }
    else if(shareID_srv.response.free && choice == 2){
        std::cout << "THERE IS NO PIECE TO BE KILLED" << std::endl;
    }
    else if(choice == 3){
 
        checkmate_srv.request.id = king_id;
        checkmate_client.call(checkmate_srv);


        if(checkmate_srv.response.mate == true){
            std::cout <<"\n YOUR KING IS UNDER THREAD. YOU ARE NOT ALLOWED TO USE CASTLE OPERATION." << std::endl;
        }
        else{
            checkcastle_srv.request.id = king_id;
            checkcastle_client.call(checkcastle_srv);


            if(checkcastle_srv.response.check){
            
                shareID_srv.request.objarucoid = rook_id;
                shareID_client.call(shareID_srv);
                
                int i = shareID_srv.response.objkauthamid;
                
                getobstaclepos_srv.request.objkauthamid = i;
                getobstaclepos_client.call(getobstaclepos_srv);
                
                std::vector<double> pos_rook(2);
                std::vector<double> pos_king(2);
                
                pos_rook[0] = getobstaclepos_srv.response.pos.position.x;
                pos_rook[1] = getobstaclepos_srv.response.pos.position.y;
                    
                shareID_srv.request.objarucoid = king_id;
                shareID_client.call(shareID_srv);
            
                i = shareID_srv.response.objkauthamid;
                
                getobstaclepos_srv.request.objkauthamid = i;
                getobstaclepos_client.call(getobstaclepos_srv);
                
                pos_king[0] = getobstaclepos_srv.response.pos.position.x;
                pos_king[1] = getobstaclepos_srv.response.pos.position.y;

                if(team == 'A' && pos_king[0] == 0.175 && pos_king[1] == -0.025 && castleoperation.geta() == true){//castle a bütün kontrolleri yükle

                    planmovement_srv.request.init.objarucoid.push_back(rook_id);//BLACK_PAWN_1 in A4
                    planmovement_srv.request.init.occupiedcells.push_back("H1");
                    planmovement_srv.request.init.freecells.push_back("F1"); //bu tasi at demekmis
                    planmovement_srv.request.goal.objarucoid.push_back(rook_id);//BLACK_PAWN_1 in B5
                    planmovement_srv.request.goal.occupiedcells.push_back("F1");

                    planmovement_client.call(planmovement_srv);
                        
                    std::cout<<"Initial state:"<<std::endl;
                    for(int i=0; i<planmovement_srv.request.init.objarucoid.size(); i++)
                    {
                        std::cout<<planmovement_srv.request.init.objarucoid[i]<<" " <<planmovement_srv.request.init.occupiedcells[i]<<std::endl;
                    }
                    std::cout<<"Goal state:"<<std::endl;
                    for(int i=0; i<planmovement_srv.request.goal.objarucoid.size(); i++)
                    {
                        std::cout<<planmovement_srv.request.goal.objarucoid[i]<<" " <<planmovement_srv.request.goal.occupiedcells[i]<<std::endl;
                    }
                    std::cout<<"Plan:"<<std::endl;
                    for(int i=0; i<planmovement_srv.response.plan.size(); i++)
                    {
                        std::cout<<planmovement_srv.response.plan[i]<<std::endl;
                    }

                    planmovement_srv.request.init.objarucoid.push_back(king_id);//BLACK_PAWN_1 in A4
                    planmovement_srv.request.init.occupiedcells.push_back("E1");
                    planmovement_srv.request.init.freecells.push_back("G1"); //bu tasi at demekmis
                    planmovement_srv.request.goal.objarucoid.push_back(king_id);//BLACK_PAWN_1 in B5
                    planmovement_srv.request.goal.occupiedcells.push_back("G1");

                    planmovement_client.call(planmovement_srv);
                        
                    std::cout<<"Initial state:"<<std::endl;
                    for(int i=0; i<planmovement_srv.request.init.objarucoid.size(); i++)
                    {
                        std::cout<<planmovement_srv.request.init.objarucoid[i]<<" " <<planmovement_srv.request.init.occupiedcells[i]<<std::endl;
                    }
                    std::cout<<"Goal state:"<<std::endl;
                    for(int i=0; i<planmovement_srv.request.goal.objarucoid.size(); i++)
                    {
                        std::cout<<planmovement_srv.request.goal.objarucoid[i]<<" " <<planmovement_srv.request.goal.occupiedcells[i]<<std::endl;
                    }
                    std::cout<<"Plan:"<<std::endl;
                    for(int i=0; i<planmovement_srv.response.plan.size(); i++)
                    {
                        std::cout<<planmovement_srv.response.plan[i]<<std::endl;
                    }

                    std::cout << "\nCASTLE OPERATION IS SUCCESFULLY PERFORMED" << std::endl;
                }
                else if(team == 'B' && pos_king[0] == -0.175 && pos_king[1] == 0.025 && castleoperation.getb() == true){

                    planmovement_srv.request.init.objarucoid.push_back(rook_id);//BLACK_PAWN_1 in A4
                    planmovement_srv.request.init.occupiedcells.push_back("A8");
                    planmovement_srv.request.init.freecells.push_back("C8"); //bu tasi at demekmis
                    planmovement_srv.request.goal.objarucoid.push_back(rook_id);//BLACK_PAWN_1 in B5
                    planmovement_srv.request.goal.occupiedcells.push_back("C8");

                    planmovement_client.call(planmovement_srv);
                        
                    std::cout<<"Initial state:"<<std::endl;
                    for(int i=0; i<planmovement_srv.request.init.objarucoid.size(); i++)
                    {
                        std::cout<<planmovement_srv.request.init.objarucoid[i]<<" " <<planmovement_srv.request.init.occupiedcells[i]<<std::endl;
                    }
                    std::cout<<"Goal state:"<<std::endl;
                    for(int i=0; i<planmovement_srv.request.goal.objarucoid.size(); i++)
                    {
                        std::cout<<planmovement_srv.request.goal.objarucoid[i]<<" " <<planmovement_srv.request.goal.occupiedcells[i]<<std::endl;
                    }
                    std::cout<<"Plan:"<<std::endl;
                    for(int i=0; i<planmovement_srv.response.plan.size(); i++)
                    {
                        std::cout<<planmovement_srv.response.plan[i]<<std::endl;
                    }

                    planmovement_srv.request.init.objarucoid.push_back(king_id);//BLACK_PAWN_1 in A4
                    planmovement_srv.request.init.occupiedcells.push_back("D8");
                    planmovement_srv.request.init.freecells.push_back("B8"); //bu tasi at demekmis
                    planmovement_srv.request.goal.objarucoid.push_back(king_id);//BLACK_PAWN_1 in B5
                    planmovement_srv.request.goal.occupiedcells.push_back("B8");

                    planmovement_client.call(planmovement_srv);
                        
                    std::cout<<"Initial state:"<<std::endl;
                    for(int i=0; i<planmovement_srv.request.init.objarucoid.size(); i++)
                    {
                        std::cout<<planmovement_srv.request.init.objarucoid[i]<<" " <<planmovement_srv.request.init.occupiedcells[i]<<std::endl;
                    }
                    std::cout<<"Goal state:"<<std::endl;
                    for(int i=0; i<planmovement_srv.request.goal.objarucoid.size(); i++)
                    {
                        std::cout<<planmovement_srv.request.goal.objarucoid[i]<<" " <<planmovement_srv.request.goal.occupiedcells[i]<<std::endl;
                    }
                    std::cout<<"Plan:"<<std::endl;
                    for(int i=0; i<planmovement_srv.response.plan.size(); i++)
                    {
                        std::cout<<planmovement_srv.response.plan[i]<<std::endl;
                    }

                    std::cout << "\nCASTLE OPERATION IS SUCCESFULLY PERFORMED" << std::endl;
                }
                else{
                    std::cout << "\nYOU ARE NOT ALLOWED FOR CASTLE OPERATION.PLEASE CHECK YOUR KING AND TOWER." << std::endl;
                }
            }
            else{
                std::cout << "\nYOU ARE NOT ALLOWED FOR CASTLE OPERATION.PLEASE CHECK YOUR KING AND TOWER." << std::endl;
            }
        }
    }  
}
