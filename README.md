# chess_with_ROS
				INTRODUCTION TO ROS
				      Buse Sibel Korkmaz 

1. General Information
	This project aims give ability to the user playing chess with robot arms by the help of Robot Operating System(ROS). Some of the packages and services had already provided. Other packages and services which will be mentioned in the following sections, created in order to accomplish the aim.  

2.Technical Information
	This projects includes 3 different module:
		1)Action manager module
		2)Planning module
		3)Sensing module
	
Those modules work together. 

	1)Action manager module
	
		Action manager module manages the whole process together. Most of the project is directed by main node in node.cpp. Firstly, all class types are defined in object_info.hpp and they are included to node.cpp. 
		
	There are 4 class types:
	A) ObjectInfo: It is a pre-defined class in order to get and set knowledge about chess pieces.
	B) Cell: It is defined later, in order to get and set cell code of the chess board.
	C)Checkmate: It is definded later, in order to get and set checkmate situations of the teams.
	D)Castle: It is defined later, in order to get and set whether castle operation is allowed or 	not.

Then objects vector is created with IDs, positions and paths, also mymap is created by cell code and cell positions (x,y) in SetChessWorld() function. After creating chess world, new service types are created in order to communication between node and other cpp files with other words server and client programs. 
	
	A)Service ShareID: ShareID takes object Aruco ID and position as a cell code from client. If client request by id, it reponses as object kautham ID, position in type of geometry_msgs/Pose, cell code, if the piece is given with Aruco ID is found bool found will be true. If the client request with cell code, shareID responses as object Aruco ID and if the given cell code does not contain object bool free will be true.
	
	B)Service getobstaclepos: getobstaclepos takes object kautham ID from client. It responses as the position of the object in type of geometry_msgs/Pose and if the object is found in objects vector bool found will be true.
	
	C)Service checkrule: checkrule takes the Aruco ID of the piece will be moved, goal position and initial position. It checks the movement whether is allowed by chess rules or not. If is not allowed it responses as bool check is set false.
	
	D)Service checkmate: checkmate takes the id of the piece will be moved and decide its team. Also finds x and y position of king of the team. Then, this service checks the checkmate situation by the help of diagonal_check, vertical_check and horizontal_check functions. Those three functions get the position of the king and the team, they move in the mymap (chess board) diagonally, vertically and horizontally and they check is there any piece can be thread for the king. If it is they return true. If non of those three function return true, checkmate service responses as bool mate is false. Also, changes variable of cm which is created in type of Checkmate.

	E)Service checkcastle: checkcastle take the id of the king which is desired to do castle operation and decides its team. After deciding, it checks the necessary cells according to checkmate thread by the help of diagonal_check, vertical_check and horizal_check functions. If the check complete without return true, castle operation is allowed. It responses as bool check true. Other conditions such as tower had not moved ant the position of king is checked inside of the planmotions with Castle type castleoperation obejct with if-else structure.
	

After that, necessary .srv files created and linked to package with CmakeLists.text. As a final step, servers are initialized in main function of node.cpp.

Chess queries are obtained from clients and they are performed by this server-client relation.

	2)Planning module
	
In planmotions.cpp, necessary client files are initialized. 4 options are served to user:
	
	0.Learn the position of the piece
	1.Move the piece
	2.Move and kill
	3. Castle operation

According to choice of the user following actions are taken:
	
	0. Take Aruco ID of the piece and return cell code by shareID_srv.
	1.Take Aruco ID of the piece and the goal position. Find cell code of the piece and check the goal position whether is free or not, and check the movement is allowed by chess rules. If all conditions are satisfied, then give positions and id to planmovement service. Perform the movement.
	2.Take Aruco ID of the both piece and find their position also check the movement is allowed by chess rule such as choice 2. If all conditions are satisfied give ID and positions to planmovement service. Perform the kill action.
	3. Take the team name. Check the castle is allowed or not by checkcastle service and the necessary positions. If it is allowed perform the castle operation. Give ids and positions to planmovement service.   
		
In movepiece.cpp, take the object Aruco ID from the user  and goal position. Then, find the position by shareID. Set goal position to the piece by setobjpose service.

In ik.cpp, take the goal position and object KauthamID. Then find the position of the piece by getobstaclepos service. Find inverse kinematics solution with ik service. Take one of the solutions and set robot configurations by setrobconf service.
