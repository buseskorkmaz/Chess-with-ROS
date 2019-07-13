This project contains the ROS wrapper of the FF task planner written by Joerg Hoffmann.

There is a rosnode to execute the planner from the terminal and another that offers FF as a servvice.

**1)** To run the **terminal node**, launch the ff_node.launch file. It has two arguments that correspond to the names of files stored in the ff-domain folder. they are:  
 a) argument *domain*: It is a pddl file describing the actions; it is defaulted to 'block_world.pddl'.  
 b) argument *problem*: It is a file describing the objects and the initial and goal states; it is defaulted to 'block_world'.  

**2)** To run the **service node**, launch the ff_service.launch file. It can be tested by calling the service from another terminal:  
 $ rosservice call /FFPlan "problem: 'path/block_world' action: 'path/block_world.pddl'"  
where path is the absolute path to the files.

Examples of the chess domain:

- Move a pawn (i.e. move a pawn to a free cell):  
 $ rosservice call /FFPlan "problem: '/home/jan/catkin/catkin_wsROS/src/git-ROS-FF/ff-domains/chess_world'
domain: '/home/jan/catkin/catkin_wsROS/src/git-ROS-FF/ff-domains/chess_world.pddl'"
- Kill a pawn (i.e. move a pawn to a cell occupied by another pawn): 
 $ rosservice call /FFPlan "problem: '/home/jan/catkin/catkin_wsROS/src/git-ROS-FF/ff-domains/chess_world_2'
domain: '/home/jan/catkin/catkin_wsROS/src/git-ROS-FF/ff-domains/chess_world.pddl'"
- Castling:  
 $ rosservice call /FFPlan "problem: '/home/jan/catkin/catkin_wsROS/src/git-ROS-FF/ff-domains/chess_world_3'
domain: '/home/jan/catkin/catkin_wsROS/src/git-ROS-FF/ff-domains/chess_world.pddl'"

To perform your desired movement you should load the problem file with its description.

