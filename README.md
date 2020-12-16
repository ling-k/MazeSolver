# maze_solver
This is the course project for Mobile Robotics. 
The components of the project include SLAM, motion planning, robot following, and creativity task. 

# Usage
Place two robots in the maze by Launch file: maze_1_two_robots.launch 


#Description of feature pack：
final_project: 
    These are mainly launch files


maze_gmapping: Robot 1 gmapping
    Launch file description:
        auto_gmapping.launch: Enable both gmapping and navigation stack

explore_lite: Automatic gmapping of robot 1
    Launch file description:
        explore.launch :Start the auto-gmapping node, but you need to start the gmapping feature                    pack and navigation stack first


fira_maze:  
    Loading simulation environment


maze_navigation: 
    Launch file description:
        maze_navigation.launch : Start the navigation stack for Robot 1, using the map that has been                          built
        amcl.launch : Start AMCL positioning function package to provide map->robot_tf1/odom                       coordinate transformation
        maze_move_base.launch: Start the navigation stack and load the parameters



nav_vel_plan:
    Launch file description:
        gmapping_demo.launch: Start the robot2 gmapping and convert odom topic to TF
        gmapping.launch: Configure the parameters of slam_gmapping

        robot1.launch: Contains the Robot1 navigation stack launch file and the search node launch                  file
        robot2.launch: Contains the Robot2 navigation stack launch file and the follow node launch                  file

        maze_navigation.launch :Start robot2's navigation stack and load the parameters
                                Robot2 navigates purely using the local cost map 

