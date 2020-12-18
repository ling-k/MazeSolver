# maze_solver
This is the course project for Mobile Robotics. 
The components of the project include SLAM, motion planning, robot following, and creativity task. 

# Part 1

# Part 2

# Part 3 Rescue operation
For the above two methods, you can start with the following instructions:
Method 1 (search along the wall) : roslaunch final_project start_quick.launch 
Method 2 (search with specific locations) : roslaunch final_project start_full.launch 

## Description of feature pack：
-final_project: The main package include starting launch files for the project.

-maze_gmapping: The package for the Robot 1 gmapping
-- auto_gmapping.launch: Enable both gmapping and navigation stack

-fira_maze:Loading simulation environment

-maze_navigation: 
-- maze_navigation.launch : Start the navigation stack for Robot 1, using the map that has been built
-- amcl.launch : Start AMCL positioning function package to provide map->robot_tf1/odom coordinate transformation
-- maze_move_base.launch: Start the navigation stack and load the parameters


nav_vel_plan:
-- gmapping_demo.launch: Start the robot2 gmapping and convert odom topic to TF
-- gmapping.launch: Configure the parameters of slam_gmapping
-- robot1.launch: Contains the Robot1 navigation stack launch file and the search node launch file
-- maze_navigation.launch :Start robot2's navigation stack and load the parameters
                         
