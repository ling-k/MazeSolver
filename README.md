# maze_solver
This is the course project for Mobile Robotics. 
The components of the project include SLAM, motion planning, robot following, and creativity task. 

# Part 1

# Part 2

# Part 3 Rescue operation
Here are two ways for the Robot M to find the Robot S. The first one is to search for the location of the entire maze by visiting points in specific locations. This method is relatively fast, but requires a preset location. The second method is to search the entire maze area by random walk, which is relatively slow, but does not require any pre-setting.

For the above two methods, you can start with the following instructions:
Method 1: roslaunch final_project start_quick.launch
Method 2: roslaunch final_project start_full.launch 

## Description of feature pack：
final_project: The main package include starting launch files for the project.

maze_gmapping: The package for the Robot 1 gmapping
Launch file description:
    auto_gmapping.launch: Enable both gmapping and navigation stack

explore_lite: Automatic gmapping of robot 1
Launch file description:
    explore.launch :Start the auto-gmapping node, but you need to start the gmapping feature pack and navigation stack first


fira_maze:
    Loading simulation environment


maze_navigation: 
Launch file description:
    maze_navigation.launch : Start the navigation stack for Robot 1, using the map that has been built
    amcl.launch : Start AMCL positioning function package to provide map->robot_tf1/odom coordinate transformation
    maze_move_base.launch: Start the navigation stack and load the parameters



nav_vel_plan:
Launch file description:
    gmapping_demo.launch: Start the robot2 gmapping and convert odom topic to TF
    gmapping.launch: Configure the parameters of slam_gmapping
    robot1.launch: Contains the Robot1 navigation stack launch file and the search node launch file
    maze_navigation.launch :Start robot2's navigation stack and load the parameters
                                Robot2 navigates purely using the local cost map 

