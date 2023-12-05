
# Temporal Memory-based RRT (TM-RRT) Exploration (ROS Noetic)

Simulation of the TM-RRT Exploration based on ROS Noetic can be found here:
[TM-RRT Exploration Simulation](https://github.com/hikashi/TM-RRT_exploration_Simulation "TM-RRT Exploration").

This is a temporal memory-based rapidly-exploring random tree (TM-RRT) algorithm, which is based on the original RRT exploration by Hassan Umari.

credit to Hasauino for creating the RRT exploration packages.
[RRT Exploration package](https://github.com/hasauino/rrt_exploration "RRT Exploration").
[RRT Exploration Tutorial package](https://github.com/hasauino/rrt_exploration_tutorials "RRT Exploration").

There are a few modifications done in order to improve the efficiency of exploration, which are mainly focused on the assigner module.
- memory-based assigner, where the assigner keeps track of the goal history and prevents other robots from exploring the same location
- time limit imposed on the goal based on the expected velocity to prevent the robot from spending time on an unreachable goal.


## Requirements
The following code is executed in ROS noetic in Ubuntu 20.04 LTS, Python 3.X

The following libraries are required to be installed before proceeding to run the code

    $ sudo apt-get install ros-noetic-gmapping
    $ sudo apt-get install ros-noetic-navigation
    $ sudo apt-get install python3-pip
    $ pip3 install scikit-learn
    
## Installation Process
Create a new folder called "catkin_explore/src" by executing the following comment:

    $ sudo mkdir -p ~/catkin_explore/src
    $ cd ~/catkin_explore/src/
    $ git clone -b main-Noetic https://github.com/hikashi/TM-RRT_exploration.git
    $ cd ~/catkin_explore
    $ catkin_make

## Fixing error of "Make sure file exists in package path and permission is set to executable (chmod +x)"
This is probably due to the files being copy over from the github.
This can be solved using the following commands:

	$ chmod +x ~/catkin_explore/TM-RRT_exploration/tm_rrt_exploration/scripts/*

## TF Tree Requirement
 ![TF_tree_example](/TF_tree_example.PNG)

## Demonstration of TM-RRT in Clearpath's Jackal
Kindly refer to the video below for a demonstration of TM-RRT Exploration:
- three robots (Clear Path's Jackal)

[![TM-RRT Exploration for Three Jackals](https://img.youtube.com/vi/rtFwfaKPcMw/0.jpg)](https://www.youtube.com/watch?v=rtFwfaKPcMw "TM-RRT Exploration for three Jackals")

- two different ground robots (Jackal and Ghost Vision 60)

[![TM-RRT Exploration for two different robots](https://img.youtube.com/vi/y49VpjyBovw/0.jpg)](https://www.youtube.com/watch?v=y49VpjyBovw "TM-RRT Exploration for two different robots")


## Execution Commands 
Comments for running/executing the TM-RRT exploration after sourcing devel/setup.bash

            $ roslaunch tmrrt_exploration trio_exploration.launch


## ROS PARAMETER
ros parameters for setting up the robot
- boundary.py
    - map frame >> map frame for the boundary points to be published
    - n_point >> number of clicked_points to be used to draw the boundary
    - start_Topic >> topic input for accepting the input of the boolean start signal
    - reset_Topic >> topic input for accepting the input of the boolean reset signal
    - controlOutput >> topic used to publish control signal for the start of the data
    - restartOutput >> topic used to publish reset signal for resetting the exploration
    - topicOutput >> topic for deciding the boundary drawn 
    - frequency >> frequency for publishing the boundary geometry
    - timeInterval >> time interval for displaying the message 
    - mapTopic >> map topic to subscribe on for performing auto boundary setting
    - initialPoint >> initial point for the RRT tree to grow
    - odom_topic >> odom topic for subscribing (just in case not using the initial point)
    - robot_frame >> robot frame where it publishes the Odom topic
- filter.py
    - map_topic >> map topic for subscribing to (merged map in the multi-robot scenario)
    - threshold >> the threshold for defining clearance for filtering frontier with high possibility near to an obstacle
    - info_radius >> information radius used to calculate the revenue
    - goals_topic >> topic for receiving the detected points of the RRT 
    - robot_namelist >> robot name list changed to the format of string (robot name separated by comma delimiter)
    - inv_frontier_topic >> topic for receiving the invalid frontier topic
    - startSignalTopic >> topic for controlling the start signal 
    - rateHz >> the frequency for determining the rate of computing the filter module node
    - global_costmap_topic >>  topic of the global cost map for each robot
    - local_map_topic >> local map of each robot if requires a subscription
    - bandwith_cluster >> the clustering parameter for each detected point 
    - robot_frame >> the frame of the robot for performing TF conversion
    - localMapSub >> subscribing to the local map of each robot
    - computeCycle >> the compute cycle for performing each cycle just in case the frequency doesn't work as intended
    - startSignalTopic >> the topic for determining whether to start the filter module or pause
    - resetSignalTopic >> reset the filter module to start from scratch
    - debug1 >> debugs message view for type 1 
    - debug2 >> debugs message view for type 2
- assigner.py
    - map_topic >> the map topic for determining the revenue/information gain for a given frontier
    - info_radius >> radius to determine the information value of a given frontier
    - info_multiplier >> multiplier for determining the base value of the information
    - hysteresis_radius >> radius for inflating the value if the robot is within the designated radius
    - hysteresis_gain >> the multiplier for the revenue if the robot is within the 
    - frontiers_topic >> topic for subscribing to from filter module
    - message_time_interval >> interval for controlling the displaying of the message
    - delay_after_assignment >> delay after each assignment of the frontier
    - start_delay >> delay before starting assignment - just in case the filter module is a bit slow
    - rateHz >> frequency for controlling the module publishing speed
    - inv_points_topic >> topic for publishing invalid points
    - inv_frontier_topic >> topic for publishing invalid centroids
    - time_per_meter >> anticipated time needed for robot per meter usage, else the goal will be cancelled
    - robot_namelist >> robot name list changed to the format of string (robot name separated by comma delimiter
    - invalid_distance >> If the goal is assigned too far away, it will be cancelled based on this distance threshold
    - rp_metric_distance >> the relative metric distance for calculating the relative distance between robots for adjusting the revenue for each goal
    - non_interrupt_time >> the minimum time the robot can travel before interruption can be done
    - start_delay >> the time to hold before starting the exploration
    - startSignalTopic >> topic for controlling the start signal 
    - resetSignalTopic >> topic for controlling the reset signal
    - debugFlag1 >> debugs message view for type 1 
    - debugFlag2 >>  debugs message view for type 2
    - global_frame >> frame for sending the goal in the robot class
    - plan_service >> plan services for planning the trajectory of current robot to the designated goal
    - base_link >> base link frame for sending the goal (for TF transformation)
    - move_base_service >> move base services for checking move bases statuses.



## Setting up Simulation for testing
An example of the simulation can be shown in the following video: 

[![TM-RRT Exploration for Ubuntu 18.04 ROS noetic](https://img.youtube.com/vi/F40GGvnIfsc/0.jpg)](https://www.youtube.com/watch?v=F40GGvnIfsc "TM-RRT Exploration for Ubuntu 18.04 ROS noetic")


For this work, we only compare against conventional RRT. All are welcome to contribute more to adding more benchmarking algorithms.
The link for accessing the simulation and the source files are found in the link below:
[TM-RRT Exploration Simulation](https://github.com/hikashi/TM-RRT_exploration_Simulation "TM-RRT Exploration").


## Paper / Publication
Please cite the paper if you are using/comparing our work.

        @article{lau2022multi,
          title={Multi-AGV's Temporal Memory-Based RRT Exploration in Unknown Environment},
          author={Lau, Billy Pik Lik and Ong, Brandon Jin Yang and Loh, Leonard Kin Yung and Liu, Ran and Yuen, Chau and Soh, Gim Song and Tan, U-Xuan},
          journal={IEEE Robotics and Automation Letters},
          volume={7},
          number={4},
          pages={9256--9263},
          year={2022},
          publisher={IEEE}
        }
        
        
## Issues
- Optimization of the goal assignment - Since the goal assignment of each AGV is based on the revenue calculation for a given frontier, hence, the goal assignment is quite limited and requires a lot of calculations. This can be slow and sub-optimal if the environment being explored is complex.
- Optimization of _rp_dist_ - Currently, the _rp_dist_ need to be chosen manually for a given environment. The study for adaptive value to optimize the _rp_dist_ is yet another issue to address.  
- Limitation of centralized paradigm - The centralized paradigm requires a server to allocate tasks to each AGV, which can be challenging in an infrastructure-free environment. To address this issue is not an easy task and the ideal solution would be a distributed approach, which is one of our next research milestones. 
- Heterogeneous robots- The current experiment robot for TM-RRT experiment only consist of three robots with the same specifications, the collaboration between different robot with different moving capability is something that we want to study in future.
- 3D environment - Currently, our approach is only able to support 2D environments, where 3D environments with rough terrain remain a challenge. 
