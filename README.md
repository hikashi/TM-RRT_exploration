# Temporal Memory-based RRT Exploration
This is a temporal memory-based rapidly-exploring random tree (TM-RRT) algorithm, which is based on the original RRT exploration by Hassan Umari.

credit to hasauino for creating the RRT exploration packages.
[RRT Exploration package](https://github.com/hasauino/rrt_exploration "RRT Exploration").
[RRT Exploration Tutorial package](https://github.com/hasauino/rrt_exploration_tutorials "RRT Exploration").

There are a few modifications done in order to improve the efficiency of exploration, which are mainly focus on assigner module.
- memory-based assigner, where assigner keep tracks of the goal history and preventing other robots from exploring same location
- time limit imposed to the goal based on the expected velocity to prevent robot from spending time on unreachable goal.


## Requirements
The following code is exectuted in ROS Melodic in Ubuntu 18.04 LTS, Python 2.7

The following libraries are required to install before proceeding to run the code

    $ sudo apt-get install ros-melodic-gmapping
    $ sudo apt-get install ros-melodic-navigation
    $ sudo apt-get install python-opencv
    $ sudo apt-get install python-numpy
    $ sudo apt-get install python-scikits-learn
    
## Installation Process
create a new folder called "catkin_explore/src" by executing the following comment:

    $ sudo mkdir -p ~/catkin_explore/src
    $ cd ~/catkin_explore/src/
    $ git clone https://github.com/hikashi/TM-RRT_exploration.git
    $ cd ~/catkin_explore
    $ catkin_make

## TF Tree Requirement
constructing....

## Demostration of TM-RRT in Clearpath's Jackal
Kindly refer to the video below for demostration of TM-RRT Exploration:
constructing...




## ROS PARAMETER
constructing...


## Setting up Simulation for testing
Constructing...

## Paper / Publication
Please cite the paper if you are using / comparing our work.

    @Test {test,
      title = {Multi-AGV's Temporal Memory-based RRT Exploration in Unknown Environment},
      author = {{Billy~Pik~Lik~Lau, Brandon~Jin~Yang~Ong, Leonard~Kin~Yung~Loh, Ran~Liu, Chau~Yuen, Gim~Song~Soh, and U-Xuan~Tan}},
      organization = {},
      address = {},
      year = {2022},
    }

## Issues
- Optimization of the goal assignment - Since the goal assignment of each AGV is based on the revenue calculation for a given frontier, hence, the goal assignment is quite limited and requires a lot calculations. This can be slow and sub-optimal if the environment being explored is complex.
- Optimization of _rp_dist_ - Currently, the _rp_dist_ need to be chosen manually for a given environment. The study for adaptive value to optimize the _rp_dist_ is yet another issue to address.  
- Limitation of centralized paradigm - The centralized paradigm requires a server to allocate task to each AGV, which can be challenging in an infrastructure-free environment. To address this issue is not an easy task and ideal solution would be distributed approach, which is one of our next research milestone. 
- heterogenous robots- The current experiment robot for TM-RRT experiment only consist of three robot with same specifications, the collaboration between different robot with different moving capability is something that we to further study.
- 3D environment - Currently, our approach only able to support 2D environment, where 3D environment with rough terrain remain a challenge. 
