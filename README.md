# MLMapping(Multilayer Mapping Kit)-Embedded version

### Use it with a planner
You can acquire the information of one global position in the world from the map data structure directly with this **Embedded version**. Just build this package first in your workspace, include the header file ````#include <mlmap.h>```` in your project, and use the interface functions. The initialization only need the ros nodehandle, and it receives the ros messages automatically to build the map. Check nodelet_map.cpp for reference.

Here is a brief example for initialization:

````
#include <mlmap.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
int main(int argc, char **argv)
{
  ros::NodeHandle nh("~");
  ros::init(argc, argv, "traj_planner");

  mlmap::Ptr mapPtr;
  mapPtr.reset(new mlmap);
  mapPtr->init_map(nh);
  while (!mapPtr->has_data)  //check if the map is build with data input
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

//write your code here to use the pointer of mlmap, say mapPtr.

mapPtr->getOccupancy(Eigen::Vector3d(1.0, 2.0, 3.0));

}
````
Don't forget to set the parameter of "/mlmapping_configfile", for example, include the code like this in your planner launch file:

````
<param name="/mlmapping_configfile" type="string" value="$(find mlmapping)/launch/config/config_sim.yaml"/>
````
 It is very fast, and most map update steps can be finished **within 2 ms**.

**Interface functions** 

Set the status of one piece of the entire map into free (input the min and max corner coordinate of the region):

````

mlmap::setFree_map_in_bound(Eigen::Vector3d box_min, Eigen::Vector3d box_max);
````
Get the status of the queried position (return -1 for unknown, 1 for free, 0 for occupied):
````
int mlmap::getOccupancy(Eigen::Vector3d pos_w);
````
Or query if a position is occupied in inflated occupancy map (return -1 for unknown, 0 for occupied):
````
int mlmap::getInflateOccupancy(Eigen::Vector3d pos_w);
````

You can also check the occupancy with a safety radius Sr, if any cell within Sr to the queried position is occupied it returns 0.

````
int mlmap::getInflateOccupancy(Eigen::Vector3d pos_w, float Sr);

int mlmap::getOccupancy(Eigen::Vector3d pos_w, float Sr);
````

Get the odds of 'occupied' of the queried position:
````
float mlmap::getOdd(Eigen::Vector3d pos_w);
````

Get the gradient of the odds of the queried position (the gradient is a vector heading to the neighboring cell of the lowest odds of occupancy):
````
Eigen::Vector3d mlmap::getOddGrad(Eigen::Vector3d pos_w);

````

The latest demo (red dots are the frontiers of the un-explored space, write for exploration task):

<img src="others/mapping_new.gif" width="800">

The odds map (red for low odds and blue for high odds) and the odds-gradient of those occupied cell (odds > 0.8), represented by the white lines:

<img src="others/oddsgrad.gif" width="800">


### Introduction
**MLMapping** is a multilayer mapping framework designed for autonomous UAV navigation applications. In this framework, we divided the map into three layers: awareness, local, and global. The awareness map is constructed on the cylindrical coordinate, which enables fast raycasting. The local map is a probability-based volumetric map. The global map adopts dynamic memory management, allocating memory for the active mapping area, and recycling the memory from the inactive mapping area. The framework supports different kinds of map outputs for the global or local path planners. 
**This version** adopts a hybrid data structure to achieve dynamic memory management and merge the local and global layers into one layer. In this new mapping toolkit, we can query the occupancy of any given global position quickly and the mapping area is not limited by the pre-allocated RAM (that is what happened in the original local map layer). Also, the occupancy odds are consistent in the whole mapping ragion and can be queried quickly, too. **Please note that ESDF function is not needed anymore** since we hope odds gradient can replace the distance gradient in trajectory optimization.

### Videos (original MLmapping):
<a href="https://www.youtube.com/embed/kBLQzIB_kWo" target="_blank"><img src="http://img.youtube.com/vi/kBLQzIB_kWo/0.jpg" 
alt="cla" width="480" height="300" border="1" /></a>

| Fast Raycasting    | Large Scale Mapping   | Autonomous UAV Navigation  |
| ---------------------- | ---------------------- |---------------------- |
| <img src="others/exp1.gif" width="250">  | <img src="others/exp2.gif" width="250">  | <img src="others/exp3.gif" width="250">  |

### Publications
[Chen, S., Chen, H., Chang, C. W., & Wen, C. Y. (2021). Multilayer Mapping Kit for Autonomous UAV Navigation. IEEE Access.](https://ieeexplore.ieee.org/abstract/document/9336584)
### Compile
Clone this repository to catkin src folder say: ~/catkin_ws/src
````
cd ~/catkin_ws/src
git clone https://github.com/PAIR-Lab/MLMapping.git
````
Install 3rd Part library
````
cd catkin_ws/src/MLMapping/3rdPartLib/
./install3rdPartLib.sh
````
Compile
````
cd ~/catkin_ws/
catkin_make
````

### Verify Using Provided Dataset
Download the [Large Scale Mapping Dataset](https://connectpolyu-my.sharepoint.com/:u:/g/personal/17903070r_connect_polyu_hk/EYGhc0ijYl9Muq33mUSCgxABZgNBJrTQPp34SY65gWoXRA?e=8lkjxb) into the bag folder <br />
decompress the rosbag
````
rosbag decompress corridor.bag
````
run (modify the path of the ROS bag first in the launch file)
````
roslaunch mlmapping mlmapping_bag_sim2.launch
````

### Maintainer
Han Chen(Dept.AAE,PolyU): stark.chen@connect.polyu.hk 

### MLmapping's author
Shengyang Chen(Huawei Technologies Co., Ltd): shengyang.chen@connect.polyu.hk

<br />

