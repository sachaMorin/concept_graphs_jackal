# ConceptGraphs: Jackal Mapping and Navigation
I'm releasing this code after a few people expressed interest in the [ConceptGraphs Jackal experiments](https://concept-graphs.github.io/). The codebase is far
from polished and is somewhat specific to our setup. As such, it should be considered as a rough starting point for your own experiments.

## Intro
A set of ROS nodes and launch files to map and navigate the Jackal using
open3d-slam and move_base. **Look at the other README for more details on ConceptGraphs.**

You can tweak all the
move_base and open-3d-slam parameters in the `params` folder.

## Install

There was a fair amount of setup already done on our robot before I started using it. As such, I only provide rough installation guidelines here. 
Some dependencies might be missing.

To use this code, you need a Jackal robot with a Velodyne VLP-16 Lidar and ROS Noetic. You should also set up 
[Open3D SLAM](https://open3d-slam.readthedocs.io/en/latest/).

Some packages might be missing on your Jackal (like [global_planner](http://wiki.ros.org/global_planner)), but they
are straightforward to install, e.g.

```shell
sudo apt get install ros-noetic-global-planner
```
Specifically for global_planner, there were issues with the goal tolerance in my setup. You may want to consider
building this [commit](https://github.com/wyca-robotics/navigation/tree/81f71d1e6fc77fdde996d0ecfb06d457cbb6b94b) from source to fix this.

## Booting and Troubleshooting
This section is specific to our robot. Feel free to skip it.

When you boot the jackal, you may want to run
```shell
rostopic echo velodyne_points
```
and then

```shell
rostopic echo /camera/color/image_raw/compressed
```
to make sure the velodyne and front camera are publishing data. If not, simply reboot the robot.

Also make sure to source the workspace with this package. Here's the command as of
Aug 2023, but this may change if you reinstall stuff.
```shell
source $YOUR_WORKSPACE/devel/setup.bash 
```

If you need to install stuff on the robot, but do not have internet access, this is likely
a problem with ip routes. Running the following command should fix it:
```shell
sudo netplan apply
```
otherwise you may need to to run 
```shell
sudo ip route
```
and manually remove some routes (I don't remember how exactly).

## map.launch
Run this if you want to build a 3D point cloud of your environment with open3d-slam. 

On the robot, you should launch
```shell
roslaunch real_nav map.launch
```

then on your laptop, you should launch
```shell
roscd real_nav/rviz; rviz -d map.rviz
```
Slowly drive the robot around. When you are
happy with the map, you can save it by running
```shell
roscd real_nav/data/maps; rosservice call /mapping_node/save_map; rosrun map_server map_saver map:=/costmap_node/costmap/costmap -f map
```
this should save 
* The 3D point cloud as a pcd file;
* The 2D costmap as a pgm file and a yaml file.
 
## nav.launch
Now you can navigate your map by launching the following on the robot
```shell
roslaunch real_nav nav.launch
```

then on your laptop, you should launch
```shell
roscd real_nav/rviz; rviz -d nav.rviz
```

You now need to use the `2D Pose Estimate` tool to approximately localize the robot in 
the map. open3d-slam should then localize the robot accurately. 

After localizing, it might be advisable to clear costmaps if some odd points got added.
```shell
rosservice call /move_base/clear_costmaps
```

You can then use the `2D Nav Goal` tool to send the robot to a goal. This stack runs a
slightly customized version of move_base. You can publish goals to two custom topics:
* `/move_base_simple/rotate_goal`: The robot will first rotate towards the optimal path, then start navigation. Somehow this hack helps a lot with the current navigation stack.
* `/move_base_simple/look_goal`: Same rotation behavior, but in addition the robot will run a small algorithm to a find a free pose to look at the goal instead of reaching it. Handy to look at objects.

You can publish to those custom topics in rviz by looking at the properties of the `2D Nav Goal` tool.

The algorithm used to find free space in `/move_base_simple/look_goal` is extremely naive and can be improved. It doesn't check
if the found pose is actually reachable and may output something behind a wall for instance. This proved to be only a minor issue
for our experiments and can be addressed by manually editing the map pgm file to block problematic areas.


## align.launch
This file can be used to register a point cloud against the Jackal's lidar map (e.g., the ConceptGraphs point cloud). The point cloud needs
to be in the pcd file format.
1. Make sure that that `params.map_initializer.pcd_file_path ` in `params/o3d/align.lua` points to the lidar map you want to use.
2. Update the pcd file path of the point cloud you want to register in `launch/align.launch`. You should change the `file_name` parameter of the `pcd_to_pointcloud` node.
3. Open 3 terminals and run the following commands:
    1. `roslaunch real_nav align.launch`. This step can throw warnings for 5-6 seconds. Be patient.
    2. `roscd real_nav/rviz; rviz -d align.rviz`
    3. `rosrun tf tf_echo map_o3d velodyne`
4. Use the 2D pose estimate rviz tool to move the pointcloud around until registered with the base map to your satisfaction.
5. Look at the tf_echo terminal to obtain the transform. Update the `map_o3d_to_map_cf` static transform publisher in `launch/include/cf_tools.launch`. Watch out, the order of the arguments is `x y z yaw pitch roll`.
