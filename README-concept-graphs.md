# ConceptGraphs
A quick tour of how to run ConceptGraph experiments, i.e., go look at the most relevant object
given a user query and potentially find another location if the object is missing.

## Install

For `ConceptGraphTools.py` you need to install the dependencies of the [main ConceptGraphs repo](https://github.com/concept-graphs/concept-graphs). I found it easier to use a venv instead of conda since it interacts better with ROS nodes. **You will need to update the shebangs in some files to point to your virtual environment.**

## Overview
* `nav.launch`: Launch the nav stack on the robot.
* `ConceptGraphTools.py`: Launch Concept Graph services on the desktop.
* Main relevant tf Frames:
  * `map`: The Jackal map frame (where base_link started).
  * `map_o3d`: The Jackal Lidar map frame (where the velodyne started).
  * `map_cf`: ConceptGraphs map frame. The ConceptGraphs map frame is registered to the main map using the `align.launch` script.
  * `object_location`: The frame of the object of interest as identified by the ConceptGraph `query_goal` service given the user query.
  * `odom`: Odom frame.
  * `base_link`: Robot frame.

## Abstract Navigation Queries
**Note: Models were hosted off-board on a separate GPU workstation named OmegaDuck.**

First you need to have mapped the environment with the Jackal (see the main README).  Then you
should map the place with ConceptGraphs and register the point cloud against the Jackal's map. Have a look
at `align.launch` in the main README for more details. We used an Azure Kinect for our ConceptGraphs map.

For deployment, you need to run the following on the robot
```shell
roslaunch real_nav nav.launch
```
Then launch rviz on OmegaDuck
```shell
roscd real_nav/rviz; rviz -d nav.rviz
```
then make sure to localize the robot.

Now we need to run the big models. On an OmegaDuck terminal, run
```shell
roscd real_nav/src/cf_tools/; ./ConceptGraphTools.py  -p $PATH_TO_SCENE_GRAPH -o llava_full_image
```
The `o` flag allows you to define an object detection strategy. The default is `llava_full_image`. You can
also use a combination of SAM and CLIP instead using the `sam_clip` option.

ConceptGraphTools will spin up some useful services: `query_goal` and `do_i_see`. You can call
those independently in a terminal without moving the robot. Useful for debugging and experimenting without running full missions.


We are now ready to run queries and missions! For a text query, you should go to
```shell
roscd real_nav/src/mission_planner
```
and from here you can launch object missions. For a simple text query mission, run
```shell
python3 object_mission.py -q "footwear to use for my ronald macdonald outfit"
```
For visual queries (where the image seen by the robot is considered as a context for the query), try
```shell
python3 object_mission.py -q "something this guy would play with" -v True
```
Note that the latter type of query requires having llava running on OmegaDuck. Currently, this is only possible
when the object detector us running the `llava_full_image` option.

### query_goal notes

`query_goal` does not directly publish the relevant object transform. It publishes a float message to
the `/cf_object_location` topic. Then the `cfslam_pub` node (running on the robot) reads the float message
and republishes in the tf tree as a `map_cf -> object_location` transform. This hack was necessary to solve
time syncing issues between the robot and OmegaDuck.

## Traversability
We will not release the traversability code at this time, since it's spread across multiple repos and hard to consolidate.
At a high-level, you need to
1. Iterate over the objects in the scene graph and ask GPT if they are safe to traverse. See the paper for the prompt.
2. Build a point cloud with the non-traversable objects and broadcast it with ROS.
3. Add the point cloud to the costmap to allow planning around non-traversable objects with the navstack.
