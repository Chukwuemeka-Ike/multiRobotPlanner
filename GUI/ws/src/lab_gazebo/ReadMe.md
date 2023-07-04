## To be able to find the models in this repo woth Gazebo:
Read under the title [Creating your own Gazebo ROS Package](http://gazebosim.org/tutorials?tut=ros_roslaunch)

Add the following lines to `~/.bashrc` and then source.
```
export GAZEBO_MODEL_PATH=/home/burak/common/RESEARCH/AssistiveRobot-SimulationFiles/catkin_ws_gazebo/src/lab_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=/home/burak/common/RESEARCH/AssistiveRobot-SimulationFiles/catkin_ws_gazebo/src/lab_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
```

If you also have implemented some Gazebo plugins:
```
export GAZEBO_PLUGIN_PATH=/home/burak/<path>/my_package_example/lib:${GAZEBO_PLUGIN_PATH}
```

## To export world files to a single COLLADA (.dea):
(This could be useful to visualize Gazebo simulation worlds statically on RVIZ as a marker later. Check: [https://answers.ros.org/question/217324/visualizing-gazebo-model-in-rviz/](https://answers.ros.org/question/217324/visualizing-gazebo-model-in-rviz/))


Install Ignition Gazebo for Ubuntu 18.04:
Read [https://gazebosim.org/api/gazebo/6.1/install.html](https://gazebosim.org/api/gazebo/6.1/install.html)

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
```

Then:
```
sudo apt-get install libignition-gazebo5-dev
```

To export:
Read [https://gazebosim.org/api/gazebo/5.0/collada_world_exporter.html](https://gazebosim.org/api/gazebo/5.0/collada_world_exporter.html)

Add the following lines as a child to the <world> tag in an SDF file.
```
<plugin
  filename="ignition-gazebo-collada-world-exporter-system"
  name="ignition::gazebo::systems::ColladaWorldExporter">
</plugin>
```

Note: You may also need to add some lines to `~/.bashrc` and then source similar to the following lines.
```
export SDF_PATH=~/catkin_ws_swarm2/src/AssistiveRobot-SimulationFiles/lab_gazebo/models
export IGN_FILE_PATH=~/catkin_ws_swarm2/src/AssistiveRobot-SimulationFiles/lab_gazebo/worlds
```

Run the world using
```
ign gazebo -v 4 -s -r --iterations 1 WORLD_FILE_NAME
```

A subdirectory, named after the world, has been created in the current working directory. Within this subdirectory is the mesh and materials for the world.

## To visualize .dae files in RVIZ, use RVizMeshVisualizer package:
Check [https://github.com/burakaksoy/RVizMeshVisualizer](https://github.com/burakaksoy/RVizMeshVisualizer)

