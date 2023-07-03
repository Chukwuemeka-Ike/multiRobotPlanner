# GUI

[Swarm Project Version 1](https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics/tree/main)

[Why PyQt?](https://www.pythonguis.com/faq/pyqt-vs-tkinter/#tkinter-vs-pyqt-a-feature-comparison)
[PyQt Calculator Tutorial](https://realpython.com/python-pyqt-gui-calculator/)
[PyQt Layout](https://realpython.com/python-pyqt-layout/)

PyQt6 needs a commercial license for commercial applications.

## Installing PyQt
```bash
pip3 install pyqt5
```
Couldn't use PyQt6 yet because of some conflicts with sip.


RViz
```bash
sudo apt install ros-noetic-visualization-tutorials
```

[RViz](http://docs.ros.org/en/melodic/api/rviz_python_tutorial/html/index.html)
[RViz Python Tutorial Github](https://github.com/ros-visualization/visualization_tutorials/blob/noetic-devel/rviz_python_tutorial/myviz.py)

Plt in PyQt5
https://www.geeksforgeeks.org/how-to-embed-matplotlib-graph-in-pyqt5/#

RVizMeshVisualizer
Changed object_visualizer.py to get the mesh visualization path parameter instead of always using the *meshes* folder in the *object_visualizer* package.

Changed 
```python
visualizer_path = os.path.join(rp.get_path('object_visualizer'), 'meshes')
```
to 
```python
visualizer_path = rospy.get_param('~visualizer_path')
```

Line 37 - 
```python
marker.mesh_resource = 'package://object_visualizer/meshes/' + file
```
to
```python
marker.mesh_resource = 'file://' + os.path.join(visualizer_path, file)
```