<a id="top"/> 
# squirrel_arm

This repo is home to a simple console based node to move the SQUIRREL's arm and base axis via keyboard input.

Technical Maintainer: [bajo](https://github.com/bajo (Markus "Bajo" Bajones, TU Wien) - markus.bajones@gmail.com

##Contents

1. <a href="#1--installation-requirements">Installation Requirements</a>
2. <a href="#2--execution">Execution</a>
3. <a href="#3--software-architecture">Software architecture</a>


## 1. Installation Requirements: <a id="1--installation-requirements"/> 

####system packages
This node depends on the readchar python module. It will be moved into the 
rosdep system eventually. Until then it can be installed as follows:

```
sudo pip install readchar
```

####ROS packages
The ROS packages dependencies can be installed with the command:
```
rosdep install --from-path squirrel_arm -i -y
```
## 2. Execution: <a id="2--execution"/> 
```
roslaunch uibk_arm_controller controller.launch 2> /dev/null
rosrun squirrel_arm tuw_incremental_arm_movement.py
```

# 2.1 Usage

1. Select the axis to rotate (0: base platform, 1-5: axis counted from bottom to top).
2. Move the axis with '+' or '-' keys.
3. Press 'q' or 'Q' to quit.


## 3. Software architecture <a id="3--software-architecture"/> 

tuw_incremental_arm_movement![tuw_incremental_arm_movement](tuw_incremental_arm_movement.png "Architecture tuw_incremental_arm_movement")

<a href="#top">top</a>
