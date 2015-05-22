#Kinematic Calibration

## General
Kinematic Calibration is an open source calibration framework for humanoid robots. Currently, is is specifically designed for the Nao humanoid by Aldebaran. 

The framework estimates the camera's extrinsic and intrinsic parameters, as well as the joint offsets of the humanoid. To this end, artifical markers (e.g. checkerboards) need to be attached to the robot's end-effectors, which are then observed by the camera. 


Please note: This code is experimental and not a stable version. Parts of the functionallity might be missing or are broken. You are invited to contribute!


## Background
The approach is described in details in the paper:
[Daniel Maier, Stefan Wrobel, Maren Bennewitz, Whole-Body Self-Calibration via Graph-Optimization and Automatic Configuration Selection, IEEE International Conference on Robotics and Automation (ICRA), 2015]( http://www2.informatik.uni-freiburg.de/~maierd/pub/maier15icra.pdf)


## Requirements

The software requires the following software:

* ROS
* hrl_kinematics (https://github.com/ahornung/hrl_kinematics.git)
* libflann
* PCL
* opencv
* g2o (only commit 96c5a7 tested)
* libgsl
* nao_robot
* opencv
* aruco (e.g. from ar_sys in ros-indigo)

* catkin
* doxygen


## Compilation
As this is a catkin package, simply add it to your workspace and compile with catkin. 


## Documentation
The documention can be created with doxygen. 


## Running

### Preparation
First of all, the framework requires that the initial marker placement is approximately known from the ROS param server. So the urdf model needs to be adjusted accordingly. The same goes for the camera calibration. 
The nao_description package (https://github.com/danielmaier/nao_description) simplifies this process as it is parameterized for this purpose and only the calibration_xxx.xacro files in the urdf directoy need to be updated. Note, this version is not the "official" nao_description package. It should be possible to merge the changes at one point. In general, the code has only been tested with the this nao_description package and not the official one which has different link and joint names. 


Second, a list of possible poses is required. The poses should be such that the marker is visible in the image. Such a list can be created manually (cumbersome) or using an initial calibration and the poseSampling node. It might be necessary to make some modifications in order to run that node. An auto-generated sample list is contained in the config folder (e.g. poses_larm_750.yaml) for the marker placement as illustrated in the paper. So if your markers are approximately at the same position, it should work out of the box. 

Third, there are some implicit assumptions made, e.g. that you are trying to calibrate the lower of Nao's two cameras and its TF frame name is CameraBottom_frame. This assumption has been lifted in some parts of the code and the camera frame is now configurable but maybe other parts of the code still rely on this assumption. The same goes for other link and joint names. 


### Configuration:
The main config file is nao_calibration.yaml.
For each kinematic chain, there is an additional config file params_$CHAINNAME_general.yaml  that allows to change the name of the frames (e.g. Marker frame, Head frame, etc.), marker type (checkerboard, aruco), and some marker specific settings (dimensions)
Finally, there is params_$CHAINNAME_capturing.yaml that contains capturing parameters (e.g. head step when searching for a marker) 



### Prerequisites:
For convenience, the following launch file should bring up all the prerequisites that are needed for the calibration:
roslaunch kinematic_calibration nao_basic.launch

In details, the calibration software needs the robot description published to the parameter server (nao_description)

Furtermore, it requires (at least) the following services:
joint_stiffness_trajectory (nao_controller)
joint_trajectory  (nao_controller)
body_pose (nao_pose)

Also, these topics need to be published:
camera/image_raw (nao_sensors)
joint_states (nao_driver)


### Running the actual calibration:
rosrun kinematic_calibration upateNode
roslaunch kinematic_calibration dataCaptureService.launch
roslaunch kinematic_calibration calibrate_nao.launch

calibrate_nao calibrates both arms of the robot. Start with that. To include the legs, adjust the launch file accordingly, as well as the config file (nao_calibration.yaml)


## After calibration:
The updateNode writes a set of files (as sepecified in the nao_calibration.yaml file) containing the camera calibration, an updated robot model and individual joint_offset, camera transform, and marker transform files.  The latter can be used in conjunction with the modified version of nao_description (https://github.com/danielmaier/nao_description).


