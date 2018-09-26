# Visual Servoing Image Based
====================

Visual Servoing IBVS for Mbot.

This package is related to Louis CHOPOT internship in Socrob@Home team. 
The report (Background, Theoritical aspect, Implementation, Results) of this work is available just send me an email: louis.chopot@sigma-clermont.fr !

=====================

***Dependencies (what to install to make it work)***

To make this package work, there are multiple dependencies:

***ViSP related***

- *ViSP* -> https://visp.inria.fr/install/
(ViSP C++ library)

- *visp_ros* -> http://wiki.ros.org/visp_ros/Tutorials/Howto_install_visp_ros
(Wrapper for visp to work with ROS), in this package we use vpROSGrabber.

**Real Sense D435 related**

- *librealsense* -> https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md 
(Driver for the camera, the software from intel realsense-viewer can be very usefull to tune camera parameters
or to see if both depth image and rgb are working or also to hardware reset the camera, in case of no depth image at all )

- *intel-ros* -> https://github.com/intel-ros/realsense : Wrapper for intel driver to work with ros.

Other dependencies:

- *darknet_ros_py* 
- *message_filters*
- *mbot_perception_msgs*
- *actionlib*

see package.xml and CMakeLists.txt for more.

======== 

Start/Input/output

This package work as an action server/ client execute callback (based on actionlib ros package, http://wiki.ros.org/actionlib/Tutorials)
The action_client_vs has to send the goal to the action_server_vs to start the node.

To start the client:
```console
rosrun mbot_visual_servoing_imagebased action_client_vs
```

To start the server (that is the actual node, where everything happened):
2 launch file, one to use the inverse kinematic cartesian controller, one to use optimization cartesian controller (emilia controller)

```console
roslaunch mbot_visual_servoing_imagebased visual_servoing_imagebased_cartesian_controller.launch
```
OR
```console
roslaunch mbot_visual_servoing_imagebased visual_servoing_imagebased_emilia_controller.launch
```
This launch files launch: the VS node (action_server_vs.cpp), the arm controller (cartesian or optimization) and YOLO.

The camera node driver also need to be launch:
```console
roslaunch realsense_camera2 rs_aligned_depth.launch
```
There are other launch file for the camera, this one gives a depth map aligned to rgb image, which is used in this node.

!!!TWO METHOD!!! (you can use blob tracking or YOLO center bounding box, you cannot use both at the same time !)
-> blob tracking and YOLO center bounding box 

INPUT
- RGB image
- (Only YOLO center bounding box) Depth Image, aligned depth to color topic.
- (Only YOLO center bounding box) Detections from YOLO
- (Only blob tracking) Click on the blob inside the display

OUTPUT
- Camera Cartesian Velocity vector in camera frame (left_arm_camera_link) by default (TF already implemented inside node,
can change this  "expression" frame easily when the ApplyVelocitiesAndPublish fct is called inside loop())
- Success bool, true if reached.

FEEDBACK
- Quadratique error

==============

Parameters than can be tuned 

In config file:
- Gain values for the vpAdaptiveGain (gain of the control law)
- Gain for the low pass filter
- Error quadratique value !!!IMPORTANT!!!, if you want more or less precision

==============

DEBUG

To debug the node:
In config file:
- Enable Display, to see the vpImage with bounding box of YOLO and current and desired features 
- Enable Plot, plot the velocities and error curves

I kept the blob tracking method, mostly for debug, because it's very simple to use doesn't need yolo. it can be used to debug.

==============

KNOWN ISSUES:
- You cannot used Blob tracking without diplay because you need to click inside the display to init !!!
- Topic names !
- Camera is losing depth image: https://github.com/IntelRealSense/librealsense/issues/1086
This can be solved by upluging and re plug the camera, if still not working hardware reset is needed, It can be solved permanently but I din't had time (see issue)
- Using Optimization Cartesian Controller work in simulation but not inside the real robot, some investigations has to be made.
- ROSGrabber master URI, use setMasterURI with the value you want (solved)



