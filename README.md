# Projects

All my projects are currently private, nevertheless I present some previews of my projects.

Feel free to contact me if you want to know more. 

## Project Table

| Plugin | README |
| ------ | ------ |
| Thumb Led | Private |
| Santorini Game | Private |
| Improved NEQR Compression | Private |
| CLup | Private |
| Working Zone | Private |
| Visual Question Answering | Private |
| Crop Segmentation | Private |
| Face Mask Detector | Private |
| Robot Odometry | Private |
| Robot Localization | Private |

## Thumb Led
One week project in which ..... 

## Robot Odometry

![scout_robot_image](resources/images/scout2.png)

[Scout 2.0](https://www.agilex.ai/index/product/id/2) is an indoor and outdoor mobile platform, dedicated to the development of multiple applications in higher education, research and industry.

In this project we are given some recorded data about the robot: speed of the 4 motors, odometry provided by the manufacturer and the ground truth pose of the robot acquired with [OptiTrack](https://www.optitrack.com/applications/robotics).

Using Robot Operating System (ROS) we have pursued are the following goals:
- compute odometry using skid steering approximated kinematics;
- use dynamic reconfigure to select between integration methods (Euler/Runge-Kutta);
- write 2 services to reset the odometry to (0,0,0) or to a certain pose (x,y,θ)
- publish a custom message with odometry value and type of integration.

![scout_robot_image](resources/images/scout2.png)

## Robot Localization

[Scout 2.0](https://www.agilex.ai/index/product/id/2) is an indoor and outdoor mobile platform, dedicated to the development of multiple applications in higher education, research and industry.

In this project we are given some recorded data, such as the odometry provided by the manufacturer and other data from the following robot sensors:
- *OptiTrack* tracking system, which publishes the pose of the robot in the `/Robot_1/pose` topic;
- *SICK LMS100* lidar, which publishes the laser data in the `/scan` topic;
- *Intel T265* camera, which publishes the visual odometry in the `/camera/odom/sample` topic and the IMU data in the `/camera/accel/sample` and `/camera/gyro/sample` topics;
- *Pixhawk mini* IMU, which publishes the IMU data in the `/mavros/imu/data_raw`.

Using Robot Operating System (ROS) we have pursued are the following goals:
- create of a map from the recorded data above;
- fuse different data sources using *robot_localization* package;
- localize the robot using *amcl* package and *robot_localization* package.

![scout_robot_image](resources/images/scout2.png)

## Face Mask Detector
The task of the project was to solve a classification problem using convolutional neural networks. In particular, the task was to distinguish, given an
image, three cases:
- No person in the image is wearing a mask
- Everyone is wearing a mask
- Someone in the image is not wearing a mask

− No person in the image is wearing a mask
− Everyone is wearing a mask
− Someone in the image is not wearing a mask

## Crop Segmentation
The task of the project was to solve a segmentation problem. In particular, given a RGB image, the task was to
create a segmentation mask to distinguish between crop, weeds, and background.

## Visual Question Answering
The task of the project was to solve a visual question answering (VQA) problem. Given an image and a question,
the goal was to provide the correct answer.

## Santorini Game

Enjoy a unique gaming experience with the digital version of the tabletop game Santorini 

![scout_robot_image](resources/images/scout2.png)

## Improved NEQR Compression
Writing a paper on images compression in Quantum Computers and relative implementation.

<p align="center">
    Abstract
</p>

*This paper presents the implementation of NEQR and its compression. The aim of the paper is to highlight the practical limits of the
compression theorized so far and to propose an improvement which can be implemented grouping CNOT gates that have the same control information. At the end of the report we present some results we obtain highlighting the improvement achieved in the compression phase.*


## CLUP

Design Document (DD) and Requirement Analysis and Specification Document (RASD) of an application which help store managers to regulate the
influx of people in the building monitoring entrances to comply with the coronavirus safety regulations.
Moreover the application has the goal to prevent people from waiting outside the store safeguarding clients health and not wasting their time.


## Working Zone
...
