# Projects

Most of my projects are currently private, nevertheless I present some previews of them.

Feel free to contact me if you want to know more. 

## Project Table

| Project | Link |
| ------ | ------ |
| [Design and Development of a Mobile Manipulator System for Industrial Tasks](#design-and-development-of-a-mobile-manipulator-system-for-industrial-tasks)| Private |
| [Deep RL based Quadcopter Stabilization](#deep-rl-based-robust-quadcopter-stabilization-system)| [QuadcopterStabilization](https://github.com/lap98/RL-Drone-Stabilization)|
| [P-Hug](#P-Hug)| [P-Hug](https://github.com/AlessandroBarbiero/Robotics-and-Design-project)|
| [iTasteGrill](#itastegrill)| Private |
| [Magic Mirror - Smart Forest](#smart-forest)| Private |
| [Missile Simulator](#Missile-Simulator)| Private |
| [Santorini Game](#SantoriniGame)| Private |
| [Improved NEQR Compression](#ImprovedNEQRCompression)| Private |
| [CLup](#CLup) | Private |
| [Working Zone](#WorkingZone) | Private |
| [Visual Question Answering](#VisualQuestionAnswering) | Private |
| [Crop Segmentation](#CropSegmentation) | Private |
| [Face Mask Detector](#FaceMaskDetector) | Private |
| [Simulator and Trajectory controller of a mobile robot](#simulator-and-trajectory-controller-of-a-mobile-robot) | Private |
| [Robot Odometry](#RobotOdometry) | Private |
| [Robot Localization](#RobotLocalization) | Private |
| [Thumb Led](#ThumbLed)| Private |

## Design and Development of a Mobile Manipulator System for Industrial Tasks

In many industrial scenarios, monitoring critical parameters is an essential task. A specific task of this type is the inspection of pressure gauges. Traditional methods of gauge inspections demand human time effort. Therefore, the system’s goal is to use a mobile manipulator to autonomously navigate industrial environments, reach designated inspection stations, and read gauge values. The system has to respond dynamically to anomalous readings by pressing an emergency button. During gauge inspections, the manipulator has to autonomously explore the environment and detect gauges or markers with a high level of accuracy. On the other hand, button press operations need a precise estimation of the position and orientation of the button to enable the interaction between the manipulator and the button itself. For both inspections and button interaction, the system has to consider the potential presence of obstacles in the environment. 

To perform these tasks, a complex software architecture has to be implemented that integrates localization, mapping, autonomous navigation, perception, and autonomous manipulation. This project uniquely addresses the challenges of autonomous manipulator inspection and manipulator interaction. While the primary focus is on gauge inspections and button operations, the study explores a generalized methodology that can be used for different types of objects and in various scenarios, promoting adaptability and scalability for a wide range of applications.

The first phase of developing an efficient mobile manipulator system involves object detection. The system has to be able to recognize and classify objects for subsequent
decision-making processes. 

The second phase involves the estimation of the object pose in three-dimensional space.
This is particularly important for object interaction where the manipulator has to approach the object in a specific position and with a specific orientation. The decision of how to implement a pose estimation algorithm and what sensors to use is central to achieving an accurate estimation. 

The third phase involves the physical movement of the manipulator end-effector towards the target object. To execute this phase, a planner has a fundamental role in computing the inverse kinematics and planning the trajectory. During trajectory execution, especially in button interaction scenarios, a feedback control mechanism is essential to regulate the manipulator approach. Furthermore, the system has to detect static or dynamic obstacles
and avoid potential collisions using depth sensors like cameras and lidars. 

Another important aspect is estimating a feasible position and orientation of the mobile base when the manipulator cannot interact with the target object due to some physical limitations. This project is carried out in collaboration with Edison, an Italian energy provider company. Edison participates in this project providing a practical industrial perspective, allowing to validate the model efficacy in actual industrial scenarios. The project aims to contribute not only to academic research but also to real-world scenarios.


<img src="Images/scout.jpeg" />



<img src="Images/object_manipulator.png" />


https://github.com/lap98/Projects/assets/47597693/a111c7c9-9bde-40dd-8f74-b0af8f3dd463



## Deep RL based Robust Quadcopter Stabilization System
Reinforcement learning has been steadily growing in
popularity for few years. Notable applications are in the
fields of finance, healthcare, natural language processing
and, especially, robotics. In particular, deep reinforcement
learning excels in the automatic development of control systems, with the advantage — compared to classical techniques — of being able to model non-linear dynamics and
kinematics.
The goal of this work is to explore and test the applications
of deep reinforcement learning in the autonomous control
systems field. Specifically, the project aim is to develop a
deep-RL based controller for quadcopters. Different RL
agents are developed and evaluated leveraging both onpolicy and off-policy learning, within a simulated environment. Our work focuses on off-policy learning, mainly due
to the much better sample-efficiency of the process. Multiple
actor-critic network structures are evaluated. The agents
are trained to stabilize the quadcopter given its state in the
world, by moving and keeping it in the defined target position.

### Contribution

- New interpretation of the actor network output (scaled delta thrust), which allows for finer-grained control
and makes the solution a drop-in component compatible with most drone once calibrated.
- New networks architecture, which strikes a balance between learning speed and representational capability.
- New controller, resistant to up to 5x stronger wind compared to previous works, and to 2x stronger wind compared to most commercial solutions.
- Completely off-policy agent training: the process is
more sample-efficient than on-policy learning algorithms, and reaches good reward values with as low
as 200000 steps. Longer training further increases the reward. No particular exploration policies are needed to converge.
- Robustness to extreme perturbations in initial pitch,
roll and yaw angles. The controller can even recover
from upside down states.
- Moreover, the neural network controller can sustain high deltas between the initial pose and the set pose (or can be used to control the drone at a lower level).

https://user-images.githubusercontent.com/47597693/193463032-2e3a48af-8b87-477e-b54e-fdce0e1ddf5e.mp4


 https://user-images.githubusercontent.com/47597693/193461113-a568ace6-0e77-424e-9d6b-f5acefe807e3.mp4

 
https://user-images.githubusercontent.com/47597693/193462668-4d48821e-8783-4eb6-b561-bdb7a02fd160.mp4

## Smart Forest

Smart Forest is a collaborative project developed by Politecnico di Milano and the energy provider Edison. The project aims to create an interactive mirror that helps users to easily monitor their energy consumption and encourages them to use renewable energy sources. The mirror displays an interactive forest, where users can earn leaves for using renewable energy and lose experience points, which is the lifespan of each tree, for using non-renewable energy. These leaves can then be used to purchase and plant new trees in order to gain more and more leaves. The mirror also provides real-time feedback and personalized recommendations for reducing energy consumption. Overall, Smart Forest project hopes to engage users in an interactive and fun way to promote sustainable energy practices and raise awareness about the importance of renewable energy.

https://user-images.githubusercontent.com/47597693/218736048-5434f455-2131-40be-b1f7-8a42c1cd1b65.mp4

## PHUG 
According to surveys and research the haptic sense is a significant aspect of human wellbeing. The experience received as a result of contact with another living creature like hugs stimulates serotonin production. High serotonin levels improve mood, make people feel better, and help having better sleep. Nowadays people are more linked than ever before as a result of the developing trend of technology. While technology has helped to bridge the gap, it has not been able to eliminate all the barriers imposed by distance. Our initiative primarily aims to bridge this gap by creating a product that sends a haptic hug from one device (Giffy: the puppet) to another one (the Jacket). The embrace and caress from the puppet is transfered via a wifi connection to the wearer of the jacket. The wearer can feel three major sensations: vibration achieved by vibration motors simulating a moving caress, pressure from an inflatable chamber simulating a hug, and a constant warm feeling from a resistive wire. Our goal is not to replace an actual embrace, but to help people that live apart and cannot hold their loved ones having a fraction of that so desired contact.

Giffy built around an `Arduino Mkr1000 wifi`

Input:
- 3 Force sensitive resistors to simulate the caress
- Force sensitive resistor to simulate the hug
- Accelerometer to detect shaking
- NFC reader to upload SSID and Password using the Application

Output:
- 2 Servo motors to move the ears
- Speaker
- Integrated Wifi module (ATSAMW25) of Arduino Mkr1000 wifi

The Jacket Built around an `Adafruit Feather Huzzah`

Input:
- Integrated Wifi module (ESP8266) of Adafruit Feather Huzzah 
- Bluetooth module (HC-05) to upload SSID and Password

Output:
- 6 Vibration motors to simulate the caress
- Air pump to inflate the chamber around the waist to simulate the hug
- Heating resistance activated via a relè

All the code has been written following the object oriented programming model with the help of the platform `PlatformIO`.
The main idea is to build a client-server model to receive the information from Giffy and send it to the jacket that operates according to that.

<img src="Images/P'hug.png" />

<img src="Images/giffy.png" />

<img src="Images/Client_wiring.png" />

<img src="Images/jacket.png" />

<img src="Images/Server_wiring.png" />

<img src="Images/Team Picture.jpg" width="600" /> 


## iTasteGrill 
iTasteGrill is a mobile application (suitable for smartphones and tablets). It can run on different operational systems such as Android and iOS.
We designed this application with the final purpose to allow people to meet each other in fairly cheerful barbecue events and sinplify the management and the organization of the barbecue party.
According to the loaded profile of whoever registered and his preferences on foods and drinks, this App can easily select products suitable to meet users’ expectations while avoiding food waste.
In addition, all users can easily have advance information on the location, date, time and number of the participants, foods and drinks expected to be found on each event.


<img src="Images/iTasteGrill.jpg" />


## Simulator and Trajectory controller of a mobile robot

In this project I implemented two main ROS packages:

<ul>
  <li>A ROS package based on Boost Odeint library to simulate the kynematic and dynamic model of a robot.</li>
  <li>A ROS package which implement a trajectory tracking controller composed of a feedback linearisation law, based on the bicycle kinematic model and a PI trajectory tracking controller with velocity feed-forward (using Forward Euler discretisation).</li>
</ul>

In the figure below is represented the reference trajectory (in blue) and the actual trajectory (in orange) of the robot

<img src="Images/ref_actual_traj.png" />

## Robot Odometry 

[Scout 2.0](https://www.agilex.ai/index/product/id/2) is an indoor and outdoor mobile platform, dedicated to the development of multiple applications in higher education, research and industry.

In this project we are given some recorded data about the robot: speed of the 4 motors, odometry provided by the manufacturer and the ground truth pose of the robot acquired with [OptiTrack](https://www.optitrack.com/applications/robotics).

Using Robot Operating System (ROS) we have pursued are the following goals:
- compute odometry using skid steering approximated kinematics;
- use dynamic reconfigure to select between integration methods (Euler/Runge-Kutta);
- write 2 services to reset the odometry to (0,0,0) or to a certain pose (x,y,θ)
- publish a custom message with odometry value and type of integration.

![odom_to_baselink](Images/odom_to_baselink.png)

![odom_gtpose_comparison.png](Images/odom_gtpose_comparison.png)

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

![map_creation](Images/map_creation.png)

## Missile Simulator 

The goal of the project is to design and implement a Vulkan application  of a missile simulator following a parabolic trajectory in a 3D landscape.

Here a table with the main functionalities of the application:

| Command      | Action          
| ------------- |:-------------------:| 
| 1			| Increase the pitch					| 
| 2			| Decrease the pitch					|
| 3			| increase the yaw						| 
| 4			| decrease the yaw						|
| 5			| move the rocket up					|
| 6			| move the rocket down					|
| 7			| move the rocket right					|
| 8			| move the rocket left					|
| 9			| move the rocket forward				|
| 0			| move the rocket backward				|
| H			| increase the acceleration				|
| B			| decrease the acceleration				|
| SPACE		| launch the rocket						|
| G			| Reset the rocket						|
| N			| Switch from day to night				|
| V			| change camera view					|
| I			| move to rocket information view		|

In the World View also those commands are possible:

| Command      | Action          
| ------------- |:-------------------:				| 
| UP ARROW		| Look up							| 
| DOWN ARROW	| Look down							|
| LEFT ARROW	| Look left							| 
| RIGHT ARROW	| Look right						|
| W				| move forward						|
| S				| move backward						|
| A				| move left							|	
| D				| move right						|
| R				| levitate up						|
| F				| levitate down						|
| Q				| turn the camera counterclockwise	|
| E				| turn the camera clockwise			|

<img src="Images/rocketSimulator2.png" width="600" /> 

## Face Mask Detector 
The task of the project was to solve a classification problem using convolutional neural networks. In particular, the task was to distinguish, given an
image, three cases:
- No person in the image is wearing a mask
- Everyone is wearing a mask
- Someone in the image is not wearing a mask

![mask](Images/mask.png)

## Crop Segmentation 
The task of the project was to solve a segmentation problem. In particular, given a RGB image, the task was to
create a segmentation mask to distinguish between crop, weeds, and background.

![crop1](Images/crop1.jpg)    

![crop2](Images/crop2.png) 
    
## Visual Question Answering 
The task of the project was to solve a visual question answering (VQA) problem. Given an image and a question,
the goal was to provide the correct answer.

![VQA](Images/VQA.png) 

## Santorini Game 

Introducing "Santorini: Digital Odyssey" - an enticing digital adaptation of the revered tabletop game, Santorini! Immerse yourself in a realm where ancient mythology converges with technology, now available for engaging online multiplayer gameplay.

Embark on a strategic journey where you assume the mantle of a Greek deity, guiding your devoted followers in a fervent race to construct majestic towers atop the mythical island of Santorini. The primary objective is to be the first to ascend to celestial heights and claim victory. This captivating game offers a harmonious blend of tactical planning, quick-wittedness, and astute maneuvers, rendering it a compelling experience for players of all proficiencies.

Developed using Apache Maven and programmed with Java, the game delivers a seamless online gaming experience, facilitating challenging encounters with acquaintances, family, or enthusiasts across the global spectrum. JavaFX framework has been used for the graphical user interface, the game showcases visually resplendent aesthetics that breathe life into the enchanting realm of Santorini.

![santorini1](Images/santorini1.png)
    
![santorini2](Images/santorini2.png)
    
![santorini3](Images/santorini3.png)

## Improved NEQR Compression 
Writing a paper on images compression in Quantum Computers and relative implementation.

<p align="center">
    Abstract
</p>

*This paper presents the implementation of NEQR and its compression. The aim of the paper is to highlight the practical limits of the
compression theorized so far and to propose an improvement which can be implemented grouping CNOT gates that have the same control information. At the end of the report we present some results we obtain highlighting the improvement achieved in the compression phase.*

![NEQR](Images/NEQR.png)


## CLUP 

Design Document (DD) and Requirement Analysis and Specification Document (RASD) of an application (CLup) which help store managers to regulate the
influx of people in the building monitoring entrances to comply with the coronavirus safety regulations.
Moreover the application has the goal to prevent people from waiting outside the store safeguarding clients health and not wasting their time.

![RASD_DD](Images/ddrasd.jpg)


## Working Zone 
The project is based on the low power dissipation coding method called "Working Zone".
Through this type of coding it is possible to transform an address according to the belonging to certain intervals (Working Zones).
The module is implemented using VHDL. The module reads the address to be encoded and the base addresses of the Working Zones and it outputs the encoded address.

A Working Zone is characterized by :
- Base address (first address that identifies the Working Zone, from 00000000 to 11111100)
- Size (fixed of 4 addresses including the base address)
- Number of bits per address (8 bits)

Each Working Zone base address is contained within a memory address (from 0 to 7) and therefore the number of Working Zones is 8. The address to be transmitted can vary from 0 to 127 (000000-011111111) and it is contained within address 8 of the memory. The encoded address, instead, is written inside
address 9 of the memory.
- If the address to be transmitted is not contained in any Working Zone, it is transmitted without change.
- If the address to be transmitted is contained in a Working Zone :
    - The first bit is set to 1
    - From the second bit to the fourth bit, the number of the Working Zone is encoded in binary.
        - 000 (WZ 0)
        - 001 (WZ 1)
        - 010 (WZ 2)
        - 011 (WZ 3)
        - 100 (WZ 4)
        - 101 (WZ 5)
        - 110 (WZ 6)
        - 111 (WZ 7)
    - In the remaining bits there is the one-hot encoding of the offset
        - 0001 (offset = 0)
        - 0010 (offset = 1)
        - 0100 (offset = 2)
        - 1000 (offset = 3)
        
![WZ1](Images/WZ1.JPG) 
![WZ2](Images/WZ2.JPG)


 ## Thumb Led

One week project in which I implemented a Convolutional Neural Network model which was able to distinguish three different classes
- Thumb UP
- Thumb DOWN
- No thumb

The predicted class is then sent via Bluetooth to the Raspberry Pi. 
An application, running on the Raspberry, processes the data received and based on what receives turns on a specific led.

https://user-images.githubusercontent.com/47597693/158889676-5202d286-a42f-43c0-abf5-110485148884.mp4


