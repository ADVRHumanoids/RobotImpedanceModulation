# ➤ Robot Impedance Modulation

## Introduction
**Robot Impedance Modulation** is a **ROS2** based package aimed to help the community in calculating a _variable_ impedance of a robot. 
Considering a robot having SEA motors, typically used for collaborative robots (Cobots), or a general robot controlled with the following control law

$$
\tau = K \\, (q^{ref} - q) - D \dot q + \tau^{ff},
$$

the package provides the required **stiffness $K$** and **damping $D$** (diagonal matrices) to be set in the motors or equivalently used to compute such a control law. By defining the 
required force and admissible error crucial for task execution, the robot impedance is accordingly designed. For further details on the algorithm, you can have a look at the related article 
available [**_here_**](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=10000215).


## Outline
* [Introduction](#introduction)
* [Outline](#outline)
* [How to Use It](#how-to-use-it)
* [How to Configure It](#how-to-use-it)
* [Installation](#installation)
* [Execution](#execution)
* [Dependences](#dependences)
* [Maintaner](#maintaner)

## How to Use It

Here, an explanation of how to use the package is given. We start by providing the rationale behind it, and then we detail the practical usage. 

As mentioned in the introduction, this package allows you to directly obtain the impedance parameters of a robot, i.e. the values of 
stiffness and damping for each joint _"j"_ of the robot, $k_j$ and $d_j$, respectively. These values can be used either to set the motor parameters or to calculate the above control law. 
Classical approaches apply fixed values to design these parameters by a manual procedure of trials and errors. For real-time applications in dynamic environments, this procedure can not 
properly generate suitable robot reactions based on occurring changes maybe leading to the failure of the task execution. 
To overcome these drawbacks, here, the impedance modulation is intended to be designed in such a way as to _better_ fit with the task to accomplish. 
For this purpose, by specifying the required force to exert and the precision demanded by the task, the package allows automatically computing a resulting 
**time-varying** and **_task_-varying** robot impedance.

Based on that, we assume that the task to execute is described with 
- **force to execute**: expressed as a wrench at the end-effector of the robot, 3D force vector, and 3D moment vector.
- **precision to satisfy**: expressed as a pose error vector, 3D position error vector, and 3D orientation error vector (Euler angles RPY).
- **joints position reference vector**: refers to the references of the joints position.
- **joints position vector**: refers to the measurements of the joints position.

For real-time applications, these quantities are expected to be time-varying, at least the joints position vector. Otherwise, the robot is expected to be fixed, and the calculation could not fit the task if it is executed.

**In order to properly use the package**, it is required to provide:
1. the above information
2. the model of the robot via standard URDF file.

How to do that? how to provide this information, It is specified below.

The task force, precision, joints positions reference, and measurements are assumed to be sent via topic by a publisher using the custom message provided in the package and available in the _msg folder_. The name of the message is _TaskMsg.msg_ and we report here its content

```
bool cartesian_space
float64[] joints_position
float64[] joints_position_reference
float64[] task_pose_reference
float64[] task_wrench
float64[] task_precision
```

By filling these fields, and cyclically sending them on a topic, the package computes the required stiffness and damping to be used. As you can notice, we give the possibility to specify the 
desired robot configuration in terms of the Cartesian pose of the end-effector. By putting true to the boolean cartesian_space, the package will ignore the field `joints_position_reference` and it will use `task_pose_reference` expressed as translation and orientation. The orientation is required to be expressed in Euler angles RPY and in radiants.


[ATTENTION] Due to the technical details of the implemented algorithm, the package will further provide a torque to be sent to the motors as well. This is due to two reasons, one is related to the control theory and the second is due to practical reasons. Specifically, to determine a robot impedance that satisfies task requirements, the algorithm computes a full stiffness matrix. To realize the computed stiffness thus is required to send to the motors this additional torque. 

The result of the calculated impedance is sent via message on a topic. The message sent is a custom message and it is provided in the package as well. It is named _ImpedanceMsg.msg_ and it is in the _msg folder_. Its content is 

```
float64[] robot_stiffness
float64[] robot_damping
float64[] robot_feedforward_torque
```

So, by subscribing to the message you can use these values to control your robot by setting these values in the motors.

## How to Configure It

As we have seen, to use the package, it is necessary to

1. Have a URDF file of the robot.
2. Send a message to give the task information.
3. Retrieve the result by listening to a topic.

But, to make everything effectively working we need to properly configure the package before using it. In particular, it is required to 

1. provide the URDF file by specifying the path where the file is.
2. set the names of the topics.
3. set the initial configuration of the robot, defined in terms of joint position at the initial instant time.
4. set the maximum stiffness and damping available by the motors.
5. set the preset stiffness and damping configured in the robot.
6. set the initial force and precision to set on the robot.
7. specify the base frame of the kinematic chain intended to control (the same as in the URDF file).
8. specify the tip frame of the kinematic chain intended to control (the same as in the URDF file).
9. set the transition time (seconds) during which the impedance of the robot is adjusted from its preset values to the one intended to start the task. This is to ensure the smoothness of the impedance profiles. Messages related to this phase will be sent on the topic and must be actuated.
10. set the rate of the plugin.
11. set the path where you would like to save the logs (the package provides them).
12. set the verbose mode if you like.
 
This can be simply done just by inserting the proper values in the configuration file. The file is named _impedance_modulation_settings.yaml_ present in the _config folder_. The ROS node will start by loading these parameters. Here, we report its content

```yaml
/robot/impedance_settings:
  ros__parameters:
    robot_initial_config: [0.0,-1.56,0.9,0.2,-0.5,1.12]
    stiffness_preset: [100.0,100.0,100.0,100.0,100.0,100.0]
    stiffness_constant: [200.0,200.0,200.0,200.0,200.0,200.0]
    stiffness_maximum: [2000.0,2000.0,2000.0,2000.0,2000.0,2000.0]
    damping_preset: [100.0,100.0,100.0,100.0,100.0,100.0]
    damping_maximum: [50.0,50.0,50.0,50.0,50.0,50.0]
    wrench_initial: [0.0,0.0,0.0,0.0,0.0,0.0]
    precision_initial: [0.1,0.1,0.1,0.1,0.1,0.1]
    robot_urdf_model_path: "/home/liana/ros2_humble/src/ROS2UtilityNodes/urdf/inail2arm.urdf"
    robot_base_frame_name: "base_link"
    robot_tip_frame_name: "arm1_6"
    topic_subscriber_name: "/robot/task_planning"
    topic_publisher_name: "impedance_planning"
    transition_time: 10.0
    rate: 1000
    log_path: "/tmp/impedance_planner"
    verbose: true
```

The values here are just an example. You have to make the proper changes. Please, do not touch any names (the green ones), just modify the fields (the blue ones).

## Installation
**To Install** Variable Impedance Modulation Package run
```console
git clone https://github.com/lia2790/RobotImpedanceModulation.git
```
inside the _**src folder**_ of the workspace run
```console
colcon build
```

## Execution
To execute the code, just run:
```console
ros2 launch rim impedance_modulation.launch.py
```

Remember to source `install/setup.sh`, by simply
```console
source install/setup.sh
```
inside the principal folder. If everything has been properly set, you should have an output similar to
```console
liana:~/ros2_humble$ ros2 launch rim impedance_modulation.launch.py 
[INFO] [launch]: All log files can be found below /home/liana/.ros/log/2023-07-08-15-45-00-723821-liana-MS-7820-20325
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [ImpedanceModulation-1]: process started with pid [20326]
[ImpedanceModulation-1] [INFO] [1688823900.952604282] [robot.impedance_settings]: I am initializing the ROS node...
[ImpedanceModulation-1] 
[ImpedanceModulation-1] Worked for 0.00 sec (0.0 MB flushed)....average load is -nan 
[ImpedanceModulation-1] [INFO] [1688823900.983917845] [robot.impedance_settings]: I am loading the ROS node params...
[ImpedanceModulation-1] 
[ImpedanceModulation-1] [INFO] [1688823900.990769844] [robot.impedance_settings]: I am initializing the robot impedance...
[ImpedanceModulation-1] 
[ImpedanceModulation-1] Created variable 'EEwrench' (20 blocks, 500 elem each)
[ImpedanceModulation-1] Created variable 'EETaskwrench' (20 blocks, 500 elem each)
[ImpedanceModulation-1] Created variable 'TaskErrorDesired' (20 blocks, 500 elem each)
[ImpedanceModulation-1] Created variable 'CartesianStiffness' (20 blocks, 500 elem each)
[ImpedanceModulation-1] Created variable 'JointStiffness' (20 blocks, 500 elem each)
[ImpedanceModulation-1] Created variable 'JointDamping' (20 blocks, 500 elem each)
[ImpedanceModulation-1] Created variable 'FeedforwardTorque' (20 blocks, 500 elem each)
[ImpedanceModulation-1] Created variable 'JointsPosition' (20 blocks, 500 elem each)
[ImpedanceModulation-1] Created variable 'JointsPositionReference' (20 blocks, 500 elem each)
[ImpedanceModulation-1] Worked for 0.02 sec (2.6 MB flushed)....average load is 0.00 
[ImpedanceModulation-1] ------------------------------------------------------------------------------
[ImpedanceModulation-1] Robot Impedance Modulation started! Ready to accept task planning...
```

If yes, then your package is listening to messages and ready to compute the impedance.

## Dependences
The Robot Impedance Modulation requires the following dependencies:
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page): to handle with basic algebra;
* [MatLogger2](https://github.com/ADVRHumanoids/MatLogger2): to log data;
* [KDL](https://www.orocos.org/kdl.html): to handle robot quantities.

## Maintaner

|<img src="https://avatars0.githubusercontent.com/u/15608027?s=400&u=aa95697b36504a10aeff4bf95d5d2f355ae94f07&v=4" width="180">|
|:-------------:|
|Liana Bertoni|
|liana.bertoni at iit.it|

