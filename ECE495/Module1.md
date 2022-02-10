# Module 1: Robotics Operating System (ROS)
## Introduction to ROS and the USAFABot
---

**You must open this file as a Jupyter Notebook (link below) to run code**

[Run this file as an executable Jupyter Notebook](http://localhost:8888/notebooks/Module1_ROS.ipynb)

### A note on this document
This document is known as a Jupyter Notebook; it is used in academia and industry to allow text and executable code to coexist in a very easy to read format. Blocks can contain text or code, and for blocks containing code, press `Shift + Enter` to run the code. Earlier blocks of code need to be run for the later blocks of code to work.

### Purpose
This Jupyter Notebook accompanies the introduction to ROS notetaker used in class. We will apply the knowledge you learned by interacting with a simulated USAFABot.

### Setting up the terminal for ROS
1. Open a new Linux terminal by pressing:

    `ctrl+alt+t`
    <br>
1. Change to your root directory:

    `cd`

    When the `cd` command has no arguments, it changes to the user's home directory. The command `cd ..` will move up one level.
    <br>
1. Print the working directory:

    `pwd`
    <br>
1. List the contents of the current directory:

    `ls`
    <br>
1. Change directory to your master_ws folder:

    `cd master_ws`
    <br>
1. List the contents of the current directory:

    `ls`
    <br>
1. Change directory to your devel folder:

    `cd devel`
    <br>
1. You should notice a setup.bash file in the devel folder. Source this file allowing you to call your ROS packages:

    `source setup.bash`
    <br> 
1. Open your .bashrc file (a script that is ran every time a new terminal instance is opened):

    `sudo nano ~/.bashrc`

    This command is ran with sudo priveleges as the .bashrc is a system level file. The '~' character indicates the .bashrc file is in the user's home directory and allows us to access it from anywhere in the file system (would be the same as using the absolute path `sudo nano /dfec/home/.bashrc` ).
<br>
1. Scroll to the bottom of the file. You should see a few lines of code such as the following:

    ```
        source /opt/ros/noetic/setup.bash
        source ~/master_ws/devel/setup.bash
        export ROS_PACKAGE_PATH=~/master_ws/src:/opt/ros/noetic/share
        export ROS_MASTER_URI=http://master:11311
        export EDITOR='nano -w'
    ```
    <br>
    The first three lines set up the ROS environment allowing access to built-in packages and packages within your master workspace. Line 4 establishes which system hosts the ROS network. You can replace master with your robot, and the entire ROS network would run on your robot.
    Hit ctrl+s to save, then ctrl+x to exit.
    <br>
    You need to source (execute) the .bashrc file any time the file is changed (the .bashrc file is ran every time a new terminal is opened, but since we haven't opened a new terminal yet, we have to run it manually).
    <br>
    <br>
1. Execute the .bashrc file

    `source ~/.bashrc`

### Running the USAFABot
The best way to learn about ROS is through implementation, however, let's start off by playing around with a virtual USAFABot! The USAFABot is a tool we will utilize throughout this course to apply and integrate robotics systems. Ultimately you will create a complex embedded robotics system to perform a specific dedicated task, such as navigating the halls of DFEC. But let's see if we can get the robot to drive around first.

1. In a terminal (**Pro tip:** ctrl+alt+t opens a new terminal window, while ctrl+shift+t opens a new terminal tab in an existing window) and initialize the ROS network:

    `roscore`
<br>
1. That terminal is now occupied. Open a new terminal tab/window and launch the USAFABot gazebo launch file (A launch file is a way to run a number of nodes at once, we will learn about launch files later):

    `roslaunch usafabot_gazebo usafabot_gazebo.launch`

>**Syntax:** `roslaunch <package> <launchfile>`

The launch file will run the nodes necessary to simulate our robot and opens two windows: Gazebo and rviz. Gazebo is a simulation tool that provides physics for our robot and enables the testing of algorithms prior to real-world implementation. You should see a USAFABot sitting at (0, 0) facing the positive x direction surrounded by a maze. Using the mouse, left-click and holding will pan, holding the scroll wheel will change the orientation of the camera, and scrolling will zoom in and out.

The second window is rviz, a tool that visualizes topics interacting in our system. You should see a number of red dots outlining the location of the wall in our simulation. These are objects detected by our LIDAR which is communicating over a scan topic. Using the mouse, left-click will change the orientation of the camera, holding the scroll wheel will pan, and scrolling will zoom in and out (would be nice if they were the same).

Don't worry! We will learn more about both of these tools at a later time.

Let's go ahead and take a look at what nodes and topics currently running in our system. 

The "!" character in front of the following commands allows us to run bash commands from the Jupyter NB and would **NOT** be used in the command line.


```python
!rosnode list
```

There are five nodes running, the first two enable the Gazebo simulation, the third allows for the visualization of the simulated robot, the fourth is created every time *roscore* is ran and helps with communication in our network, and the last enables rviz.

Not too exciting yet, so lets see what topics are active.


```python
!rostopic list
```

There are a lot of topics that are utilized to simulate our robot. Our real-world robot will eventually have most of these, such as **/cmd_vel**, **/imu**, **/scan**, and **/usb_cam**. These are topics that allow us to communicate with some of the simulated hardware, such as the orientation sensor (/imu), LIDAR (/scan), driving the robot (/cmd_vel), and camera (/usb_cam). The rest of the topics enable visualization and movement within the simulation and can be ignored for now.

Another useful tool for visualizing nodes and topics is called *rqt_graph*:


```python
!rqt_graph
```

All that is going on right now is Gazebo is publishing the position and scan data which rviz uses to visualize the robot. **Close your rqt_graph** and let's add another node to make things a bit more interesting. 

Earlier we saw the topic **/cmd_vel**. This is a topic used to send *Twist* messages to a robot. A *Twist* message includes linear and angular x, y, z values to move a robot in a 3-dimensional space (google `ROS twist message` for more information). Our robot only drives in 2-dimensions and the wheels can only move forward and backward, so we will only use the linear x (forward and backward) and angular z (turning) attributes of the *Twist* message. To drive our simulated robot, we need a node that can publish *Twist* messages to the **/cmd_vel** topic. Luckily, there is a pre-built node called Teleop_Twist_Keyboard that sends *Twist* messages using a keyboard!

Open a new terminal tab (select the terminal and press ctrl+shift+t) and run the following (**Pro tip**: in a Linux terminal if you type the first couple letters of a command and then hit tab it will autocomplete the command for you):

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

>**Syntax:** `rosrun <package> <executable>`

To drive the robot use the 'x' key to decrease linear x speed to about .25 m/s and use the 'c' key to decrease angular z speed to about .5 rad/s. Now follow the directions to utilize the keyboard to drive the robot! You should see the robot move in the simulation.

Let's take a look at our rqt_graph to see if anything has changed.


```python
!rqt_graph
```

You should now see the teleop_twist_keyboard node which sends messages over **/cmd_vel** topic to Gazebo. **Close the rqt_graph window**. Let's run through a number of commands that will provide you more information about your ROS network. You will use these throughout the course to determine what is going on in your ROS network.

### Common ROS commands

The `rosnode` command allows us to interact with nodes. Typing any ROS command followd by `--help` will provide information about that command:


```python
!rosnode --help
```

Let's get some information about our new node, teleop_twist_keyboard:


```python
!rosnode info /teleop_twist_keyboard
```

The output of the command lists the topics the node is publishing and subscribing to (here is where we can see it publishes on **/cmd_vel**).

The `rostopic` command interacts with topics.


```python
!rostopic --help
```

Some of the common rostopic commands we will use in this course are `echo`, `hz`, `info`,  `type`, and `list`


```python
!rostopic list
```

Let's get some information about the **/cmd_vel** topic.


```python
!rostopic info /cmd_vel
```

From the output we can see what nodes are publishing and subscribing to the **/cmd_vel** topic.

Echoing the topic will allow us to see what messages are sent over the topic. After running the below command, browse back to your teleop_twist_keyboard node and drive the robot. You should see the twist messages sent to Gazebo.


```python
!rostopic echo /cmd_vel
```

Note that when moving forward and backward ('i' and ',' keys) only a linear x value is sent, when turning left or right ('j' and 'l' keys) only an angular z value is sent, and when arcing ('u', 'o', 'm', and '.' keys) both a linear x and angular z value are sent.

To learn more about the messages sent over the **/cmd_vel** topic we can use the `type` command and the `rosmsg` tool.


```python
!rostopic type /cmd_vel
```


```python
!rosmsg show geometry_msgs/Twist
```

Or in one combined command:


```python
!rostopic type /cmd_vel | rosmsg show
```

## Cleanup
Before moving on to the in-class exercise, close all running nodes. In the Jupyter NB at the top menu bar select "Kernel" and "Restart & Clear Output". In each terminal window, close the node by typing `ctrl+c`. Ensure roscore is terminated before moving on to the ICE.
