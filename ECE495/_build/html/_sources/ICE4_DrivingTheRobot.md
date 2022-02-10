# Module 4: Driving the Robot
## In-Class Exercise 4
---

**You must open this file as a Jupyter Notebook (link below) to run code**

[Run this file as an executable Jupyter Notebook](http://localhost:8888/notebooks/ICE4_DrivingTheRobot.ipynb)

### A note on this document
Now that you have a better understanding of the Linux operating system and Python programming language the Jupyter Notebooks will be used primarily to guide you through the In-Class Exercises and Laboratories. You will execute the majority of your commands and code within the Linux terminal.

### Purpose
This In-Class Exercise will introduce you to utilizing pre-built ROS packages to accomplish a task. It will also provide you experience interacting with someone else's source code (.py files) to learn how that component works. You will use ROS to run two nodes, **usafabot_serial** and **teleop_twist_keyboard**, to drive the USAFABOT with a keyboard. You will continue to practice using ROS tools to observe how these components communicate.

### Code used to drive the robot
1. On the Master, open a terminal and run **roscore**.

1. Open a new terminal on the Master and create a secure shell into the Robot using the SSH command you learned during Module 2. This will allow you to run commands as if you were on the Robot.

1. Using the secure shell, open the source code for the **usafabot_serial** node using the nano command line editor tool through the rosed command:

    `pi@robot:~$ rosed usafabot usafabot_serial.py`
>**Syntax:** `rosed <package> <filename>`

    **Note**: You may remember when we set up our *.bashrc* file we set the system variable **EDITOR** to `nano -w`. This enables the `rosed` command to utilize the nano editor.
    There is a lot going on in this file. We will go over the important aspects in class, but you should understand that it creates a node called **usafabot_serial** which subscribes to the **/cmd_vel** topic and receives *Twist* messages (we only use the linear x and angular z characteristics). These values are then sent to the MSP_432 microcontroller over a serial connection (USB) to drive the robot accordingly. Simultaneously, the node receives wheel speeds from the robot and publishes the values over the **/wheel_speeds** topic.
<br>

1. Close the editor by hitting `ctrl+x`.

1. It is always a good idea to check that the robot is communicating with the master. To do this, we can list the active topics the Robot sees. Run the following within your secure shell:

    `pi@robot:~$ rostopic list`
<br>
    If all is well, then there should be two topics provided by **roscore**: **/rosout** and **/rosout_agg**. We will typically ignore these topics.

1. Run the **usafabot_serial** node using the rosrun command:

    `pi@robot:~$ rosrun usafabot usafabot_serial.py`
    
    Your robot is now ready to drive and should be listening for *Twist* messages to be sent over the **/cmd_vel** topic.

### Driving the robot
1. Open a new terminal on the Master and observe the nodes currently running:

    `dfec@master:~$ rosrun rqt_graph rqt_graph`
    
    You should only see one node running right now, **usafabot_serial**, with no connections.
1. Open a new terminal tab and list the active topics. There should be two active topics besides the ones created by **roscore**: **/cmd_vel** and **/wheel_speeds**
1. You can find information on pre-built packages by googling the package name along with the ROS distribution. Open up your favorite browser and google "teleop twist keyboard noetic". The first result should be from the ROS wiki page.
1. Ensure the ROS package **teleop_twist_keyboard** is installed on your Master:

    `dfec@master:~$ rospack find teleop_twist_keyboard`
    
    If installed, the command should return the absolute path to the package, similar to `/opt/ros/noetic/share/teleop_twist_keyboard`
    
    If the command instead returns an error, then you need to install the package using apt:
    
    `dfec@master:~$ sudo apt install ros-noetic-teleop-twist-keyboard`
    
    **Hint**: All packages built for Noetic can be downloaded in the above manner (ros-noetic-desired-pkg with underscores in the package name replaced by dashes). Some packages were only built for previous ROS distribution and will have to be built from source (we will demonstrate this at a future time).
    
1. Run the **teleop_twist_keyboard** node on the Master:

    **PRO-TIP**: Don't forget your tab completion! You can start typing a package name or node and then hit tab for it to complete the command for you!
    
    `dfec@master:~$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
    
1. Before we get too excited and drive the robot off a cliff, observe how the nodes are communicating using the **rqt_graph** tool in a new terminal (if you still have the previous rqt_graph running, you can hit the refresh button in the top left corner).

1. Select the terminal that has the **teleop_twist_keyboard** node running and observe the instructions for sending *Twist* messages. These are the same as when driving the simulated robot.

1. The USAFABOT operates best with a linear velocity between 0.5 m/s and 0.2 m/s. It turns best with an angular velocity between 0.5 rad/s and 1.5 rad/s. Drive the robot using these parameters.

### ROS
In labs throughout this course we will request information about the topics, nodes, and messages within your system. Accomplish the following in a new terminal on your Master (you can ignore all nodes/topics that result from **roscore**).

1. List all running nodes.

1. Determine what topics the nodes subscribe and publish to (repeat for each node).

1. Display running nodes and communication between them.

1. List the active topics.

1. Determine the type of messages sent over the topics (repeat for each topic).

1. Determine the fields of the messages.

1. Observe the information sent over a topic (repeat for each topic).

### Checkpoint
Once complete, get checked off by an instructor showing the output of each of the above.

## Summary
In this exercise you examined and used pre-built packages and source code to drive the USAFABOT and understand how the system worked. You then were able to analyze the topics, nodes, and messages within the ROS system to better understand the flow of information and control. The **pro-tips** presented throughout this exercise will make you a better user of Linux and ROS.

## Cleanup
In each terminal window, close the node by typing `ctrl+c`. Exit any SSH connections. In each of the notebooks reset the Jupter kernel and clear output. Now it is safe to exit out of this window. Shutdown the notebook server by typing `ctrl+c` within the terminal you ran `jupyter-notebook` in. Select 'y'.
