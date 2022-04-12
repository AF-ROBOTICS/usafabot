#!/usr/bin/env python
# coding: utf-8

# # Lab 1: Custom Messages
# ---

# ## Purpose
# This lab will provide practice in creating custom messages. You will take the **cmd_vel** topic and *Twist* message from the **teleop_twist_keyboard** node and use a **controller** node to send *USAFABOT_Cmd* messages to the **usafabot_serial** node. As we introduce new subsystems to the robot, the **controller** node will determine priority between these sensors/systems and then drive the robot using the **usafabot_cmd** topic accordingly.
# 
# > 📝️ **Note:** The **teleop_twist_keyboard** node only sends a *Twist* message when a new key is pressed; we will have to provide logic within our controller to do the same for our *USAFABOT_Cmd* message, otherwise our controller will continuously try to send a value to the robot and fill the robot's buffer fairly quickly. We can do this by comparing the previously sent *USAFABOT_Cmd* message with the current message and if they are **not** the same then publish the command. You will see this in the **controller.py** instructions.

# ## Robot
# ### Setup:
# 1. In a new terminal on the **Master**, create a secure shell connection to your **Robot**.
# 
# 1. Create a custom message within the **usafabot** package's **msg** folder that is called *USAFABOT_Cmd.msg* and has two fields and a header:
# 
#     ```python
#     Header header
#     float64 lin_x
#     float64 ang_z
#     ```
# 
# 1. Edit **package.xml** per the ICE instructions (don't remove anything from the file, just add what is needed to run your custom message).
# 
# 1. Edit **CMakeLists.txt** per the ICE instructions (don't remove anything from the file, just add what is needed to run your custom message).
# 
# 1. Make and source your workspace.
# 
# > 📝️ **Note**: This message is now part of the **usafabot** package and if you ran `rosmsg show USAFABOT_Cmd` it would return *usafabot/USAFABOT_Cmd*.

# ### usafabot_serial.py
# 1. Edit **usafabot_serial.py** within the **usafabot** package (`rosed usafabot usafabot_serial.py`) to:
# 
#     - Import the new message instead of *Twist*
#     - Subscribe to the **usafabot_cmd** topic instead of **cmd_vel** and use the new message instead of *Twist*
#     - Edit the `callback_write()` function to use the new message fields.
#     
# > 💡️ **Tip:** Look for the tag "TODO" within the python file.

# ## Master 
# ### Setup:
# To use the same custom message on two different machines the custom message must be created and part of the same package on both machines. On the **Robot**, the *USAFABOT_Cmd* message is part of the **usafabot** package. Therefore, the message must be part of the **usafabot** package on the **Master**. When communicating, ROS uses the **whole** name of the msg, *usafabot/USAFABOT_Cmd*, so these must be the same on each system. There should already be a **usafabot** package on your master.
# 
# 1. In a terminal on the **Master**, browse to the **usafabot** package.
# 
# 1. Create a custom message within the **usafabot** package's **msg** folder that is called *USAFABOT_Cmd.msg* and has two fields and a header:
# 
#     ```python
#     Header header
#     float64 lin_x
#     float64 ang_z
#     ```
# 
# 1. Edit **package.xml** per the ICE instructions (don't remove anything from the file, just add what is needed to run your custom message).
# 
# 1. Edit **CMakeLists.txt** per the ICE instructions (don't remove anything from the file, just add what is needed to run your custom message).
# 
# 1. In the `/master_ws/src/ece495_master_spring2022-USERNAME/` folder, create a **lab1** package which depends on **std_msgs**, **rospy**, **geometry_msgs**, and **usafabot**.
# 
# 1. Make and source your workspace.

# ### controller.py
# 1. Create a **controller.py** node within the src folder of **lab1**.
# 1. Edit the file and import **rospy**, **Twist**, and the new message, **USAFABOT_Cmd**.
# 1. Create a Controller class that will have four functions:
# 
#     ```python
#     class Controller:
#             # class initialization
#             def __init__(self):
#                 # TODO: complete __init__() function
#                 
#             # callback called when a new message is received over 
#             # the cmd_vel topic
#             def callback_keyboard(self, kb):
#                 # TODO: complete callback_keyboard() function
# 
#             # callback that will run at 100 Hz
#             # will be used to control the robot
#             def callback_controller(self, event):
#                 # TODO: complete callback_controller() function
#                 
#             # function to ensure all values are zeroed out when shut down
#             def shutdownhook(self):
#                 # TODO: complete shutdownhook() function
#     ```
#                 
# 1. Add the following to the `__init__()` function:
#     
#     - Instance variable, `self.cmd`, to store the `USAFABOT_CMD()` message.
#     - Instance variable, `self.kb`, to store the `Twist()` message sent from **teleop_twist_keyboard**.
#     - Instance variable, `self.prev_x`, to store the previous linear x value.
#     - Instance variable, `self.prev_z`, to store the previous angular z value.
#     - A subscriber to the **cmd_vel** topic with a callback to the `callback_keyboard() function.
#     - A publisher to the **usafabot_cmd** topic.
#     - A timer that runs at 100 Hz with a callback to the `callback_controller()` function.
#     - The `ctrl_c` boolean instance variable.
#     - The shutdown method: `rospy.on_shutdown(self.shutdownhook)`.
#     <br>
#     
# 1. Write the `callback_keyboard()` function that will copy the message received (kb) to the keyboard instance variable. Remember, `callback_keyboard()` is called every time a new *Twist* message is sent over the **cmd_vel** topic. The *Twist* message is passed to the function as the second input (kb).
# 
# 1. Write the `callback_controller()` function that:
#     1. If `ctrl_c` is not pressed.
#     1. Set the `lin_x` and `ang_z` characteristics of the *USAFABOT_Cmd* message to the `linear.x` and `angular.z` characteristics of the *Twist* message. For example:
# 
#     ```python
#     self.cmd.lin_x = self.kb.linear.x
#     ```
#     where the left side of the expression is our custom *USAFABOT_Cmd* message and the right side is the *Twist* message sent over the **cmd_vel** topic (you can find the format of the *Twist* message in the ROS documentation: [Twist Message](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/Twist.html)).
#     
#     1. Compare the previous linear x and angular z values with the current values and if they are **NOT** the same then publish the command.
#     1. Reset previous values to current values.
#     
# 1. Write the `shutdownhook()` function to print "Controller exiting. Halting robot.", set `self.ctrl_c` to *True*, and publish a message over the **usafabot_cmd** topic with a linear x and angular z of 0 so the robot will stop.
# 
# 1. Create a `__main__()` function that initializes the controller node (node names are lower case), creates an instance of the Controller class, and then spins (runs forever).
# 
# > 📝️ **Note:** Remember that in a Python file the `main()` function is different than what we used in Jupyter Notebooks. Look at ICE5 for an example.

# ## Run your nodes
# 1. On the **Master** open a terminal and run **roscore**.
# 1. Open another terminal and enable statistics for **rqt_graph**.
# 1. Run the controller node.
# 1. Run the **teleop_twist_keyboard** node.
# 1. Using the terminal with the secure shell into the **Robot** and run the **usafabot_serial.py** node.
# 1. Use the keyboard to drive the robot (operation should be exactly as before and the robot should respond accordingly).

# ## Report
# Complete a short 2-3 page report that utilizes the format and answers the questions within the report template. The report template and an example report can be found within the Team under `Resources/Lab Template`.

# ## Turn-in Requirements
# **[25 points]** Demonstration of keyboard control of USAFABot (preferably in person, but can be recorded and posted to Teams under the Lab1 channel).
# 
# **[50 points]** Report via Gradescope.
# 
# **[25 points]** Code: push your code to your repository. Also, include a screen shot of the **controller.py** and **usafabot_serial.py** files at the end of your report.
