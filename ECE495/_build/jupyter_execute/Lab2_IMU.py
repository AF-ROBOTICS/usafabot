#!/usr/bin/env python
# coding: utf-8

# # Module 6: Inertial Measurement Unit
# ## Lab 2
# ---

# ### Purpose
# This lab will integrate the UM7-LT Orientation Sensor with the USAFABot controller to turn the robot 90 degrees left or right.

# ### Master
# #### Setup:
# In the `/master_ws/src/ece495_master_spring2022-USERNAME/` folder, create a **lab2** package which depends on **std_msgs**, **rospy**, **geometry_msgs**, **um7**, and **usafabot**.
# 
# #### controller.py
# 1. Copy the controller.py file from lab1 into the lab2 package.
# 
# 1. Open the controller.py file from lab2 using the **Thonny** editor.
# 
# 1. Import the math library, /imu/rpy message, and um7.srv services used in ICE6.
# 
# 1. Add the following Class variables within the class above the `__init__()` function:
# 
#     1. `K_HDG = 0.1 # rotation controller constant`
#     1. `HDG_TOL = 15 # heading tolerance +/- degrees`
#     1. `MIN_ANG_Z = 0.5 # limit rad/s values sent to USAFABot`
#     1. `MAX_ANG_Z = 1.5 # limit rad/s values sent to USAFABot`
#     
# 1. Add the following to the `__init__()` function:
# 
#     1. A call to the `init_imu()` function
#     1. Instance variable, `self.curr_yaw`, initialized to 0 to store the current orientation of the robot
#     1. Instance variable, `self.goal_yaw`, initialized to 0 to store the goal orientation of the robot
#     1. Instance variable, `self.turning`, initialized to `False` to store if the robot is currently turning
#     1. A subscriber to the IMU topic of interest with a callback to the callback_imu() function
#     
# 1. Add the `init_imu()` and `convert_yaw` functions from ICE6.
#     
# 1. Add the `callback_imu()` function from ICE6, removing print statements and setting the instance variable, `self.curr_yaw`.
# 
# 1. Edit the `callback_controller()` function so it turns the robot 90 degrees in the direction inputed by the user (left or right). Below is some pseudo-code to help you code the controller function 
# 
# > 📝️ **Note:** Pseudo-code is not actual code and cannot be copied and expected to work! Make sure you comment out all the subscriber to the **cmd_vel** topic.

# ```python
# def callback_controller(self, event):
#     # local variables do not need the self
# 	yaw_err = 0
# 	ang_z = 0
#     # not turning, so get user input
#     if not turning:
#         read from user and set value to instance variable, self.des_yaw
#         input("Input l or r to turn 90 deg")
#         
#         # check input and determine goal yaw
#         if input is equal to "l" 
#             set goal yaw to curr yaw plus/minus 90
#             turning equals True
#         else if input is equal to "r"
#            	set goal yaw to curr yaw plus/minus 90
#             turning equals True
#         else 
#         	print error and tell user valid inputs
#             
#         # check bounds
#         if goal_yaw is less than 0 then add 360
#         else if goal_yaw is greater than 360 then subtract 360
#     
#     # turn until goal is reached
#     elif turning:
#         yaw_err = self.goal_yaw - self.curr_yaw
#         
#         # determine if robot should turn clockwise or counterclockwise
#         if yaw_err > 180:
#             yaw_err = yaw_err - 360
#         elif yaw_err < -180:
#             yaw_err = yaw_err + 360
#             
#         # proportional controller that turns the robot until goal 
#         # yaw is reached
#         ang_z = self.K_HDG * yaw_err
#         
#         if ang_z < self.MIN: ang_z = self.MIN		# need to add negative test as well!
#         elif ang_Z > self.MAX: ang_z = self.MAX	# need to add negative test as well!
#         
#         # check goal orientation
#         if abs(yaw_err) < self.HDG_TOL
#             turning equals False
#             ang_z = 0
#             
#    # set USAFABOT_Cmd message and publish
#    self.cmd.lin_x = 0
#    self.cmd.ang_z = ang_z
#    publish message
# ```

# ### Run your nodes
# 1. On the **Master** open a terminal and run **roscore**.
# 1. Open another terminal and enable statistics for **rqt_graph**.
# 1. Run the controller node.
# 1. Open secure shell into the **Robot** and run the **usafabot_serial.py** node.
# 1. Open another secure shell into the **Robot** and run the **um7_driver** node.
# > 📝️ **Note:** You should now have four terminals (or tabs) open. On the master: **roscore** and **controller.py**. On the robot: **um7_driver** and **usafabot_serial.py**. I typically have one terminal window with all of the tabs for the mater and one terminal window with all of the tabs for the robot.
# 1. Type "l" or "r" to turn the robot 90 degrees.

# ### Report
# Complete a short 2-3 page report that utilizes the format and answers the questions within the report template. The report template and an example report can be found within the Team under `Resources/Lab Template`.

# ### Turn-in Requirements
# **[25 points]** Demonstration of keyboard control of USAFABot (preferably in person, but can be recorded and posted to Teams under the Lab1 channel).
# 
# **[50 points]** Report via Gradescope.
# 
# **[25 points]** Code: push your code to your repository. Also, include a screen shot of the **controller.py** file at the end of your report.
