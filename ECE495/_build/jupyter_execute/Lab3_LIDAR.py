#!/usr/bin/env python
# coding: utf-8

# # Lab 3: LIDAR
# ---

# ## Purpose
# This lab will integrate the Slamtec RPLIDAR A1 Range Scanner with the existing USAFABot controller to drive the robot forward and turn 90 degrees when there is an obstacle.

# ## Master
# ### Setup:
# In the `/master_ws/src/ece495_master_spring2022-USERNAME/` folder, create a **lab3** package which depends on **rospy**, **std_msgs**, **geometry_msgs**, **um7**, **rplidar_ros**, and **usafabot**.
# 
# Make and source your workspace.

# ### controller.py
# 1. Copy the controller.py file from lab2 into the lab3 package.
# 
# 1. Open the controller.py file from lab3 using the **Thonny** editor.
# 
# 1. Import the laser message used in ICE8.
# 
# 1. Remove the import of the um7.srv library (we will no longer be initializing the IMU within the controller.py file, because it is initialized in the robotics.launch file).
# 
# 1. Copy the 2 lambda functions from ICE8 (RAD2DEG & DEG_CONV).
# 
# 1. Add the following Class variables within the class above the `__init__()` function:
# 
#     1. `DISTANCE = 0.4 # distance from the wall to stop`
#     1. `K_POS = 100 # proportional constant for slowly stopping as you get closer to the wall`
#     1. `MIN_LIN_X = 0.05 # limit m/s values sent to USAFABot`
#     1. `MAX_LIN_X = 0.2 # limit m/s values sent to USAFABot`
#     
# 1. Add the following to the `__init__()` function:
# 
#     1. Instance variable, `self.avg_dist`, initialized to 0 to store the average dist off the nose.
#     1. Instance variable, `self.got_avg`, initialized to False to store when an average is calculated.
#     1. A subscriber to the LIDAR topic of interest with a callback to the callback_lidar() function.
#     
# 1. Remove the call to `self.init_imu()` from the `__init__()` function. The IMU is initialized within the robotics.launch file.
# 
# 1. Remove the `init_imu()` function.
# 
# 1. Add the `callback_lidar()` function from ICE8, removing print statements and setting the instance variables, `self.avg_dist` and `self.got_avg`.
# 
# 1. Edit the `callback_controller()` to accomplish the following:
# 
#     1. Remove user input.
#     1. When not turning and you have an average LIDAR reading, calculate the distance error (`actual dist` - `desired dist`) and use that to drive your robot straight at a proportional rate (very similar to how we calculated the turn rate in lab 2).
#     1. Limit the linear speed of the robot to `MIN_LIN_X` and `MAX_LIN_X`.
#     1. If within `DISTANCE` of a wall, then stop and start turning (left or right, you decide).
#     
#     > 💡️ **Tip:** You should be able to reuse a lot of code for this step!
#     
#     1. Save the linear x and angular z values to the `USAFABOT_Cmd` message and publish.
#     
# 1. Save, exit, and make executable if necessary.

# ### Launch File
# Create a launch file on the **Master**, lab3.launch, that launches the `controller.py` file.

# ## Run your nodes
# 1. Open secure shell into the **Robot** and run the **robotics.launch** file.
# 1. On the **Master**, open a terminal and run your **lab3.launch** file.

# ## Report
# Complete a short 2-3 page report that utilizes the format and answers the questions within the report template. The report template and an example report can be found within the Team under `Resources/Lab Template`.
# 
# > 📝️ **NOTE:** We will be primarily grading sections 3.3 System level design and 3.4 Testing for this lab, but do include the entire lab as you will need other components for the final project report.

# ## Turn-in Requirements
# **[25 points]** Demonstration of keyboard control of USAFABot (preferably in person, but can be recorded and posted to Teams under the Lab1 channel).
# 
# **[50 points]** Report via Gradescope.
# 
# **[25 points]** Code: push your code to your repository. Also, include a screen shot of the **controller.py** file at the end of your report.
