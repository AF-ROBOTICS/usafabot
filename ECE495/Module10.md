# Final Project
---

## Purpose
Students will utilize all previous modules to drive their robot in the DFEC halls. The LIDAR will be used to drive the robot centered between two walls, the IMU will be used to make 90/180/360 deg turns, OpenCV will be used to identify randomly placed stopped signs and act accordingly, and apriltag_ros will be used to navigate the robot throughout the halls (when the robot is a certain distance from the apriltag then turn left, right, around, or the robot has reached the goal depending on the tag ID).

## Design
The first step to a complex task such as this is to plan. You will not be successful if you just start trying to code a solution. Therefore, **you should not write ANY code until you have completed the design phase of the project**. There are a number of tools to help with design, but we will focus on two: graphs and finite state machines.

**Graphs:**
Build a notional ROS graph for your application using PowerPoint or another digital tool. You have a lot of practice up to this point using the *rqt_graph* tool, but this time you will design the graph before the project. Ensure your graph includes all nodes and topics you foresee using and which machine these will operate on.

**Finite State Machine:**
A finite state machine is a great way to plan out the logic flow of a complex problem. It allows you to visualize how your robot will transition states (current condition of the robot: e.g., driving forward) based on the inputs received from all of the sensors. Your "base state" will be driving forward. The rest is up to your design. For simplicity, we will use a Moore Machine, so the output only relies on the current state. 

A quick intro on finite state machines from ECE382 is here: [ECE382: Module 7 Part 1](https://youtu.be/A8m0qo2MKlE?list=PLLvuo5HBf25HKbK18J28lJNyPkltAxfln&t=26)

> 📝️ **Note:** Make sure you include both the graph and FSM in your final report in the design section under System level design (3.3)

## Design Presentation
Your team will present both the ROS graph and Finite State Machine in class in a presentation taking no more than 10 minutes.

## Implementation
The robot will start at an undisclosed location within the large halls of DFEC. When traveling between two walls (the halls are approximately 4 meters wide), the robot should stay in the middle of the hall. There will be an AprilTag at the end of each hall providing turn commands:

- ID 0: turn left
- ID 1: turn right
- ID 2: turn around

You should stop halfway in the intersection (approximately 2 meters from the tag) and turn accordingly. Stop signs will be randomly placed throughout the halls. The robot should stop approximately 1 meter from the stop sign, wait five seconds, and then continue driving forward.

If at any time there is an open door in the hall, the robot should ignore the door and continue driving in the middle of the hall.

When the goal is discovered (AprilTag ID 3), the robot should stop within 0.5 meters from the goal and complete a 360 deg turn.

## Demonstration
Demonstrations will be accomplished on lesson 40 using a randomized course. Points will be deducted for failed checkpoints (e.g., does not stop and turn within approximately 2 meters of AprilTag 0). The final rubric is below and each item is worth **6 points** for a total of **60 points** assigned to the demonstration:

- Wall following
- Ignore doors
- AprilTag0: Turn left
- AprilTag1: Turn right
- AprilTag2: Turn around
- AprilTag3: Turn 360
- Stop halfway in intersectiom
- Stop 1.0 meters from stop sign
- Wait 5 seconds after stop sign detection
- Stop within 0.5 meters from AprilTag3

## Turn-in Requirements
**[10 points]** Design Presentation

**[60 points]** Demonstration

**[30 points]** Report and code

> ⚠️ **Warning:** The final project is 25% of your final grade!
