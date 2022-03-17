# Module 5: Custom Messages
## In-Class Exercise 5
---

**You must open this file as a Jupyter Notebook (link below) to run code**

[Run this file as an executable Jupyter Notebook](http://localhost:8888/notebooks/ICE5_CustomMessages.ipynb)


### A note on this document
Now that you have a better understanding of the Linux operating system and Python programming language the Jupyter Notebooks will be used primarily to guide you through the In-Class Exercises and Laboratories. You will execute the majority of your commands and code within the Linux terminal.

### Purpose
This In-Class Exercise will provide you more insight into ROS messages and how information is passed between two nodes. A node can publish specific messages over a topic and other nodes are able to subscribe to that topic to receive the message. The format of these messages must be pre-defined and each node needs to know the format of the message. ROS provides a number of pre-built messages, but also allows for developers to create custom messages. In this lesson you will learn the method for and practice creating custom messages. In the corresponding lab you will develop a custom message to drive the robot. The custom message will eventually be used to enable a controller to drive the robot based on multiple data sources (e.g., IMU, LIDAR, keyboard).

### ROS msgs
#### msg
ROS utilizes a simplified message description language to describe data values that ROS nodes publish. There are a lot of built-in messages that are widely used by ROS packages ([common_msgs](http://wiki.ros.org/common_msgs?distro=noetic)). THe *geometry_msgs* package is one example of a pre-built message which provides the *Twist* message type used to drive the robot in the previous ICE.

#### Custom messages
If a pre-built message type does not meet the needs of a system, custom messages can be created. A custom message is created using a `.msg` file, a simple text file that descries the fields of a ROS message. The *catkin_make* tool uses the message file to generate source code for messages. The `.msg` files are stored in the **msg** directory of a package. There are two parts to a `.msg` file: fields and constants. Fields are the data that is sent inside of the message. Constants define useful values that can be used to interpret those fields. We will primarily use fields.

#### Fields
Each field consists of a type and a name, separated by a space per line. The field types you can use are:

- int8, int16, int64 (plus uint*)
- float32, float64
- string
- time, duration
- other msg files
- variable length array[] and fixed-length array[X].

#### Header
The header contains a timestamp and coordinate frame information that are commonly used in ROS. You will frequently see the first line in a `.msg` file have a header.

#### Format
    Header header
    fieldtype fieldname
    
For example, if we created a custom message file titled `Person.msg` that describes a person it might look like this:

    Header header
    string firstname
    string lastname
    int32 age
    
#### Importing messages
To utilize a *msg* in a node it must first be imported into the script.

```python
    from geometry_msgs.msg import Twist
    from ice5.msg import Person
```
>**Syntax:** `from <package>.msg import msg`

#### Using messages
After importing the *msg* you can access the fields similar to any object. For example, if we created a class instance variable to store our Person message, `person = Person()`, you would then access the fields like using the field names of the *msg*:

```python
    print("%s %s is %d years old!" % (person.firstname, person.lastname, person.age))
```

If you wanted to set the linear x and angular z speeds of a *Twist* message before publishing it to the USAFABOT you would again use the field names provided by the pre-built message. If you googled the Twist message ([Twist Message](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/Twist.html), you would see the contents of the Twist message include two other messages, linear and angular, of type Vector3:

    geometry_msgs/Vector3 linear
    geometry_msgs/Vector3 angular
    
Each of those two messages include the same fields and fieldnames:

    float64 x
    float64 y
    float64 z
    
To set the linear x and angular z values, we have to access those fields using an objected oriented method. For example:

```python
    from gemoetry_msgs.msg import Twist
    
    # create a publisher to send Twist messages over the cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    
    # set the Twist message to drive the robot straight at 0.25 m/s
    bot_cmd = Twist()
    bot_cmd.linear.x = 0.25  # notice you have to access both the "linear" & "x" fields of the Twist message
    bot_cmd.angular.z = 0.0
    
    # publish the Twist message
    pub.publish(bot_cmd)
```

### ICE 5
In this exercise you will create a custom message that describes a person. This message will provide two strings, first and last name, and an integer age for a person. We will then create a node that publishes information about that person and a node that subscribes to that information.

#### Create the custom message:
1. In a new terminal on the **Master**, create an **ice5** package which depends on the *std_msgs* package and *rospy* package, compile and source the ws:
 
    ```bash
    pi@master: cd ~/master_ws/src/ece495_master_spring2022-USERNAME/
    pi@master: catkin_create_pkg ice5 std_msgs rospy
    pi@master: cd ~/master_ws
    pi@master: catkin_make
    pi@master: source ~/.bashrc
    ```
1. Change directory to the package folder and create a *msg* directory:

    ```bash
    pi@master: roscd ice5
    pi@master: mkdir msg
    pi@master: cd msg
    ```
    
1. Create the *msg* file for the Person and add the fields previously discussed (header, firstname, lastname, and age):

    ```bash
    pi@master: nano Person.msg
    ```
    
1. Save and exit: `ctrl+s`, `ctrl+x`

#### Write the Publisher
1. Create the file for the publisher:

    ```bash
    pi@master: roscd ice5/src
    pi@master: touch ice5_publisher.py
    ```
    
1. Copy the below code to the ice5_publisher.py file and fill in the required lines (look for the TODO tag). You can edit via the terminal using nano, but it is often easier to use a GUI editor. Browse to the publisher in the file browser and double-click. This will open the file in thonny (if it is open in any other editor, stop, raise your hand, and get help from an instructor)

```python
#!/usr/bin/env python3
import rospy
from ice5.msg import Person # import the message: from PKG.msg import MSG

class Talker:
    """Class that publishes basic information about person"""
    def __init__(self, first = "Cadet", last = "Snuffy", age = 21):
        self.msg = Person()         # creates a Person message
        self.msg.firstname = first  # assign the firstname field
        self.msg.lastname = last    # assign the lastname field
        self.msg.age = age          # assign the age field
        
        # TODO: create the publisher that publishes Person messages over the person topic
        # Since we don't care about losing messages we can set the queue size to 1
        self.pub =
        
        # TODO: create a timer that will call the callback_publish() function every .1 seconds (10 Hz)
        rospy.Timer()
        
        # nicely handle shutdown (ctrl+c)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        
    def callback_publish(self, event):
        if not self.ctrl_c:
            # TODO: publish the msg
            
            
    def shutdownhook(self):
    	print("Shutting down publisher.")
    	self.ctrl_c = True
    	
if __name__ == '__main__':
    rospy.init_node('talker')
    
    # create an instance of the Talker class changing the class variables
    Talker("Steven", "Beyer", 33)
    rospy.spin()	# run forever until ctrl+c    
```

3. Save and exit.

4. Make the node executable.

#### Write the Subscriber
1. Create the file for the subscriber:

    ```bash
    pi@master: touch ice5_subscriber.py
    ```

1. Copy the below code to the ice5_subscriber.py file and fill in the required lines (look for the TODO tag).

```python
#!/usr/bin/env python3
import rospy
from ice5.msg import Person # import the message: from PKG.msg import MSG

class Listener:
    """Listener class that prints information about person"""
    def __init__(self):
    	# TODO: create the subscriber that receives Person messages over the person topic
    	# and calls the callback_person() function.
        
        
        # nicely handle shutdown (Ctrl+c)
        rospy.on_shutdown(self.shutdownhook)
        
    def callback_person(self, person):
    	# TODO: print the information about the person
        
        
    def shutdownhook(self):
    	print("Shutting down subscriber.")
        
if __name__ == '__main__':
    rospy.init_node('listener')
    # create an instance of the class
    Listener()
    # keeps python from exiting until this node is stopped
    rospy.spin()
```

3. Save and exit.

4. Make the node executable.

#### Requirements to use custom messages.
There are a number of settings that have to be set within the `package.xml` and `CMakeLists.txt` files that tell catkin to compile the messages.

##### package.xml
1. Edit `package.xml` (`rosed ice5 package.xml`) and uncomment these two lines (remove arrows on both sides of the line):

    ```
    <build_depend>message_generation</build_depend>
    exec_depend>message_runtime</exec_depend>
    ```
    
1. Save and exit.

##### CMakeLists.txt
1. Edit `CMakeLists.txt` (`rosed ice5 CMakeLists.txt`) and make the following changes:

    1. Add the `message_generation` dependency to the `find_package` call so that you can generate messages:
    
        ```python
        # Do not just add this to your CMakeLists.txt, modify the existing text to 
        # add message_generation before the closing parenthesis
        find_package(catkin REQUIRED COMPONENTS
            rospy
            std_msgs
            message_generation
        )
        ```
    
    1. Find the following block of code:
        
        ```python
        ## Generate messages in the 'msg' folder
        # add_message_files(
        #    FILES
        #    Message1.msg
        #    Message2.msg
        #)
        ```
        
        and uncomment by removing the `#` symbols and then replace the `Message*.msg` files with your `.msg` file, such that it looks like this:
        
        ```python
        add_message_files(
            FILES
            Person.msg
        )
        ```

    1. Find the following block of code:
    
        ```python
        # generate_messages(
        #    DEPENDENCIES
        #    std_msgs
        #)
        ```
        
        and uncomment so it looks like:
        
        ```python
        generate_messages(
            DEPENDENCIES
            std_msgs
        )
        ```
        
    1. Uncomment and add the export message runtime dependency to the `CATKIN_DEPENDS` line in the `catkin_package()` function near the bottom without changing anything else:
        
        ```python
        catkin_package(
            ...
            CATKIN_DEPENDS rospy std_msgs message_runtime
            ...
        )
    
    1. Save and exit.

#### Compile and run the code
1. Make and source your package:

    ```bash
    pi@master: cd ~/master_ws
    pi@master: catkin_make
    pi@master: source ~/.bashrc
    ```
    
1. Run roscore!
    
1. The `rospy` tool can measure certain statistics for every topic connection. We can visualize this in `rqt_graph`, but we have to enable it after `roscore`, but before any nodes. In a new terminal run the following to enable statistics:

    ```bash
    pi@master: rosparam set enable_statistics true
    ```
    
1. Run the publisher:

    ```bash
    pi@master: rosrun ice5 ice5_publisher.py
    ```
    
1. In a new terminal, run the subscriber:

    ```bash
    pi@master: rosrun ice5 ice5_subscriber.py
    ```
    
1. In a new terminal observe the nodes running:

    ```bash
    pi@master: rosnode list
    ```
    
1. Observe information about each node:

    ```bash
    pi@master: rosnode info /talker
    ```
    
1. Observe how information is being passed:

    ```bash
    pi@master: rosrun rqt_graph rqt_graph
    ```
    
    > 📝️ **Note**: You may have to hit refresh a few times to get the statistics previously mentioned.
    
1. Observe all active topics:

    ```bash
    pi@master: rostopic list
    ```
    
1. Observe the information about the message sent over the topics (repeat for each topic, remember we do not care about topics we did not create (e.g., rosout, rosout_agg, statistics)).

    ```bash
    pi@master: rostopic info person
    ```
    
1. Observe the fields of the message sent over each topic:

    ```bash
    pi@master: rostopic type person | rosmsg show
    ```

### Checkpoint
Once complete, get checked off by an instructor showing the output of each of the above.

### Summary
In this lesson you learned about ROS pre-built and custom messages! You created your own message which described a Person. You then developed a publisher to send information about the Person to a subscriber.

### Cleanup
In each terminal window, close the node by typing `ctrl+c`. Exit any SSH connections. Shutdown the notebook server by typing `ctrl+c` within the terminal you ran `jupyter-notebook` in. Select 'y'.

**Ensure roscore is terminated before moving on to the lab.**

## Lab 1
---

**You must open this file as a Jupyter Notebook (link below) to run code**

[Run this file as an executable Jupyter Notebook](http://localhost:8888/notebooks/Lab1_CustomMessages.ipynb)


### Purpose
This lab will provide practice in creating custom messages. You will take the **cmd_vel** topic and *Twist* message from the **teleop_twist_keyboard** node and use a **controller** node to send *USAFABOT_Cmd* messages to the **usafabot_serial** node. As we introduce new subsystems to the robot, the **controller** node will determine priority between these sensors/systems and then drive the robot using the **usafabot_cmd** topic accordingly.

### Robot
#### Setup:
1. In a new terminal on the **Master**, create a secure shell connection to your **Robot**.

1. Create a custom message within the **usafabot** package's **msg** folder that is called *USAFABOT_Cmd.msg* and has two fields and a header:

    ```python
    Header header
    float64 lin_x
    float64 ang_z
    ```

1. Edit **package.xml** per the ICE instructions (don't remove anything from the file, just add what is needed to run your custom message).

1. Edit **CMakeLists.txt** per the ICE instructions (don't remove anything from the file, just add what is needed to run your custom message).

1. Make and source your workspace.

**Note**: This message is now part of the **usafabot** package and if you ran `rosmsg show USAFABOT_Cmd` it would return *usafabot/USAFABOT_Cmd*.

#### usafabot_serial.py
1. Edit **usafabot_serial.py** within the **usafabot** package (`rosed usafabot usafabot_serial.py`) to:

    - Import the new message instead of *Twist*
    - Subscribe to the **usafabot_cmd** topic instead of **cmd_vel** and use the new message instead of *Twist*
    - Edit the `callback_write()` function to use the new message fields.
    
> 💡️ **HINT**: Look for the tag "TODO" within the python file.

### Master 
#### Setup:
To use the same custom message on two different machines the custom message must be created and part of the same package on both machines. On the **Robot**, the *USAFABOT_Cmd* message is part of the **usafabot** package. Therefore, the message must be part of the **usafabot** package on the **Master**. When communicating, ROS uses the **whole** name of the msg, *usafabot/USAFABOT_Cmd*, so these must be the same on each system. There should already be a **usafabot** package on your master.

1. In a terminal on the **Master**, browse to the **usafabot** package.

1. Create a custom message within the **usafabot** package's **msg** folder that is called *USAFABOT_Cmd.msg* and has two fields and a header:

    ```python
    Header header
    float64 lin_x
    float64 ang_z
    ```

1. Edit **package.xml** per the ICE instructions (don't remove anything from the file, just add what is needed to run your custom message).

1. Edit **CMakeLists.txt** per the ICE instructions (don't remove anything from the file, just add what is needed to run your custom message).

1. In the `/master_ws/src/ece495_master_spring2022-USERNAME/` folder, create a **lab1** package which depends on **std_msgs**, **rospy**, **geometry_msgs**, and **usafabot**.

1. Make and source your workspace.

#### controller.py
1. Create a **controller.py** node within the src folder of **lab1**.
1. Edit the file and import **rospy**, **Twist**, and the new message, **USAFABOT_Cmd**.
1. Create a Controller class that will have four functions:

    ```python
    class Controller:
            # class initialization
            def __init__(self):
                # TODO: complete __init__() function
                
            # callback called when a new message is received over 
            # the cmd_vel topic
            def callback_keyboard(self, kb):
                # TODO: complete callback_keyboard() function

            # callback that will run at 100 Hz
            # will be used to control the robot
            def callback_controller(self, event):
                # TODO: complete callback_controller() function
                
            # function to ensure all values are zeroed out when shut down
            def shutdownhook(self):
                # TODO: complete shutdownhook() function
    ```
                
1. Add the following to the `__init__()` function:
    
    - Instance variable, `self.cmd`, to store the `USAFABOT_CMD()` message.
    - Instance variable, `self.kb`, to store the `Twist()` message sent from **teleop_twist_keyboard**.
    - A subscriber to the **cmd_vel** topic with a callback to the `callback_keyboard() function.
    - A publisher to the **usafabot_cmd** topic.
    - A timer that runs at 100 Hz with a callback to the `callback_controller()` function.
    - The `ctrl_c` boolean instance variable.
    - The shutdown method: `rospy.on_shutdown(self.shutdownhook)`.
    <br>
    
1. Write the `callback_keyboard()` function that will copy the message received (kb) to the keyboard instance variable. Remember, `callback_keyboard()` is called every time a new *Twist* message is sent over the **cmd_vel** topic. The *Twist* message is passed to the function as the second input (kb).

1. Write the `callback_controller()` function that when `ctrl_c` is not pressed it sets the `lin_x` and `ang_z` characteristics of the *USAFABOT_Cmd* message to the `linear.x` and `angular.z` characteristics of the *Twist* message and publishes the message. For example:

    ```python
    self.cmd.lin_x = self.kb.linear.x
    ```
    where the left side of the expression is our custom *USAFABOT_Cmd* message and the right side is the *Twist* message sent over the **cmd_vel** topic (you can see the format of the *Twist* message in the ROS documentation: [Twist Message](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/Twist.html).
    
1. Write the shutdownhook() function to print "Controller exiting. Halting robot.", set `self.ctrl_c` to *True*, and publish a message over the **usafabot_cmd** topic with a linear x and angular z of 0 so the robot will stop.

1. Create a `__main__()` function that initializes the controller node (node names are lower case), creates an instance of the Controller class, and then spins (runs forever).

### Run your nodes
1. On the **Master** open a terminal and run **roscore**.
1. Open another terminal and enable statistics for **rqt_graph**.
1. Run the controller node.
1. Run the **teleop_twist_keyboard** node.
1. Using the terminal with the secure shell into the **Robot** and run the **usafabot_serial.py** node.
1. Use the keyboard to drive the robot (operation should be exactly as before and the robot should respond accordingly).

### Report
Complete a short 2-3 page report that utilizes the format and answers the questions within the report template. The report template and an example report can be found within the Team under `Resources/Lab Template`.

### Turn-in Requirements
**[25 points]** Demonstration of keyboard control of USAFABOT (preferably in person, but can be recorded and posted to Teams under the Lab1 channel).

**[50 points]** Report via Gradescope.

**[25 points]** Code: push your code to your repository. Also, include a screen shot of the **controller.py** and **usafabot_serial.py** files and submit at the end of your report.

