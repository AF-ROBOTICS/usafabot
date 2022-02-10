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

## Summary
In this lesson you learned about ROS pre-built and custom messages! You created your own message which described a Person. You then developed a publisher to send information about the Person to a subscriber.

## Cleanup
In each terminal window, close the node by typing `ctrl+c`. Exit any SSH connections. Shutdown the notebook server by typing `ctrl+c` within the terminal you ran `jupyter-notebook` in. Select 'y'.

**Ensure roscore is terminated before moving on to the lab.**
