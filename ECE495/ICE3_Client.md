# Module 3: Python3 for Robotics
## In-Class Exercise 3 - Client
---

**You must open this file as a Jupyter Notebook (link below) to run code**

[Run this file as an executable Jupyter Notebook](http://localhost:8888/notebooks/ICE3_Client.ipynb)


### A note on this document
This document is known as a Jupyter Notebook; it is used in academia and industry to allow text and executable code to coexist in a very easy to read format. Blocks can contain text or code, and for blocks containing code, press `Shift + Enter` to run the code. Earlier blocks of code need to be run for the later blocks of code to work.

### Purpose
This Jupyter Notebook will allow you to practice some of the techniques you have learned over the last few modules. You will develop an advanced chat client and server (similar to ICE1) enabling a client to send a message and a server to respond accordingly.

#### Initialize ROS:
The first step when utilizing  ROS is to initialize *roscore*. There are two methods to accomplish this: first, by explicitly running *roscore* and second, by running a launch file (which will initialize *roscore* if it is not already running). During the first portion of this course we will explicitly run *roscore* and then take advantage of launch files later in the course.

Copy the following code and run it in a new terminal (use the shortcut `ctrl+alt+t` to open a new terminal window or select an open terminal and hit `ctrl+shift+t` to open a new terminal tab):

`roscore`

### Implementing the chat client
**NOTE**: This Jupyter Notebook will require you to enter Python3 code within code sections. You can type any Python3 code and expand the block if necessary. After typing the code, execute the code block before moving forward.

#### Import modules

**Important**: Importing classes may take some time. You will know the code block is still executing as the bracket on the left of the cell will change to a `*` character. Do not move to the next step until the `*` is gone.


```python
# import required modules for ROS and the String message from std_msgs


```

#### Client Class
1. Create a Client class with a dictionary used to map numbers to messages.
2. Initialize the class with the following:
    1. an instance variable to store the String message
    2. a publisher that publishes String messages on the client topic
    3. a subscriber to the server topic which receives String messages and calls a callback when messages are sent.
    4. a timer that runs every second and calls a class method
    5. nicely handle shutdown
3. Create the callback input class method that is ran every second and has the user pick a message to send.
4. Create the callback received class method that is called every time a message is received from the server.
5. Handle shutdown.


```python
class Client:
    MESSAGE = {1: "Hello!", 2: "How are you?", 3: "Where are you from?",
               4: "What are you doing today?"}
    
    def __init__(self):
        # 2.A

        # 2.B
        
        # 2.C
        
        # 2.D
        
        # 2.E nicely handle shutdown
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def callback_input(self, event):
        valid = False
        while not valid and not self.ctrl_c:
            # get input from user (you must inform them their options)
            
            try:
                # convert to int, if not number throw ValueError
                val = int(chat_str)
                # check if valid number, if valid then access
                # that entry in the dictionary, publish the message
                # and set valid to True; if not valid, print error
                # message to user (make this error message useful)
                
                
                
                
                
            except ValueError:
                # print error message to user (make 
                # this error message useful)
                
                
    def callback_received(self, msg):
        # print message sent to the server
        
        # print the response from the server
    
    # handle shutdown
    def shutdownhook(self):
        print("Shutting down the client")
        self.ctrl_c = True
```

#### Main
The main function calls initializes our node, creates an instance of the client class, then runs forever.


```python
def main():
    # initialize node
    
    # create an instance of the client class
    
    # run forever
    rospy.spin()
```

#### Run the client


```python
main()
```

#### Create the server
At this point the client is waiting for user input. Don't start typing yet, though! We need to implement and run our server. Open the [ICE3_Server](ICE3_Server.ipynb). notebook and follow the instructions.

### ROS commands
Open the [ICE3_ROS](ICE3_ROS.ipynb) notebook and follow the instructions.
