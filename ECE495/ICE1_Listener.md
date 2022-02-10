# Module 1: Robotics Operating System (ROS)
## In-Class Exercise 1 - Listener
---

**You must open this file as a Jupyter Notebook (link below) to run code**

[Run this file as an executable Jupyter Notebook](http://localhost:8888/notebooks/ICE1_Listener.ipynb)

### A note on this document
This document is known as a Jupyter Notebook; it is used in academia and industry to allow text and executable code to coexist in a very easy to read format. Blocks can contain text or code, and for blocks containing code, press `Shift + Enter` to run the code. Earlier blocks of code need to be run for the later blocks of code to work.

### Implementing the chat subscriber

#### Import modules

**Important**: Importing classes may take some time. You will know the code block is still executing as the bracket on the left of the cell will change to a `*` character. Do not move to the next step until the `*` is gone.


```python
# import required modules
import rospy
from std_msgs.msg import String
```

#### Listener Function
This function will create the subscriber ("listener") used to receive chat messages from the publisher ("talker").


```python
def listener():
    rospy.Subscriber('chat', String, callback_chat)
```

The above function creates the subscriber to the **/chat** topic. Every time a *String* message is sent over the topic the `callback_chat()` function is called. This is an interrupt that spins a new thread to call that function.

#### Callback function
The callback function will log and display what the chat listener sent.


```python
def callback_chat(message):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", message.data)
```

The callback function receives the *String* message as an input (you can name this parameter anything, but it is helpful if it is a meaningful variable name). To access the actual message, we need to utilize the data attribute of the *String* message. If you browse to the documentation for the [String Mesage](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html), you will note that the message attribute is called *data* and it is of type *string*. This is why we use the command `message.data`.

#### Main


```python
def main():
    rospy.init_node('listener')
    try:
        listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

The above is similar to the talker, but adds the `rospy.spin()` function call to create an infinite loop to allow the subscriber to operate in the background.

In an actual Python script we will replace 
```python
def main()
``` 
with 
```python
if __name__ == "__main__":
```
This allows our python files to be imported into other python files that might also have a main() function.

#### Run the listener


```python
main()
```

At this point, the subscriber is waiting for the publisher to send a message. Browse back to your talker and type a message! You should see that message show up above after hitting `enter` in the talker Notebook.
