# Module 1: Robotics Operating System (ROS)
## In-Class Exercise 1 - ROS
---

**You must open this file as a Jupyter Notebook (link below) to run code**

[Run this file as an executable Jupyter Notebook](http://localhost:8888/notebooks/ICE1_ROS.ipynb)

Below you will see the ROS commands you will use throughout this course to investigate your ROS system and write your lab reports. The "!" character in the front allows us to run bash commands from Jupyter and would **NOT** be used in the command line.

With the talker and listener nodes running execute the below commands.

List all running nodes:


```python
!rosnode list
```

You should see the listener and talker nodes. The */rosout* node is created when running `roscore` and facilitates communication in the network. You can ignore this node in your lab reports.

Get more information about the */listener* node:


```python
!rosnode info /listener
```

You can see what topics the node is publishing and subscribing to. It publishes to the ROS log file (for debugging) and subscribes to the **/chat** topic.

List the active topics:


```python
!rostopic list
```

The first topic is the one we created. The last two are created by `roscore` and can be ignored.

Show information about the **/chat** topic such as what type of messages are sent over the topic and publishing and subscribing nodes.




```python
!rostopic info /chat
```

As expected, the *talker* node is publishing to the **/chat** topic while the *listener* node subscribes.

Display running nodes and communication between them:


```python
!rqt_graph
```

Close the rqt_graph.

Display information about the message that is sent over the **/chat** topic.


```python
!rostopic type /chat | rosmsg show
```

The output of the command is the same as the information we saw from the ROS documentation. Again, to access the message we have to use the `data` attribute.

Display messages sent over the **/chat** topic:


```python
!rostopic echo /chat
```

In the ICE1_Talker notebook send a message to the listener. You should see that message show up both here and at the listener. This echo tool is useful to ensure your nodes are sending the messages as expected.

### Checkpoint
Once complete, get checked off by an instructor showing the output of each of the above.

## Cleanup
In each of the notebooks reset the Jupyter kernel and clear output (at the top menu bar select "Kernel" and "Restart & Clear Output"). Close each notebook. Shutdown the notebook server by typing `ctrl+c` within the terminal you ran `jupyter-notebook` in. Select 'y'.
