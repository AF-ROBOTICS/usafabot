#!/usr/bin/env python
# coding: utf-8

# # Module 3: Python3 for Robotics
# ---
# ### A note on this document
# This document is known as a Jupyter Notebook; it is used in academia and industry to allow text and executable code to coexist in a very easy to read format. Blocks can contain text or code, and for blocks containing code, press `Shift + Enter` to run the code. Earlier blocks of code need to be run for the later blocks of code to work.

# ## Computer Player
# We will now implement a computer player node that will send a random choice to the player node.

# First import your required modules:

# In[1]:


import rospy, random
from std_msgs.msg import String


# Here, we are importing two modules: *rospy*, which allows us to run ROS code in Python, and the *String* message from the *std_msgs* ROS package.

# In[2]:


class Computer:
    # class constant to store computer choices
    CHOICES = ['Rock', 'Paper', 'Scissors']
    
    # initialize class
    def __init__(self,):
        # instance variables
        self.computers_choice = String()
        
        # subscriber to receive the computer's choice over the computer topic
        rospy.Subscriber('user_choice', String, self.callback_computers_choice)
        
        # publisher to send player's choice over the player topic
        self.pub = rospy.Publisher('computer_choice', String, queue_size=1)
    
    def callback_computers_choice(self, data):
        # use random module imported earlier to select a random index from the list
        rand_index = random.randint(0,len(self.CHOICES)-1)
        # use the index to select an item from the list
        self.computers_choice = self.CHOICES[rand_index]
        self.pub.publish(self.computers_choice)


# In[3]:


rospy.init_node('computer')
Computer()
rospy.spin()


# Now return to the ICE3_RPSUser Notebook and fill in the requested inputs.
