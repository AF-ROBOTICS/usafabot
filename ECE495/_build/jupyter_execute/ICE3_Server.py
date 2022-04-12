#!/usr/bin/env python
# coding: utf-8

# # Module 3: Python3 for Robotics
# ## In-Class Exercise 3 - Server
# ---

# ### Implementing the chat server
# **NOTE**: This Jupyter Notebook will require you to enter Python3 code within code sections. You can type any Python3 code and expand the block if necessary. After typing the code, execute the code block before moving forward.
# 
# #### Import modules
# 
# > ⚠️ **Important:** Importing classes may take some time. You will know the code block is still executing as the bracket on the left of the cell will change to a `*` character. Do not move to the next step until the `*` is gone.

# In[1]:


# import required modules for ROS and the String message from std_msgs


# #### Server Class
# 1. Create a server class with a dictionary used to map messages to responses (one for each message from the client).
# 2. Initialize the class with the following:
#     1. an instance variable to store the String message
#     2. a subscriber to the client topic which receives String messages and calls a callback called received.
#     3. a publisher to the server topic which sends String messages 
#     4. nicely handle shutdown
# 3. Create the callback received class method that is called every time a message is received from the client.
# 5. Handle shutdown.

# In[2]:


class Server:
    # class dictionary storing server responses
    MESSAGE = 
    
    def __init__(self):
        # 2.A

        # 2.B
        
        # 2.C
        
        # 2.D nicely handle shutdown
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def callback_received(self, msg):
        # print the message from the client
        
        # print the response that will be sent to the client
        
        # publish the response
    
    # handle shutdown
    def shutdownhook(self):
        print("Shutting down the server")
        self.ctrl_c = True


# #### Main
# The main function calls initializes our node, creates an instance of the server class, then runs forever.

# In[ ]:


def main():
    # initialize node
    
    # create an instance of the server class
    
    # run forever
    rospy.spin()


# #### Run the program

# In[ ]:


main()


# At this point, the server is waiting for the client to send a message. Browse back to your client and type a message! You should see that message show up above.
