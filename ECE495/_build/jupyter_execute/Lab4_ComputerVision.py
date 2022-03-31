#!/usr/bin/env python
# coding: utf-8

# # Lab 4: Computer Vision
# ---

# ## Purpose
# This lab will integrate a USB Camera with the Robot. You will use a Python script to take pictures of the stop sign and build a stop sign detector then test it using a live video feed. You will then use the detector and known size of the stop sign to estimate how far the stop sign is from the camera. Lastly, you will create a node to identify and determine how far an April Tag is from the robot.

# ## Setup packages
# Open a terminal on the **Master** and create a lab4 package:
# 
# ```bash
# cd ~/master_ws/src/ece495_master_spring2022-USERNAME/
# catkin_create_pkg lab4 rospy sensor_msgs std_msgs cv_bridge apriltag_ros
# ```
# 
# Make and source your workspace.
# 
# If you have not already done so, repeat on the **Robot**

# ## Create a ROS node to save images
# Browse to your lab4 source folder on the **Master** and create a node called **image_capture.py**.
# 
# ```python
# #!/usr/bin/env python3
# import rospy, cv2, argparse
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# 
# class SavingImage(object):
# 	def __init__(self, img_dest):
# 		self.img_dest = img_dest
# 		self.ctrl_c = False
# 		self.count = 0
#         
#         # subscribe to the topic created by the usb_cam node
# 		self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.camera_callback)
#         
#         # CV bridge converts between ROS Image messages and OpenCV images
# 		self.bridge_object = CvBridge()
#         
#         # callback to save images when user presses button
# 		rospy.Timer(rospy.Duration(.1), self.callback_save)
# 		
# 		rospy.on_shutdown(self.shutdownhook)
# 
# 	def camera_callback(self,img):
# 		if not self.ctrl_c:
# 			try:
#                 # convert ROS image to OpenCV image
# 				self.cv_image = self.bridge_object.imgmsg_to_cv2(img, desired_encoding="bgr8")
# 			except CvBridgeError as e:
# 				print(e)
# 			
#             # show the image (waitKey(1) allows for automatic refressh creating video)
# 			cv2.imshow('image', self.cv_image)
# 			cv2.waitKey(1)
# 		
# 	def callback_save(self, event):
#         # when user is ready to take picture press button
# 		_ = input("Press enter to save the next image.")
# 		dest = self.img_dest + "img" + str(self.count) + ".jpg"
# 		self.count += 1
# 		print(dest)
# 		try:
#             # write to file
# 			cv2.imwrite(dest, self.cv_image)
# 		except:
# 			print("Not valid image name. Try restarting with valid path.")
# 			
# 	def shutdownhook(self):
# 		print("Shutting down")
# 		cv2.destroyAllWindows()
# 
# if __name__ == '__main__':
# 	rospy.init_node('image_saver', anonymous=True)
# 	ap = argparse.ArgumentParser()
# 	ap.add_argument("-o", "--output", required=True, help="path to output img")
# 	args = vars(ap.parse_args())
# 	saving_image_object = SavingImage(args["output"])
# 	try:
# 		rospy.spin()
# 	except KeyboardInterrupt:
# 		pass
# ```
# 
# Save, exit, and make executable.

# ## Train your stop detector
# 
# Create a new folder in your **lab4** package called **training_images**.
# 
# Run the `image_capture.py` node on the **Master** using the following command:
# 
# ```bash
# rosrun lab4 image_capture.py -o /home/dfec/master_ws/src/ece495_master_spring2021-NAME/lab4/training_images/
# ```
# 
# Store images of the stop sign by pressing `enter` when prompted. You decide how many and at what orientations to properly train your detector. When complete, hit `ctrl+c` to exit.
# 
# Utilize the steps from Module 9: [Building a detector using HOG features](ICE9_ComputerVision.ipynb#Building-a-detector-using-HOG-features) to label your images and train your object detector using the new images, saving the `stop_detector.svm` file within the **training_images** folder.

# ## Test your stop detector
# Create a node in the **lab4** package on the **Master** called `stop_detector.py` and copy the below into it:
# 
# ```python
# #!/usr/bin/env python3
# import rospy, cv2, dlib
# from cv_bridge import CvBridge, CvBridgeError
# 
# # TODO: import usb_cam message type
# 
# 
# class StopDetector(object):
# 
#     def __init__(self, detectorLoc):
#         self.ctrl_c = False
#         
#         #TODO: create subscriber to usb_cam image topic
# 
#         
#         self.bridge_object = CvBridge()
#         self.detector = dlib.simple_object_detector(detectorLoc)
#         
#         rospy.on_shutdown(self.shutdownhook)
#         
#     def camera_callback(self,data):
#         if not self.ctrl_c:
#             #TODO: write code to get ROS image, convert to OpenCV image,
#             # apply detector, add boxes to image, and display image
#             
# 
#     def shutdownhook(self):
#         print("Shutting down")
#         self.ctrl_c = True
#         cv2.destroyAllWindows()
#         
# if __name__ == '__main__':
#     rospy.init_node('stop_detector')
#     detector = rospy.get_param("/stop_detector/detector")
#     stop_detector = StopDetector(detector)
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         pass
# ```
# 
# Edit the `stop_detector.py` node so it utilizes the `camera_callback()` function we used above to get images from the camera.
# 
# After getting the `cv_image` within the `camera_callback()`, apply the detector in a similar method as Module 9: [Testing a detector](ICE9_ComputerVision.ipynb#Testing-a-detector) creating boxes around all detected stop signs. Using a `waitKey(1)` will allow for the image to refresh automatically without user input and display the video.

# ## Checkpoint 1
# Demonstrate the stop detector on the **Master** detecting a stop sign from the **Robot's** camera.
# 
# ```bash
# rosrun lab4 stop_detector.py _detector:=/home/dfec/master_ws/src/ece495_master_spring2021-NAME/lab4/training_images/stop_detector.svm
# ```
# 
# >📝️ **Note:** You must have the `lab4.launch` file running on the **Robot**.

# ## Move detector to robot
# Copy the detector and node to the robot:
# 
# ```bash
# roscd lab4/training_images
# scp stop_detector.svm pi@robotX:/home/pi/robot_ws/src/ece495_robot_spring2021-NAME/lab4/training_images/stop_detector.svm
# roscd lab4/src
# scp stop_detector.py pi@robotX:/home/pi/robot_ws/src/ece495_robot_spring2021-NAME/lab4/src/stop_detector.py
# ```
# 
# Remove the lines that display the video and instead print "Stop detected" if `boxes` is not empty.
# 
# Do you note a difference in processing speed?

# ## Launch file
# Edit the `lab4.launch` file on the **Robot** so it will run the stop detector node with the `detector` param set to the location of the detector. For example:
# 
# ```xml
# <node name="stop_detector" pkg="lab4" type="stop_detector.py" output="screen">
#     <param name="detector" value="/home/pi/robot_ws/src/ece495_robot_spring2021-Name/robot/lab4/training_images/sl_detector.svm"/>
# </node>
# ```

# ## Checkpoint 2
# Demonstrate the stop detector on the **Robot** detecting a stop sign.

# ## Determine distance from stop sign
# 
# ### Edit `stop_detector.py`
# 
# You will edit your stop sign detector on the **Robot** to calculate an estimated distance between the camera and the stop sign using triangle similarity. 
# 
# Given a stop sign with a known width, $W$, we can place the stop sign at a known distance, $D$, from our camera. The detector will then detect the stop sign and provide a perceived width in pixels, $P$. Using these values we can calculate the focal length, $F$ of our camera:
# 
# $F = \frac{(P\times D)}{W}$
# 
# We can then use the calculated focal length, $F$, known width, $W$, and perceived width in pixels, $P$ to calculate the distance from the camera:
# 
# $D' = \frac{(W\times F)}{P}$
# 
# Use the above information and create two class variables, `FOCAL` and `STOP_WIDTH`, and a class function to calculate distance given a known `FOCAL` length and a known width of the stop sign, `STOP_WIDTH`. You will need to print the perceived width of the stop sign to determine the $P$ value used in the calculation to find the focal length.
# 
# > 💡️ **Tip:** Pay attention to what the `x` and `w` variables of the `box` actually represent!
# 
# Create a new publisher that will publish the distance using **Float32** *std_msgs* messages over the */stop_dist* topic.
# 
# Publish the distance of each object seen in the image. 
# 
# Remove any print statements after troubleshooting!

# ## Checkpoint 3
# Demonstrate the **stop_detector** node publishing distance from the stop sign.

# ## Printing April Tag information
# 
# Create a node on the master in lab4 called `apriltag_dist.py`. Import the appropriate AprilTag message. Subscribe to the `tag_detections` topic. Print the identified AprilTag ID and distance. If the camera sees multiple tags, it should print the information for each tag.
# 
# In your callback function you will want to create a for loop such as:
# 
# ```python
# for tag in data.detections:
# ```
# 
# Use print statements to determine the characteristics of the message (you can also google the message).
# 
# Add the `apriltag_dist` node to the **lab4** launch file on the **Robot**.

# ## Checkpoint 4
# 
# Demonstrate the `apriltag_dist` node printing the ID and distance of each April Tag.

# ## Report
# Complete a short 2-3 page report that utilizes the format and answers the questions within the report template. The report template and an example report can be found within the Team under `Resources/Lab Template`.
# 
# > 📝️ **Note:** We will be primarily grading sections 3.1, 3.2, and 3.3 for this lab, but do include the entire lab as you will need other components for the final project report.

# ## Turn-in Requirements
# **[25 points]** All checkpoints marked off.
# 
# **[50 points]** Report via Gradescope.
# 
# **[25 points]** Code: push your code to your repository. Also, include a screen shot of the **controller.py** file at the end of your report.
