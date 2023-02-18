import rospy
import cv2
import smach
import smach_ros
import actionlib

import numpy as np
# import torch

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image # The input
from geometry_msgs.msg import Pose
from darknet_ros_msgs.msg import BoundingBoxes
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from functions.data_storage_functions import *
import communication_functions as cf


# Define the state machine
class ProcessImage(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['image_processed', 'image_not_processed'], 
                             input_keys=['stateInfo'],
                             output_keys=['stateInfo'])

    def execute(self, userdata):
        rospy.loginfo("Executing PROCESS_IMAGE")
        try:
            userdata.stateInfo.print_data()
        except:
            userdata.stateInfo = StateInformation()
            rospy.logerr("This error is fine if you are running the dummy")
        
        # trying to look down
        try:
            look_down()
            
        except:
            rospy.logwarn('Looking down is unsuccessfull is unsucsessfull')
            return 'image_not_processed'

        # trying to process the information
        try:
            # image = cf.get_from_topic("/xtion/rgb/image_raw", Image)
            #  image returns an object

            image_msg = cf.get_from_topic("/xtion/rgb/image_raw", Image) # the input

            image_msg_depth = cf.get_from_topic('/xtion/depth_registered/image_raw', Image) #takes depth image data calling it Image

            image = image_msg.data
            image_2 = image_msg_depth.data  # use for depth 
            rospy.loginfo(' userdata.image recieved  %s', type(image))


            # TODO Ploy and Charlotte please work on your code here and return a Pose object


            pose = Pose()

            bb_msg = cf.get_from_topic("/darknet_ros/bounding_boxes", BoundingBoxes )

            #### Should not have the [0] if because there are more object on teh table
            # make sure you hande this case somehow
            # also store the numebr of cloth object that are on the table so that bence can store that for later use
            xmin = bb_msg.bounding_boxes[0].xmin
            ymin = bb_msg.bounding_boxes[0].ymin
            xmax = bb_msg.bounding_boxes[0].xmax
            ymax = bb_msg.bounding_boxes[0].ymax

            rospy.loginfo("The xmin is cropped: " + str(xmin))
            rospy.loginfo("The ymin is cropped: " + str(ymin))
            rospy.loginfo("The xmax is cropped: " + str(xmax))
            rospy.loginfo("The ymax is cropped: " + str(ymax))
            

            cropped_image = image[ymin:ymax,xmin:xmax,:] # likely to cause problems
            cropped_image_depth = image_2[ymin:ymax,xmin:xmax,:] # likely to cause problems

            rospy.loginf("Cropped image is working")


            image_callback(cropped_image,cropped_image_depth)
           

            rospy.loginfo("The image is cropped: ")

            


            # rospy.loginfo(' bounding_boxes recieved  %s', bounding_boxes)

            userdata.stateInfo.add_to_past_actions("Image is processed. ")

            return 'image_processed'
           
        except:
            rospy.logerr('Image processing is unsucsessfull')
            userdata.stateInfo.add_to_past_actions("Image is NOT processed. ")
            userdata.stateInfo.set_is_aborted(True)
            return 'image_not_processed'
        
def image_callback(data,data_2):
        try:
            cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        
        #Convert image from BGR to RGB
        RGB_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        Gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) # uses gray image to get centroid 
        ret, thresh = cv2.threshold(gray, 127, 255, 0) 
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnt = contours[0]
        X = cv2.moments(cnt)
        cent_x = int(X['m10']/X['m00'])
        cent_y = int(X['m01']/X['m00'])
        

        pixels = np.float32(RGB_image.reshape(-1, 3))
        n_colors = 5
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, .1)
        flags = cv2.KMEANS_RANDOM_CENTERS
        _, labels, palette = cv2.kmeans(pixels, n_colors, None, criteria, 10, flags)
        _, counts = np.unique(labels, return_counts=True)
        dominant = palette[np.argmax(counts)]

        # Print the most common color
        print("The dominant is: {}".format(dominant))
        image_callback_depth(cent_x,cent_y,data_2)


def image_callback_depth(central_x,central_y,image):
    depth_value = image[central_x, central_y]
    return depth_value



def look_down():
    client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
    goal.trajectory.points.append(JointTrajectoryPoint())
    goal.trajectory.points[0].positions = [0., -0.5]
    goal.trajectory.points[0].time_from_start = rospy.Duration(1.0)
    client.send_goal(goal)
    client.wait_for_result()