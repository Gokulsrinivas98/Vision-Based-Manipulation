# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# This code is modified by Berk Calli from the following author.
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from numpy.lib.function_base import average
import matplotlib.pyplot as plt
from tf2_ros import TransformException # Base class to handle exceptions
from tf2_ros.buffer import Buffer # Stores known frames and offers frame graph requests
from tf2_ros.transform_listener import TransformListener # Easy way to request and receive coordinate frame transform information
from std_msgs.msg import Float64MultiArray # Handle float64 arrays
from tf_transformations import quaternion_matrix

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.

    self.subscription = self.create_subscription(
                                            Image, 
                                            '/camera1/image_raw', 
                                            self.listener_callback, 
                                            10)
    self.subscription # prevent unused variable warning

    # Create the publisher. This publisher will publish an Float64MUltiArray
    # to the video_frames topic. The queue size is 10 messages.
    self.velocity_publisher_ = self.create_publisher(
                                            Float64MultiArray, 
                                            '/forward_velocity_controller/commands',
                                            10)

      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    #green,blue,meg,red
    self.f_r = np.array([639,398,557,383,574,303,655,317])
    self.gcx, self.rcx, self.bcx, self.mcx,self.gcy, self.rcy, self.bcy, self.mcy = ([] for i in range(8))
  
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data,'bgr8')
    self.colorThreshold(current_frame)

    # PLACE YOUR CODE HERE. PROCESS THE CURRENT FRAME AND PUBLISH IT. IF YOU ARE HAVING DIFFICULTY PUBLISHING IT YOU CAN USE THE FOLLOWING LINES TO DISPLAY IT VIA OPENCV FUNCTIONS
  def findCenters(self,img, disp):
    # Finding contour centers : 
    cnts, heirarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers =[]
    for c in cnts:
        if cv2.contourArea(c) <= 25:
            continue
        cx = int(average([x for x in c[:,0,0]]))
        cy = int(average([y for y in c[:,0,1]]))
        center = (cx, cy)
        centers.append(center)
    return centers

  def checkConvergence(self,err):
    x_sum, y_sum = 0, 0
    x_sum = sum([abs(i) for i in err[::2]])
    y_sum = sum([abs(i) for i in err[1::2]]) 
    if x_sum < 10 and y_sum < 10:
      return True
    else:
      return False
  
  def plot(self,convergence,val):
    if convergence:   
      plt.plot(self.gcx, self.gcy, color='g', label='Green Center')
      plt.plot(self.rcx, self.rcy, color='r', label='Red Center')
      plt.plot(self.bcx, self.bcy, color='b', label='Blue Center')
      plt.plot(self.mcx, self.mcy, color='m', label='Meg Center')
      plt.xlabel('x_coordinate')
      plt.ylabel('y_coordinate')
      plt.legend()
      plt.title("Trajectory of centers over time")
      plt.show()
      rclpy.shutdown()
    self.velocity_publisher_.publish(val)

  def masks(self,mask, img):
      imask = mask>0
      color = np.zeros_like(img, np.uint8)
      color[imask] = img[imask]
      return self.findCenters(mask, color)

  def colorThreshold(self,img):      
    # # Color thresholding!
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Green
    gmask = cv2.inRange(hsv, (36, 25, 25), (70, 255,255))
    green_centers=self.masks(gmask,img)
    self.gcx.append(green_centers[0][0])
    self.gcy.append(green_centers[0][1])
    # Blue
    bmask = cv2.inRange(hsv, (95, 25, 25), (135, 255,255))
    blue_centers=self.masks(bmask,img)
    self.bcx.append(blue_centers[0][0])
    self.bcy.append(blue_centers[0][1])
    # Red
    rmask = cv2.inRange(hsv, (0, 100, 25), (15, 255,255)) 
    red_centers=self.masks(rmask,img)
    self.rcx.append(red_centers[0][0])
    self.rcy.append(red_centers[0][1])
    #meg
    mmask = cv2.inRange(hsv, (140,100,20) , (170,255,255) )
    meg_centers=self.masks(mmask,img)
    self.mcx.append(meg_centers[0][0])
    self.mcy.append(meg_centers[0][1])
    #current features and error calculation
    f_c = np.array([green_centers,blue_centers,red_centers,meg_centers])
    f_c = f_c.flatten()# 8 x 1
    error = [0,0,0,0,0,0,0,0]
    error = f_c - self.f_r 
    print("error",error)
    self.findJ(error)

  def findJ(self,error,lmbda = -0.00225):
    L_e_i = -1*np.eye(2)    
    L_e = np.vstack((L_e_i,L_e_i,L_e_i,L_e_i))    
    L_e_inv = np.linalg.pinv(L_e)    
    v_c = -lmbda*L_e_inv @ error
    v_c = np.concatenate((v_c,[0,1])).reshape((-1))
    
    try:
      now = rclpy.time.Time()
      trans = self.tf_buffer.lookup_transform("link1","camera_link",now)
    except TransformException as ex:
      self.get_logger().info(
      f'Could not transform {"link1"} to {"camera_link"}: {ex}')
      return
    Rotation_matrix = quaternion_matrix([ trans.transform.rotation.x,
                                          trans.transform.rotation.y,
                                          trans.transform.rotation.z,
                                          trans.transform.rotation.w])
    v_w = Rotation_matrix @ v_c # 4 x 1 # link1 frame velocities
    trans_link1_camera = [trans.transform.translation.x,
                          trans.transform.translation.y,
                          trans.transform.translation.z]

    try:
      now = rclpy.time.Time()
      trans = self.tf_buffer.lookup_transform("link2","camera_link",now)
    except TransformException as ex:
      self.get_logger().info(
      f'Could not transform {"link2"} to {"camera_link"}: {ex}')
      return
    trans_link2_camera = [trans.transform.translation.x,
                          trans.transform.translation.y,
                          trans.transform.translation.z]

    J11 = np.cross(np.array([0,0,1]),trans_link1_camera)
    J12 = np.cross(np.array([0,0,1]),trans_link2_camera)
    J21 = np.array([0,0,1])
    J22 = J21
    J = np.hstack((np.vstack((J11,J12)),np.vstack((J21,J22)))) # 6 X 2 
    J_inv = np.linalg.pinv(J) # 2 X 6 
    j_v = J_inv[0:2,0:2] @ v_w[0:2] # 2 X 1

    # publish
    val = Float64MultiArray()
    val.data = np.array([j_v[0],j_v[1]]).astype(np.float64).tolist()
    # self.velocity_publisher_.publish(val)
    self.plot(self.checkConvergence(error),val)
    
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)  
  # Create the node
  image_subscriber = ImageSubscriber()  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
