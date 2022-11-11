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

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'output_image', 10)

      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
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
  def findAndDisplayCenter(self,img, disp):
    # Finding contour centers : 
    cnts, heirarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in cnts:
        if cv2.contourArea(c) <= 25:
            continue
        cx = int(average([x for x in c[:,0,0]]))
        cy = int(average([y for y in c[:,0,1]]))
        center = (cx, cy)
        print("Center coordinate: "+str(center))
        radius = 7
        cv2.circle(disp, (cx,cy), radius, (0, 255, 255), -1)
        cv2.putText(disp, "center", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        # self.publisher_.publish(self.br.cv2_to_imgmsg(disp, encoding="bgr8")) #Uncomment this line and comment out the following 3 lines to view the image via image viewer
    cv2.imshow("Center labelled image", disp)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


  def colorThreshold(self,img):
    def masks(mask, img):
      imask = mask>0
      color = np.zeros_like(img, np.uint8)
      color[imask] = img[imask]
      self.findAndDisplayCenter(mask, color)
     
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (36, 25, 25), (70, 255,255)) | cv2.inRange(hsv, (95, 25, 25), (135, 255,255)) | cv2.inRange(hsv, (0, 100, 25), (15, 255,255)) | cv2.inRange(hsv, (140,100,20) , (170,255,255) )
    masks(mask,img)
    # # Color thresholding!
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Green
    mask = cv2.inRange(hsv, (36, 25, 25), (70, 255,255))
    masks(mask,img)
    # Blue
    mask = cv2.inRange(hsv, (95, 25, 25), (135, 255,255))
    masks(mask,img)
    # Red
    mask = cv2.inRange(hsv, (0, 100, 25), (15, 255,255)) 
    masks(mask,img)
    #meg
    mask = cv2.inRange(hsv, (140,100,20) , (170,255,255) )
    mask(mask,img)
   
    # Publish the image.
    # The 'cv2_to_imgmsg' method converts an OpenCV
    # image to a ROS 2 image message
      # self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, encoding="bgr8"))
      #self.publisher_.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))

    
  
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
