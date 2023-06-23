# Computer Vision Techniques
This subfolder deals with various computer vision techniques to detect several image features on an object. These image features are not only important to understand the object geometry to be used for planning and executing robotic manipulation strategies, but also would allow us to implement vision-based control algorithms. 


The following are performed:
- Spawn the robot and the object of interest in gazebo. 
- Get the camera images from the simulator and publish them back to ROS2
- Perform the following on the captured image using OpenCV functions
    - Apply color thresholding
    - Apply canny edge detection algorithm
    - Apply Harris corner detection algorithm
    - Apply Hough circles algorithm to detect the circles and find their centers.
## Installation

Install python

```bash
 sudo apt-get install python3-opencv
```
## Output
The compiled outputs of the project can be found in [Results](
https://github.com/Gokulsrinivas98/Vision-Based-Manipulation/edit/main/HW3/GSrinivasan_Hw3.pdf)    
