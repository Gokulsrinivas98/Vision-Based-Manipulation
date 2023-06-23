# Vision-Based-Manipulation
This repository contains notes, reference documents, and tutorials for the following concepts. 
- [Basics of manipulation](https://github.com/Gokulsrinivas98/Vision-Based-Manipulation/tree/main/HW1)
- [Grasping Basics and Grasp stability](https://github.com/Gokulsrinivas98/Vision-Based-Manipulation/tree/main/HW2)
- [Computer Vision](https://github.com/Gokulsrinivas98/Vision-Based-Manipulation/tree/main/HW3)

<details>
<summary>:eyes: Show Details</summary>

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
</details>

- [Visual Servoing](https://github.com/Gokulsrinivas98/Vision-Based-Manipulation/tree/main/HW4)
  <details>
<summary>:eyes: Show Details</summary>
# Visual Servoing 
This subfolder deals with implementing visual servoing algorithm for a 2 DOF robot


The following are performed:
- Spawn the created object(rectangular block with 4 cylinders placed on top of it) on the ground within robot's workspace.

- Move the robot via the position controller so that the whole object is visible in the image. Take an image, get the coordinates of the 4 circle centers.

- Move the robot to a different location using the position controller. In the new location, the whole object should still be visible by the virtual camera. Take an image, get the coordinates of the 4 circle centers.

- Implement visual servoing algorithm that uses these four point features (the centers of the circles) and servos the robot from one image configuration to the other.
## Output
The compiled outputs of the project can be found in [Results](
https://github.com/Gokulsrinivas98/Vision-Based-Manipulation/blob/main/HW4/VisionBased%20Hw4.pdf)


</details>
