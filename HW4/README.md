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

