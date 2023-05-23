title: Collection and Processing of Sensor Data
author: Tamás Nagy

# Gathering and processing sensory data

---

## Practice

---

### 1: Leo Rover

---

1. Install the Leo rover ROS packages:

    ```bash
    sudo apt update
    sudo apt install ros-noetic-leo*
    ```

    ---

2. Start the Gazebo simulator with the Mars landscape using the instructions from http://wiki.ros.org/leo_gazebo.

    ---

3. Start the teleop node and move the robot.

---

### 2: Coffee on Mars - Capturing Images

---

!!! warning
    The Mars rover sent an image of an unusual object that looks like a coffee mug! The task is to turn the rover towards the mug and approach it for detailed examination.

![](img/coffee.png){:style="width:600px"}


---

1. Start Gazebo:

    ```bash
        gazebo
    ```

    ---

2. In the `insert` panel, search for the `googleresearch/models/cole_hardware_mug_classic_blue` model and place it
in the simulation. This is necessary to have the mug model in our file system later.

    ---

3. Close Gazebo.

    ---

4. Download the `leo_masryard_coffee.launch` and `marsyard_coffe.world` files,
then copy them to the `catkin_ws/src/ros_course/launch` and `catkin_ws/src/ros_course/worlds`
directories respectively.

    ---

5. Modify the file paths `/home/tamas/.ignition/fuel/fuel...` in the `.world` files to match your own.

    ---

6. Launch the simulator:

    ```bash
        roslaunch ros_course leo_marsyard_coffee.launch
    ```

    ---

7. Start the teleop and `rqt_image_view`:

    ```bash
    rosrun leo_teleop key_teleop
    ```

    ```bash
        rosrun rqt_image_view rqt_image_view
    ```

    ---

8. Capture images showing the coffee mug being visible and not visible.

---

### 3: Coffee on Mars - Offline Image Processing

---

1. Write a Python script to read and display the captured images.

    ---

2. Perform color-based segmentation (or any other method) to segment the coffee mug.

    ---

3. Determine the center of the mug in image coordinates.

    ---

4. Filter out the noise caused by segmentation.

---

### 4: Coffee on Mars - Online Perception Node

---

1. Subscribe to the `/camera/image_raw` topic and display the received images using the `cv.imshow()` function.

    ---

2. Integrate our working computer vision algorithm into a ROS node.

    ---

3. Publish the detected mug's center coordinates in a new topic. You can use types like `Int32MultiArray`, `Point2D`, or define your own (the mug size will be needed later).

    ---

4. *Bonus: Publish the mask and masked image in separate Image topics.*

---

### 5: Coffee on Mars - Operation Logic Node

---

1. Write a new ROS node that receives messages from the perception node and is capable
of controlling the rover's movement.

    ---

2. Rotate the rover in place until the mug is in the center of the image.

    ---

3. Approach the mug until its apparent size does not exceed 50% of the image size.

    ---

4. Capture an image of the suspicious object.

---

### 5+1: Bonus

---

1. Explore the insertable models in Gazebo's `insert` panel and choose one that can be detected on the camera image using a different method (e.g., template matching).

    ---

2. Modify the nodes to approach this object with the rover.

---


## Useful links

- [http://wiki.ros.org/leo_gazebo]()
- [http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython]()
- [https://realpython.com/python-opencv-color-spaces/]()
- [https://stackoverflow.com/questions/59164192/how-to-find-the-contour-of-a-blob-using-opencv-python]()
- [Turtlebot](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html)


