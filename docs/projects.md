---
title: Követelmények
author: Tamas D. Nagy
---

# Projects

---


## Challenge levels and grades

---

Projects can be completed at three *Challenge levels*. The *Challenge level* determines the  **best** grade that can be received to the project!

| Challenge level  | Best grade |
| -------- | :-------: |
| Basic    |     3 |
| Advanced |     4 |
| Epic     |     5 |

!!! tip
    The projects are defined in a way that it is recommended to tart with the **Basic** level, and then gradually work 
towards **Epic**.

The projects are graded based on the follwoing aspects:

- Proved to be the student's own work
- Running results valid output
- Usage of versioning, usage of GitHub/GitLab/other repository
- Launch files
- Completeness of the solution
- Proper ROS communication
- Proper structure of the program
- Quality of implementation
- Documentation quality

---

## Schedule

---

| Week | Date     | Event                  |
|:----:|----------|------------------------|
|  8.  | April 18 | Project lab I.         |
| 13.  | May 23   | Project lab II.        |
| 14.  | May 30   | Project presentations. |

---

## Grading

---

To pass the course, Tests and the Project must be passed (grade 2). One of the Test can be taken again.

!!! abstract "Grade"
    $Grade = (Test1 + Test2 + 2 \times Project) / 4$
	
---
---


## Project topics

---

### 1. TurtleBot3


[TurtleBot3 ROS tutorial](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html)



---

#### 1.1. TurtleBot obstacle avoidance


![turtlebot_world.png](img%2Fturtlebot_world.png){:style="width:380px" align=right}

- **Basic:** Simulator animation, SLAM testing. Implement ROS node/nodes to read sensor data and move the robot.
- **Advanced:** Implement ROS system to detect obstacle and plan and implement obstacle avoidance trajectory in simulated environment using any sensor.
- **Epic:** Impress me!

---

#### 1.2. TurtleBot path following

![](https://robots.ros.org/assets/img/robots/turtlebot3/turtlebot3.png){:style="width:380px" align=right}



- **Basic:** Simulator animation, SLAM testing. Implement ROS node/nodes to read sensor data and move the robot.
- **Advanced:** Implement ROS system for tracking in a simulated environment using any sensor (e.g. passing a wall at a given distance using LIDAR).
- **Epic:** Impress me!
<!--suppress XmlDeprecatedElement -->
<font size="1"> Image source: https://robots.ros.org/turtlebot3/ </font>


---

#### 1.3. TurtleBot object tracking/visual servoing

- **Basic:** Simulator animation, SLAM testing. Implement ROS node/nodes to read sensor data and move the robot.
- **Advanced:** Implement ROS system to find/recognize object and track/move it in simulated environment using any sensor (e.g. visual servoing).
- **Epic:** Impress me!

---

#### 1.4. TurtleBot action library

- **Basic:** Simulator animation, SLAM testing. Implement ROS node/nodes to read sensor data and move the robot.
- **Advanced:** Implement a ROS action-based library of simple operations and a system to execute them (e.g. push object, move to object, turn around).
- **Epic:** Impress me!



---

### 2. YouBot

<iframe width="560" height="315" align="right" src="https://www.youtube.com/embed/qvBEQsGvC3M" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


[YouBot controller GitHub](https://github.com/ABC-iRobotics/YoubotDriver/tree/ROS)

---


#### 2.1. YouBot ROS integration

- **Basic:** YouBot repo build, getting to know it
- **Advanced:** Moving a simulated robot in an articulated ROS environment
- **Epic:** Testing on real robot and/or impress me!


---

### 3. AMBF

[AMBF GitHub](https://github.com/WPI-AIM/ambf)

!!! tip "Building AMBF"
    Fork AMBF, then clone our fork:
    ```bash
    cd ~/ros2_ws/src
    git clone <MY_AMBF_FORK.git>
    ```
    Don't use make as suggested in the AMBF documentation, use colcon:
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    ```    
    Launch the simulator:
    ```bash    
    cd ~/ros2_ws/src/ambf/bin/lin-x86_64
    ./ambf_simulator -l 4
    ```

---


#### 3.1. AMBF da Vinci ROS integration

![ambf_psm.png](img%2Fambf_psm.png){:style="width:200px" align=right}

- **Basic:** Simulator animation, robot control in joint space and task space (IK already implemented in AMBF) from ROS via CRTK topics
- **Advanced:** Object detection in *Peg transfer puzzle
- **Epic:** Autonomous manipulation in *Peg transfer* and/or impress me!

#### 3.2. AMBF KUKA arm ROS integration

![ambf_kuka.png](img%2Fambf_kuka.png){:style="width:200px" align=right}


- **Basic:** Simulator animation, robot control in joint space from ROS
- **Advanced:** Generate trajectories in joint space
- **Epic:** Implement inverse kinematics and/or impress me!

#### 3.3. AMBF PR2 humanoid ROS integration

![ambf_pr2.png](img%2Fambf_pr2.png){:style="width:200px" align=right}

- **Basic:** Simulator animation, robot control in joint space from ROS
- **Advanced:** Robot control in task space, IK?
- **Epic:** Trajectory planning/Navigation/Manipulation and/or impress me!

---

### X. Own topic

---

By discussion.

---

## Useful links

- [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
- [TurtleBot3 Tutorial](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html)
- [AMBF](https://github.com/WPI-AIM/ambf)
- [My fork of AMBF](https://github.com/TamasDNagy/ambf)
- [CRTK topics](https://github.com/jhu-cisst/cisst/blob/devel/utils/crtk-port/crtk-ros-commands.dict)
- [Navigation stack](http://wiki.ros.org/navigation)
- [Paper on LiDAR SLAM](https://www.hindawi.com/journals/jat/2020/8867937/)
- [Paper on vSLAM](https://ipsjcva.springeropen.com/articles/10.1186/s41074-017-0027-2)
- [Paper on Visual Servoing Mobile Robot](https://www.researchgate.net/publication/252057005_An_image_based_visual_servoing_scheme_for_wheeled_mobile_robots)









