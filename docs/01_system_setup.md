---
title: Bevezetés
author: Nagy Tamás
---

# 01. Bevezetés

---

## Robot Operating System (ROS) bevezetés

---

### A robot fogalma

![](img/what_is_a_robot_1.png){:style="width:300px" align=right}

- **Joseph Engelberger, pioneer in industrial robotics:** *"I can't define a robot, but I know one when I see one."*
- **Wikipedia:** *"A robot is a machine—especially one programmable by a computer— capable of carrying out a complex series of actions automatically. Robots can be guided by an external control device or the control may be embedded within. Robots may be constructed on the lines of human form, but most robots are machines designed to perform a task with no regard to their aesthetics."*
- **ISO 8373:2012 Robots and robotic devices – Vocabulary, FDIS 2012:** *"A robot is an actuated mechanism programmable in two or more axes with a degree of autonomy, moving within its environment, to perform intended tasks."*
- **Rodney Brooks, Founder and CTO, Rethink Robotics:** *"A robot is some sort of device, wich has sensors those sensors the world, does some sort of computation, decides on an action, and then does that action based on the sensory input, which makes some change out in the world, outside its body. Comment: the part "make some change outside its body" discriminates a washing machine from e.g. a Roomba."*
- **Tamás Haidegger, Encyclopedia of Robotics**: *"A robot is a complex mechatronic system enabled with electronics, sensors, actuators and software, executing tasks with a certain degree of autonomy. It may be pre-programmed, teleoperated or carrying out computations to make decisions."*

---

### Mi a ROS?

![](https://moveit.ros.org/assets/images/logo/ROS_logo.png){:style="width:300px" align=right}

- Open-source, robotikai célú middleware
- Modularitás, újra-felhasználhatóság (driverek, algoritmusok, library-k, ...)
- Hardware absztrakció, ROS API
- C++ és Python támogatás
- Ubuntu Linux (kivéve ROS 2)
- Népes közösség

<iframe title="vimeo-player" src="https://player.vimeo.com/video/639236696?h=740f412ce5" width="640" height="360" frameborder="0"    allowfullscreen></iframe>


---


### Történet

![](https://www.freshconsulting.com/wp-content/uploads/2022/06/path-planning-1024x693.jpg){:style="width:300px" align=right}

- 2000-es évek közepe, Stanford: robotikai célú rugalmas, dinamikus szoftverrendszer prototípusok fejlesztése
- 2007, Willow Garage: inkubáció, kialakult a ROS alapja BSD open-source licensz alatt
- Robotikai kutatások területén egyre inkább elterjedt, PR2
- 2012: Ipari robotika, ROS-Industrial
- 2017: ROS 2



---

## Fejlesztőkörnyezet felállítása - Házi feladat

---

![](https://brandslogos.com/wp-content/uploads/thumbs/ubuntu-logo-vector.svg){:style="width:400px" align=right}

Recommended environment:

- Ubuntu 20.04
- ROS1 Noetic
- ROS2 Foxy
- *IDE: QtCreator/CLion/VSCode*



---

## ROS 1 Noetic


![](http://wiki.ros.org/noetic?action=AttachFile&do=get&target=noetic.png){:style="width:300px" align=right}

1. Install ROS Noetic

    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-desktop-full
    source /opt/ros/noetic/setup.bash
    ```

    ---

2. ROS 1 dependencies

    ```bash
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo rosdep init
    rosdep update
    ```

    ---

3. Once we are done with this, we can test our ROS 1 installation with the following command:

    ```bash
    source /opt/ros/noetic/setup.bash
    roscore
    ```

---    

### ROS 2 Foxy

![](https://global.discourse-cdn.com/business7/uploads/ros/optimized/2X/d/d6fd5322bd2ddc06530d8352fcab20f0bca08c06_2_861x1024.png){:style="width:300px" align=right}


1. Setup locale.

    ```bash
    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings
    ```

    ---

2. Install ROS 2 Foxy


    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-foxy-desktop python3-argcomplete ros-dev-tools ros-foxy-moveit* ros-foxy-control*
    ```

    ---

3. Once we are done with this, we can test our ROS 2 installation with the following command:

    ```bash
    source /opt/ros/foxy/setup.bash
    ros2 run demo_nodes_py talker
    ```

    ---

4. The `source` command is responsible for setting the environment variables, which must be entered every time a new terminal window is opened. This command can be pasted at the end of the `~/.bashrc` file, which will run every time a terminal window is opened, so we don't have to type it all the time (ROS 2 will be the default):

    ```bash
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    ```
    
---

### Further packages


1. The following packages will also be needed during the semester, so it is worth installing them as well:

    ```bash
    sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev python3-catkin-tools python3-osrf-pycommon libasound2-dev libgl1-mesa-dev xorg-dev ros-foxy-turtlebot3*
    ```

---


### IDE


1. QtCreator

QtCreator is currently one of the most usable IDEs for developing ROS packages, for which a ROS plugin has also been created. The installer is available at the link below. You should use the "18.04 **offline** installer", it also works on Ubunutu 20.04.

[https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html](https://ros-qtc-plugin.readthedocs.io/en/latest/ _source/How-to-Install-Users.html)

Once downloaded, the IDE can be installed with the following command (it is important to insert `cd` in the download location):

    ```bash
    chmod +x qtcreator-ros-bionic-latest-offline-installer.run
    sudo ./qtcreator-ros-bionic-latest-offline-installer.run
    ```

    When the installer asks where to install it, change to e.g `/home/<USER>/QtCreator` mappára. Ha a root-ba teléepítjük, nem fogjuk tudni futtatni. A telepítés után "Qt Creator (4.9.2)" néven keressük.
   
    ---

2. CLion

CLion has a high degree of ROS integration, the use of which is the most recommended during the course. A free student license can be requested at the following link: [https://www.jetbrains.com/community/education/#students](https://www.jetbrains.com/community/education/#students)

After installation, find the `/var/lib/snapd/desktop/applications/clion_clion.desktop` file. We rewrite the appropriate line for this, so the environment set by the terminal will be used by the IDE:
    
    ```bash
    Exec=bash -i -c "/snap/bin/clion" %f
    ```

    ---

3. Visual Studio

    Microsoft Visual Studio also supports source codes for ROS, this IDE can also be used during the semester.
    
---

!!! tip "Suggestion"
Install **Terminator** terminal emulator:
```bash
sudo apt update
sudo apt install terminator
```


---

## Hasznos linkek

- [https://www.ros.org/](https://www.ros.org/)
- [ROS 1 Noetic installation](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [ROS 2 Foxy installation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [ROS Distributions](http://wiki.ros.org/Distributions)
- [http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [CLion hallgatói licensz](https://www.jetbrains.com/community/education/#students)
- [QtCreator + ROS plugin](https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html)
- [IROB virtual tour](https://www.youtube.com/watch?v=8XmKGWBV5Nw)




















