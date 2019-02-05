# Mobile base and robotic manipulator telecontrol

Robotic arm ([CPR Mover 4](https://cpr-robots.com/education#Mover4)) has been connected to [differential wheeled mobile robot](https://en.wikipedia.org/wiki/Differential_wheeled_robot) and the possibility of controlling both sub-systems has been implemented through wireless joystick. The same  functionality has been implemented through  WEB GUI with live stream of movement of tele-controlled robotic arm - mobile base system. Inverse kinematics were the core idea of the robot movemenet, basically inverse kinematics makes use of the kinematics equations to determine the joint parameters that provide a desired position for robotic arm's end-effector (gripper).

(picture goes here)

## Prerequisites

### Hardware

* mobile base (paletar) - with central processing computer running Linux OS and with connected router for creating own local area network
* manipulator (robotic arm) - connected to 12V VCC and to mobile base's computer via USB-CAN communication protocol

### Software

* any Linux distro will probably work fine on central computer of the mobile base
* [ROS](http://www.ros.org/), in our case [ROS kinetic](http://wiki.ros.org/kinetic) - flexible framework for robotic software development
* [Peak-System CAN interface](http://www.peak-system.com/fileadmin/media/linux/index.htm) - provides communication  of the robotic arm with ROS via PCAN-USB communication protocol, installation process and test case of CPR Mover4 robotic arm check out official GitHub profile [CPR-Robots](https://github.com/CPR-Robots/Mover4)
* Rosbridge_suite

## Dev

TBA

## Installation and getting started

TBA

## Controls

### web interface

TBA

### joystick

TBA

## Authors

* Filip Bašić
* Joško Jukić
* Ante Lojić Kapetanović
* Ana Šćulac

for electronics engineering graduate course [Programming of mobile robots and drones [222]](https://nastava.fesb.unist.hr/nastava/predmeti/9687).
