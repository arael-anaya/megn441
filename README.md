# MEGN 441: Intro to Robotics

This is the home repository for the labs of MEGN 441: Intro to Robotics at Colorado School of Mines.

## Quick start

See ROS2 Install for further guidance. With Docker installed, navigate to directory with `Dockerfile` and run

```` bash
docker build -t ros2-dev .
````

Once you have built the docker image using the Dockerfile, modify `startDocker.sh` to capture the path to your local me441 directory with `ros2_ws`. You may need to make the shell script executable with:

```` bash
chmod 777 startDocker.sh
````

Once `startDocker.sh` is executable, you can run it with:

```` bash
./startDocker.sh
````

## Lab Sequence Description

| Lab | Weeks | Content |
| --- | --- | --- |
| 1 | 2 | Introduction to Rosbots, ROS2 Packages, and Remote Control |
| 2 | 2 | Simultaneous Localization and Mapping (SLAM) |
| 3 | 3 | Robot Arm Pick and Place |
| 4 | 3 | Delivery and Navigation |
| 5 | 3 | Final Project Showcase |

### Assorted tips and tricks

**Nomachine resolution fix**:
- Using nomachine, go to the !M dropdown menu, then select `Open Menu Panel`
- Click the `Display` icon
- Select `Resize remote display`