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
## Connection Information                                                                                               
There are 3 options to connect to the Rosbots. In all approaches, the Rosbot will have the username `ubuntu` and password `ubuntu`

1. **Remote Desktop**: Connect your WiFi to the Rosbot's access point, using the password `hiwonder`. The specific Access Point SSID for your bot is located on the OLED screen at the front of the robot. Then, download and install the software Nomachine ([Download Links](https://download.nomachine.com/personal-edition/)). From Nomachine, connect to your bot using the username and password above. **NOTE**: Only one device can be connected through Nomachine at a time.
2. **SSH over WiFi**: Once you have connected to the Rosbot's access point, as described in the Remote Desktop description, open a terminal and enter the command `ssh -Y ubuntu@192.168.149.1`. You'll be prompted for the password (`ubuntu`).
3. **SSH over wire**: Depending on your computer's firewall settings, you may be able to connect directly to the bot with a wire. We have USB-A to USB-C cables available in the lab. To test your connection, try `ping ubuntu@192.168.55.1`, or skip directly to using `ssh -Y ubuntu@192.168.55.1`. You'll be prompted for the password (`ubuntu`).

### Assorted tips and tricks

**Nomachine resolution fix**:
- Using nomachine, go to the !M dropdown menu, then select `Open Menu Panel`
- Click the `Display` icon
- Select `Resize remote display`

