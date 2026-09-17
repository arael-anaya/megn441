# Lab 2: Simultaneous Localization and Mapping (SLAM)

## Lab Overview

By the end of this lab, you will be able to:

- Install off-the-shelf ROS2 packages
- Build off-the-shelf ROS2 packages
- Visualize sensors using RViz
- Map a room using SLAM

During week 1 of this lab, we will be getting our LiDAR and RGBD cameras working and then visualizing the output using RViz. And then, during week 2, we will use slam_toolbox to map the basement hallway of Brown!

### Lab Procedure

0. Create a clean workspace on the robot for your team for this project. Something like: a7_ws for team A7. You won't need any of the base ros2_ws packages - those are all in ~/ros2_ws on the bot, and they get sourced automatically on the robot's startup. `bringup.launch.py` also gets launched automatically. This way, you can just rebuild your locally edited files in lab2. You may still want to bring a `rosbot` package from lab1, this is where I would put new launch files for this lab. You can bring in one team member's teleop_joy from lab1 if you like.
1. Download and install the sllidar_ros2 package from [Slamtec sllidar](https://github.com/Slamtec/sllidar_ros2). Copy this package over to the robot in your workspace, and build it with `colcon build --packages-select sllidar_ros2`. In order to run the node, you will need to locate the address of your Lidar on the robot. It will be in the `/dev/` folder and should have `USB` in the name. To see potential candidates, you can use `ls /dev | grep USB`. You can also list usb devices on the bot to see what's available with `lsusb` to see all devices. There are two other parameters that we'll need to change from the default node: our `frame_id` should be `lidar_frame` and our `serial_baudrate` should be 115200. Use the ROS2 Humble documentation to figure out how to apply those three parameters in your ros2 run command - they will be flags on your command.
2. While one terminal runs the sllidar node, open another terminal and run

    ```bash
    ros2 run rviz2 rviz2
    ```

    This is RViz, one of the most useful ROS2 tools. With it, we can visualize our robot with its current joint configuration along with all of the sensor readings that our robot is taking in. In order to view those things, we will need to add those displays to our RViz. On the left-hand side of RViz is a panel titled Displays. At the bottom of the panel is a button that says Add. Selecting this option brings up a pop-up with various types of display plugins for RViz. We want to select two now: a `LaserScan` and a `RobotModel`. After adding those, you'll need to click on the dropdown for each within the Displays panel to select the topic that each is subscribed to. We'll also need to update the Fixed Frame in the global options at the top. I recommend choosing `base_footprint` as our fixed frame, as it is the reference frame centered under the robot on the floor. Then, using the File menu, you can save this view as `sensors.rviz` in your `rosbot/config` folder.
3. Download and install the OrbbecROS2_SDK packages from [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2/tree/main). After you download, you'll need an older version of the repository, so type `git checkout main`. (NOTE: When you copy this package over to the robot, it has symbolic links in it (this broke mine). If using rsync to copy files, use the `-l` flag, as in `rsync -rluv` to copy files to the robot.) We'll need to change one argument in this launch file - we need `camera_name` to be `depth_cam`. Then we'll use the `dabai_dcw.launch.py` launch file from the `orbbec_camera` package. Once you get this running, add a `DepthCloud` display and a `Camera` display to RViz, finding the relevant topic(s) for each.
4. Write a launch file to run your lidar, your depth camera, and rviz, pulling all relevant parameters, arguments, and RViz configuration file and include it in your `rosbot` package. I have given you a new template, which pulls in a lidar_filter node as well, in `Lab2/sensors_launch.py`
5. Use slam_toolbox to map the basement of Brown! I'll leave you to search through slam_toolbox's documentation and figure out how to get it running. Ask your TA if you get stuck.
6. Be sure to save your map! To do so, look through available services with `ros2 service list`, and then look into how to call a service from the command line in the ROS2 Humble tutorials.

## Lab Grading

The grading of each lab is based 50\% on the successful completion of the lab. For this lab, that 50\% breaks down into:

- 10\% - Get your LiDAR running
- 10\% - Get your RGBD camera running
- 10\% - Visualize your LiDAR and RGBD camera in RViz
- 20\% - Map the basement hallways of Brown

### Lab Report Guidelines

The guidelines below will be used in grading your lab report. Be sure to include everything that the guidelines below mention for full credit!

#### 1. Problem Statement

- Write a succinct 1-3 sentence description of the goals of this lab.

#### 2. Methods

- Briefly research and describe how your depth camera and lidar work. You may have a hard time finding documentation on your specific depth camera model, so just research a similar Orbbec depth camera.
- Describe your team's contributions to the software of the robot for Lab 2.
- Describe how slam_toolbox works and which files you had to use to run it. Which topics does it use as inputs? How did you save your map?

#### 3. Results

- Include a screenshot and description of RViz showing your LiDAR and RGBD camera running with the robot visible in frame. Be sure each of these elements are visible in the figure.
- Include a screenshot of your map generated using SLAM toolbox and describe how effectively your robot was able to map the space. 
- Explain the key communication channels used in Lab2: topics, services, etc.

#### 4. Conclusions

- Discuss what your team's biggest lessons learned are from this lab.
- Discuss the challenges, if any, of installing/using an off-the-shelf ROS package.
- Discuss your experience exploring slam_toolbox

#### AI Appendix

- Include a 1 paragraph reflection on the questions below, whether or not you used AI.
  - Did you use any AI to support support the completion of this lab? Why or why not?
  - If no, how could it have helped? What did you gain by avoiding AI use?
  - If yes, how was your experience of using it? How did it help? What did you miss out on by using AI?
  - If you used AI, what resources do you think it used in generating its answers?
  - If you used AI, please copy and paste your interactions below:
