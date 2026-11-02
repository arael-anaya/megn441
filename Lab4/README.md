# Lab 4: Delivery and Navigation

## Lab Overview

In Lab 4, we will get the robot to navigate using a map of the space from one position to a final position and delivers a payload to its target location. You'll begin inside a square with a set initial position and deliver to a box with a target position. There are two deliveries to make. The first goes form below the projector at the back of the classroom to the area in front of the lockers. The second goes from the area to the right of the entrance to the small hallway next to the classroom. Demonstrating one course or the other will earn you full credit. For extra credit, connect the block detection for lab 3 to make both deliveries in a single run.

By the end of this lab, you'll be able to:

- Use nav2 for robot navigation
- Modify package parameters to improve performance
- Write an action server
- Write an action client
- Use a finite state machine to manage robot behaviors

### Lab Procedure

#### Week 1

0. **Prepare the bot**: Create a clean workspace on the robot for your team for this project. Something like: `a9_ws_lab4` for team A9. You won't need any of the base ros2_ws packages - those are all in ~/ros2_ws on the bot, and they get sourced automatically on the robot's startup. `bringup.launch.py` also gets launched automatically, and the Orbbec and sllidar packages get sourced from ros2_ws automatically. In your new package, include any previous code from prior labs that you may need. You will certainly need `rosbot_arm` and `rosbot_msgs`, and you may need others.

   **Prepare your team**: Talk through how collaboration will work on this lab. You've worked on two labs to this point. What worked well for collaboration in the past? What didn't work well? How will you share files? How will you divide tasks? If you need to come in outside of lab hours, what days/times work well?

1. **Use nav2 as a black box**: Using the `Robo_Lab` map that is included in this folder, test your robot in nav2! I have already cloned the correct version of `nav2_bringup` from [the navigation2 github](https://github.com/ros-navigation/navigation2/tree/main/nav2_bringup) for you into `A7_lab4_ws/src/rosbot_nav` and added the map of the basement to maps. I haven't changed anything else here. Colcon build this package on the bot.

   You'll need three browser tabs to launch everything. First, launch your sensors_launch.py from Lab2. Then, launch nav2 using:

   ```bash
   ros2 launch rosbot_nav bringup_launch.py \
   map:=/path/to/Robo_Lab.yaml \
   params_file:=/path/to/nav2_params.yaml \
   use_sim_time:=false
   ```

   Wait until the output settles and reports that the managed nodes are active. In another tab, open RViz with the Nav2 view:

   ```bash
   ros2 launch nav2_bringup rviz_launch.py
   ```

   The view shows the map, the lidar scan, both costmaps, and the planned path.

2. **Experiment with nav2 params**: Spend some time experimenting with nav2, and edit the `nav2_params.yaml` file to explore some of the possible parameters. Two key replacements you should make are to replace your Planner Server and Controller Server with ones that are compatible with an Ackermann Drive car. [Here is a high-level guide on parameter tuning, including tables with planners/controllers and their compatibility](https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/). To use a different planner, go to the configuration guide for that planner and grab the params example from the site. After updating your planner/controller, spend some time playing with specific parameters. Some key ones to pay attention to are `inflation_radius`, `motion_model_for_search`, `minimum_turning_radius`, and various cost metrics. Record your changes to parameters in a table of changed values and a description of their effect. As it drives around, step in front of the bot. What does it do?

3. **Begin your delivery manager node**: By the end of week 3, this delivery manager node will handle pick and place operations along with autonomously calling the navigation servers. For now, we will get it started by creating an Action Client for the `navigate_to_pose` nav2 action ([API](https://api.nav2.org/actions/humble/navigatetopose.html)).

   A nice way to handle a manager node is with a Finite State Machine and a ROS2 timer. This approach will prevent your code from blocking other ROS interfaces. I have provided a starting template in the provided `A7_lab4_ws/src/nav_manager` package.

#### Week 2

4. **Write an action servers for arm movement**: The final type of ROS2 interface is an action. I'd like you to modify your inverse kinematics node to create two actions: Pick and Place. The Pick action should receive a goal location for the block, pick it up, and bring it back to a Stowed position, keeping the arm in a nicely tucked orientation for driving. The Place action should move the block from the Stowed position, release it in its target location, and then return to a Stowed position. **Always stow the arm before driving**. Depending on how you want the camera/block position, some possible stow positions are `xyz=[0.1,0,0.24], pitch=-45` or `xyz=[0.1,0,0.2], pitch=-60`.

5. **Write an action clients for your arm movement**: Add action clients for both Pick and Place to your `nav_manager` node, so that your bot can pick and place on command. Update your finite state machine to handle the logical sequence of behaviors.
<!-- 5. **Write custom behavior tree**: The behavior tree is written using an XML format and is fairly readable on its own, [documentation here](https://www.behaviortree.dev/docs/3.8/learn-the-basics/xml_format). The behavior tree folks also have a lovely free IDE for writing behavior trees in a visual format called [Groot](https://www.behaviortree.dev/groot/). If using Groot, you can access the nav2 custom nodes here: [nav2 Groot tutorial](https://docs.nav2.org/rolling/tutorials/general_tutorials/groot_tutorials/groot/groot/#groot-interacting-with-behavior-trees) -->

#### Week 3

6. **Deliver the block to the target**: Finalize your manager node and fine tune your code so that the robot begins at its starting location, picks up the block, drives to the target, and releases the block in its target location.

## Lab Grading

The grading of each lab is based 50\% on the successful completion of the lab. For this lab, that 50\% breaks down into:
resample_interval
- 10\% - Successfully demonstrate navigation using nav2
- 10\% - Successfully write an action client to the NavigateToPose action
- 10\% - Successfully write Pick and Place action servers
- 20\% - Deliver a block from the given starting location to a set final position.
- **10\% Extra Credit** - Make both deliveries, finding and picking up the 2nd block after delivering the first.

### Lab Report Guidelines

The guidelines below will be used in grading your lab report. Be sure to include everything that the guidelines below mention for full credit!

#### 1. Problem Statement

- Write a succinct 1-3 sentence description of the goals of this lab.

#### 2. Methods

- Describe your team's contributions to the software of the robot for Lab 4.
- Describe the difference between topics and actions in ROS2.
- Explain how your finite state machine logic works.

#### 3. Results

- Explain which parameters you changed in nav2 and their impact. Include a table of changed values and a description of their effect.
- Describe what the robot did when someone stepped into its path. Which part of Nav2 do you think made that decision?
- Include one or more videos of your robot demonstrating the lab's goals. 

#### 4. Conclusions

- Discuss what your team's biggest lessons learned are from this lab.
- What did your robot do well?
- What could you do to improve your robot's behavior, given more time?
- Discuss the challenges, if any, that your team encountered using nav2.

#### AI Appendix

- Include a 1 paragraph reflection on the questions below, whether or not you used AI.
  - Did you use any AI to support support the completion of this lab? Why or why not?
  - If no, how could it have helped? What did you gain by avoiding AI use?
  - If yes, how was your experience of using it? How did it help? What did you miss out on by using AI?
  - If you used AI, what resources do you think it used in generating its answers?
  - If you used AI, please copy and paste your interactions below:
  
#### Others

- Be sure to upload your code as a Zip file or a link to a github page (please don't zip build, install, log).
- If including a link to a video, make sure that it works from a private browser (so the TAs don't have to log in or have access to it)
