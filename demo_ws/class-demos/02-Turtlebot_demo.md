# MEGN 441 Lecture 02

## Turtlebot demo

Note: This demo makes use of the turtlesim package from the official [ros docs](https://docs.ros.org/en/humble/Tutorials.html). I definitely recommend checking those out.

## Running the turtlesim demo

First, we open two terminals. If your system doesn't do it by default, `source` your ROS installation in each one, probably:

```bash
source /opt/ros/humble/setup.bash
```

Then, use  `ros2 run` to run one node in each: `turtlesim_node` and `turtle_teleop_key`. Both are in the `turtlesim` package.

```bash
# In terminal 1
ros2 run turtlesim turtlesim_node
```

```bash
# In terminal 2
ros2 run turtlesim turtle_teleop_key
```

From terminal 1's command, a little turtle should open in its own window. With terminal 2 selected, you can use the arrow keys to make the turtle move around!

If you open a third terminal, you can inspect the ROS system that is running under the hood. First, view all current topics:

```bash
# In terminal 3
ros2 topic list
```

Then, view the output of some of those topics:

```bash
# In terminal 3
ros2 topic echo /turtle1/pose
```

If you reselect Terminal 2 and drive the turtle around, you can see the pose changing. When you're done, press `ctrl-C` to stop viewing the pose. It's also viewing the `cmd_vel` while driving around:

```bash
# In terminal 3
ros2 topic echo /turtle1/cmd_vel
```

Finally, in-class we took a brief look at what services were available:

```bash
# In terminal 3
ros2 service list
```

And what actions are available:

```bash
# In terminal 3
ros2 action list
```
