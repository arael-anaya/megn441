# MEGN 441 Lecture 04

## Writing your own ROS package

Note: This demo makes use of the turtlesim package from the official [ros docs](https://docs.ros.org/en/humble/Tutorials.html). I definitely recommend checking those out.

### Generating package files

First, we need to create our workspace folder, which I will call `demo_ws`. Wherever you want it to be, run:

```bash
mkdir demo_ws
```

Within that workspace folder, we need a `src` folder in the demo_ws.

```bash
cd demo_ws
mkdir src
```

Then, we want to create our package, which I'll call `pubsub`. There are a number of useful flags when creating a package, the most important is `--build-type` to specify development language. I'll also use `--node-name` to initialize a blank node file - `demo_pub` Note that you need to have access to your ROS workspace for this, so you'll need to be inside your Docker container or alternative. From within `src`,

```bash
ros2 pkg create demo_pubsub --build-type ament_python --node-name demo_pub
```

Then, we write the Python class for our node - see [demo_pub.py](../src/demo_pubsub/demo_pubsub/demo_pub.py).

After writing our node, we need to build our package with `colcon`. Should be in the workspace with just `src`, so may need to `cd ..`:

```bash
colcon build --symlink-install
```

This will create three more folders: `build`, `install`, and `log`. We need to source the local build:

```bash
source install/setup.bash
```

To run our node, we use `ros2 run`, which runs a single node. First, in one terminal:

```bash
ros2 run turtlesim turtlesim_node
```

Then, in another:

```bash
ros2 run demo_pubsub demo_pub
```

We also showed how to inspect topics in the class. First, we listed which topics are available.

```bash
ros2 topic list
```

Then, we inspected the `/turtle1/cmd_vel` topic,

```bash
ros2 topic info /turtle1/cmd_vel
```

This showed us that `turtle1/cmd_vel` uses message type `geometry_msgs/msg/Twist`. To learn more about that message type, we used:

```bash
ros2 interface show geometry_msgs/msg/Twist
```
