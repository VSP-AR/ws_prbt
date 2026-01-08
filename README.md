# Launching the simulation on PRBT robot with RViz and Gazebo

## Step 1: Clone the Repository 
Clone the repository inside the workspace in your local:

```bash
git clone https://github.com/VSP-AR/ws_prbt.git
```
## Step 2: Build and Source repository
 Once cloned, build and source the repository
```bash
colcon build
source install/setup.bash
```
## Step 3: Launching the simulation

Then launch the below launch files:

```bash
ros2 launch prbt_cell_moveit_config move_group.launch.py use_sim_time:=true
```

```bash
ros2 launch prbt_cell_bringup sim_robot.launch.py use_sim_time:=true
```

This launches Rviz, Gazebo and the robot moves in a continuous loop until interrupted.
