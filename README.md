# Structural Integrity Bot
2IRR10 CBL Autonomous Systems Twinning project, team 22.

## Usage guide
Setup and usage:
- [Docker ROS2 workspace](#docker-ros2-workspace-setup)
- [Nav2](#nav2-setup)
- [Exploration](#exploration-setup)
- [Unity](#unity-setup)

- [Expected usage](#expected-usage)

### Docker ROS2 workspace setup
This [gist by AEAEAEAE4343](https://gist.github.com/AEAEAEAE4343/4429cc35d7bd02cdffe0442711fb1d28) is extremely helpful in setting up Docker/Unity (instead of VM/Unity or Docker/Gazebo). Follow its steps before continuing the setup, most notably:
1. Have Docker Desktop working
2. Build the image `2irr10_custom` from the given Dockerfile (once)
3. Create a container from this build image

### Nav2 setup
(This is for testing it virtually, in the real case this won't be needed. We will be using Cartographer)

Go to the [Robotics-Nav2-SLAM-Example repo](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example) and download the `unity_slam_example` folder (at `ros2_docker/colcon_ws/src/unity_slam_example`).

In the Files tab of your Docker container, find the `src` of your ROS2 workspace directory.
Right-click this folder, click "Import" and select the `unity_slam_example` folder.

You can now run `ros2 launch unity_slam_example unity_slam_example.py` inside your container, after playing the Unity Project. It should map and launch RViz.

### Exploration setup
Similar to `unity_slam_example`, import the `explore` folder from the [m-explore-ros2 repo](https://github.com/robo-friends/m-explore-ros2) into the `src` of your ROS2 workspace.

in a terminal, go to the root of your ros workspace and run colcon build, then source install/local_setup.bash

You can now run `ros2 launch explore_lite explore.launch.py` inside your container, after playing the Unity Project and launching Nav2. The robot should start exploring.

> IMPORTANT:
Can someone research how to use this package physically, before the Tuesday the 3rd tutorial? Useful sources:
> - https://github.com/robo-friends/m-explore-ros2/tree/main?tab=readme-ov-file#Simulation-with-a-TB3-robot
> - https://husarion.com/tutorials/ros2-tutorials/10-exploration

### Unity setup

Clone this repo via GitHub Desktop.
It will ask you to initialize LFS or something, click yes.
Open the project in Unity and open the 'test' scene in the Scenes folder
The scripts that read the map and do calculations are attached to the MainCamera GameObject.

"If something breaks when importing the Unity Project, Quinten probably knows how to fix it lol"

### Expected usage
When running the unity project with something that publishes to map (cartographer or the slam example shown above),
Three maps will appear in the top right, one showing the old map, one showing the map currently being made, and the last one showing the differences between the two.

You can control the turtlebot in 3 ways:
- Manually (TurtleBot3 teleoperation)
- Set the goal manually and navigate autonomously (RViz 2D Goal Pose)
- Explore autonomously

The third option aligns the most with our Proof of Concept.
1. Run the navigation (simulation: Nav2, physical: Cartographer)
2. Run the exploration

Four important notes with using exploration are:
- First estimate the bot's pose in RViz
- Do not have objects too close to the bot initially, as this will cause "All frontiers traversed/tried out, stopping."
- For the same reason as the second note, first use RViz to move the bot a small distance. This gives the exploration package a better start, decreasing the chance that it will halt immediately.
- If the exploration algorithm halts before creating a full map, just run it again. 

Once you have explored the environment autonomously once, press P to save the costmap. Then we assume that something in the environment might have changed, so we explore again.

# Instructions to run
1. Run ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=ROSIP (ros args are unneeded if using docker)
2. ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True slam_params_file:=INSERTPATH/mapper_params_online_async.yaml  (use slam params if needed, probably wont though)
3. Control the robot with ros2 run turtlebot3_teleop teleop_keyboard
4. ros2 run nav2_map_server map_saver_cli -f ~/map  to save the map
5. Close slam toolbox and run ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml
6. ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.30
7. drive around until the costmap is fully filled in, then press p in unity to save it.
8. restart the unity simulation, change something in the enviroment.
9. restart navigation2
10. the robot should detect stuff.

## Member list
Members (GitHub usernames):
- Francesca Paraschiv (francesca66)
- Quinten Potma (potmq)
- Nikola Seleš (Nik315)
- Sanula De Silva (Maxawa)
- Kanupriya Singh (KSingh168)
- Bogdan Spătaru (skneww)
- Petar Zhelev (PetarZh123)
