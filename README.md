# Structural Integrity Bot
2IRR10 CBL Autonomous Systems Twinning project, team 22.

## Usage guide
Clone this repo via GitHub Desktop.
It will ask you to initialize LFS or something, click yes.
Open the project in Unity and open the 'test' scene in the Scenes folder
The scripts that read the map and do calculations are attached to the MainCamera GameObject.

Setup:
This is for testing it virtually, in the real case this won't be needed (we will be using cartographer).
Go to https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example and download the unity_slam_example folder at ros2_docker/colcon_ws/src/unity_slam_example

In your ros workspace, put that folder inside the src folder

now you can run ros2 launch unity_slam_example unity_slam_example.py before starting the Unity Project, it will map and launch rviz.

Expected Usage:
When running the unity project with something that publishes to map (cartographer or the slam example shown above),
Three maps will appear in the top right, one showing the old map, one showing the map currently being made, and the last one showing the differences between the two.

You can control the turtlebot via slam navigation.

If something breaks when importing the Unity Project, Quinten probably knows how to fix it lol

## Member list
Members • GitHub usernames:
- Francesca Paraschiv • francesca66
- Quinten Potma • potmq
- Nikola Seleš • Nik315
- Sanula De Silva • Maxawa
- Kanupriya Singh • KSingh168
- Bogdan Spătaru • skneww
- Petar Zhelev • PetarZh123
