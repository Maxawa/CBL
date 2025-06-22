# Technical Reflection of Team 22
In this technical reflection, we are going to break down what we have achieved with our implementation, what we chose not to complete within our remaining time, the bottlenecks or challenges we faced, and what could be improved or expanded given additional time.

Contents:
- [Bidirectional Communication](#bidirectional-communication)
- [State Synchronization](#state-synchronization)
- [Environmental & Object Interaction](#environmental--object-interaction)

Username: team22
Password: SadSmiley22

## Bidirectional Communication
In our solution, the physical turtlebot sends map and other navigation data to the digital twin, where the information is 
processed to identify differences between the baseline and new map. The coordinates of these differences are passed on to the real turtlebot to navigate towards.
<img src="simplified bidirectional comm outlined.png" width="320" height="203" />
#Achievements
- The digital twin can save costmaps
- The navigation system receives the correct coordinates
Challenges:
- The docker image was facing issues connecting to unity
- Unity interfered with the physical robot's sensors
- The original SLAM mapping program that was used (slam_toolbox) failed to work with the physical robot 

## State Synchronization
Achievements:
- Digital Twin listens to the same navigation

## Environmental & Object Interaction
#Achievements
- The digital twin can accurately compare the maps
- There is a system to filter noise out of the comparison map, involving blurring and thresholds
- The map comparison algorithm runs extremely fast (no obvious impact on performance)
- Functions have been made to convert between Unity coordinates and ROS2 navigation coordinates
#Challenges
- 
