# Technical Reflection of Team 22
In this technical reflection, we are going to break down what we have achieved with our implementation, 
what we chose not to complete within our remaining time, the bottlenecks or challenges we faced, 
and what could be improved or expanded given additional time.

Username: `team22` \
Password: `SadSmiley22`

### Table of Contents
- [Bidirectional Communication](#bidirectional-communication)
- [State Synchronization](#state-synchronization)
- [Environmental & Object Interaction](#environmental--object-interaction)

## Bidirectional Communication
In our solution, the physical TurtleBot sends map data and other navigation data to the digitally twinned TurtleBot,
where the information is processed to identify differences between the baseline and new map. 
The coordinates of these differences are then passed on back to the real TurtleBot, so that it will navigate towards them.

<img src="simplified bidirectional comm outlined.png" width="320" height="203" />

This data exchange is established using Unity in combination with ROS 2.

### Achievements
The digital twin receives real sensor data through ROS 2 in the form of costmaps, and it is able to save these.
After processing these maps through some layers (such as shader filters), the navigation system receives the correct goal from the digital twin.
It is then able to navigate to these coordinates, afterwards notifying the user that is has reached the destination.

### Not completed
The physical TurtleBot does not communicate camera data to the virtual side, as we have not integrated the RPi-camera with the TurtleBot.
This meant that the digital twin receives less data from the physical twin, and so it has less information to generate a response back to the real robot.
We were also unable to implement a camera to take pictures of the area due to time constraints.

### Challenges faced
The Docker image we used to simulate a ROS 2 workspace was facing issues connecting to Unity, hindering the data exchange until we found a fix.
Furthermore, the original SLAM mapping software that was used (`slam_toolbox`) also failed to work with the physical TurtleBot.
This was eventually resolved by switching to a different software (`cartographer`).
Lastly, Unity tried to send sensor data to our implementation, which interfered with processing the right sensor data.
This resulted in incorrect navigation goals being set, which momentarily confused us during development. To fix this, certain virtual sensors were selectively disabled.
During the last labsessions, we faced issues with certain topics not showing up in `ros2 topic list` and the Unity scene not receiving maps from navigation properly.
We managed to figure out that the topic issue was due to the node finding system that ros implements not being updated fast enough. This was fixed by running `ros2 daemon stop`.
As for the Unity scene not receiving maps, that turned out to be an issue with `ros_tcp_endpoint`, which a simple restart fixed.

### What could be improved/expanded
The user experience of our current implementation is still quite technical, 
as it requires multiple separate Linux command-line interfaces and multiple programs to run in synchronization.
To a safety inspector, our current implementation may not be immediately intuitive, which is something we want to avoid.
A better interface would be a single GUI consisting of visualizations, relevant settings and commands the user could make use of to control the system.
With a few more weeks, it would be possible to implement the camera system as initially planned.


## State Synchronization
### Achievements
The digital twin listens to the same navigation commands as the physical twin, and moves similarily

### Not completed 
The friction and damping values of the physical and digital turtlebot appear to be different. 
However, it is not as simple as just these values being wrong, these are just the best comparison.
Additionally, the starting position of the digital turtlebot does not necessarily match that of the physical twin.

### Challenges faced 
The Unity project given by the course had several bugs in it, costing a lot of time: 
  - The /tf topic did not work because of a missing component on the base_footprint GameObject.
        This was resolved by finding the correct component to add by looking at a different example project.
  - The movement of the digital twin did not work, because of faulty if-else statements in its navigation code.
        A fix to the if-else statement resolved this issue
    
### What could be improved/expanded
With more time, we could correct the starting position of the digital turtlebot (properly synchronizing according to the `/tf` topic).
The movements of the twins could also be more finely tuned to match well.

## Environmental & Object Interaction
### Achievements
The digital twin can accurately compare two full maps of the environment, and it reliably locates changes between the two.
There is a system to filter noise out of the comparison map, involving a blur filter and thresholds to decide what is noise and what is significant change. 
The map comparison algorithm runs extremely fast, as there is no obvious impact on performance and no noticeable delay in response by the navigation.
Functions have been written to convert between Unity coordinates and relative ROS2 navigation coordinates.

### Challenges faced
We initially faced issues with the accuracy of SLAM maps. The resolution was simply not high enough to accurately compare new obstacles. Increasing the resolution of the SLAM map also added too much noise to comfortably deal with.
Eventually, the solution was found to be to use the costmap, which is a version of the map used by the navigation package instead, as it offered much better visibility for obstacles.
When initially comparing the maps, we were met with large performance issues. The CPU was simply not fast enough to iterate over all the pixels of a 512 x 512 texture every map update without noticeable performance stutter.
The performance issue was resolved by switching to compute shaders, which offload the calcualations to the GPU, which is much faster for this application. This also allowed us to implement fairly extensive filtering at no noticeable performance cost.

We found that the turtlebot's navigation simply did not work when placed in a small area, which was resolved by simply making the area bigger and by changing the `inflation_radius` parameter in ros.
Our final challenge was converting from the coordinates of a pixel on the texture of the map to Unity coordinates, and from Unity coordinates to the coordinates used by the navigation package.
The messages published to the map topic contained data about its location, which was used to convert from texture coordinates to Unity coordinates,
and Unity turned out to have a built in function to convert from their coordinate system to that of ros2 navigation.

### What could be improved/expanded
With some more time, we could find more optimal parameters for the map resolution and the navigation inflation_radius. We could also
change the code to support finding multiple obstacles and including the detection of removed obstacles (currently, the code only detects obstacles that have been newly added).
If a camera were to be implemented, that could also be used to aid in obstacle detection (through image recognition algorithms).
