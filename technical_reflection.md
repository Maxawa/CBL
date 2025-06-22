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

This data exchange is established using our Unity implementation in combination with ROS 2.

### Achievements
The digital twin receives real sensor data through ROS 2 in the form of costmaps, and it is able to save these.
After processing these maps through some layers (such as shader filters), the navigation system receives the correct goal from the digital twin.
It is then able to navigate to these coordinates, afterwards notifying the user that is has reached the destination.

### Not completed
The physical TurtleBot does not communicate camera data to the virtual side, as we have not integrated the RPi-camera with the TurtleBot.
This meant that the digital twin receives less data from the physical twin, and so it has less information to generate a response back to the real robot.

### Challenges faced
The Docker image we used to simulate a ROS 2 workspace was facing issues connecting to Unity, hindering the data exchange until we found a fix.
Furthermore, the original SLAM mapping software that was used (`slam_toolbox`) also failed to work with the physical TurtleBot.
This hindered bidirectional communication as we could not test the first part of our bidirectional communication.
Lastly, Unity tried to send sensor data to our implementation, which interfered with processing the right sensor data.
This resulted in incorrect navigation goals being set, which momentarily confused us during development.

### What could be improved
The user experience of our current implementation is still quite technical, 
as it requires multiple separate Linux command-line interfaces and multiple programs to run in synchronization.
To a safety inspector, our current implementation may not be immediately intuitive, which is something we want to avoid.
A better interface would be a single GUI consisting of visualizations, relevant settings and commands the user could make use of to control the system.

### What could be expanded
- Something about updating the navigation goal based on camera data/AI obstacle recognition?
- TODO

## State Synchronization
### Achievements
- The digital twin listens to the same navigation as the physical twin
- There is no noticeable inconsistency between the relative positions of the twins
- TODO

### Not completed 
- TODO

### Challenges faced 
- TODO

### What could be improved
- TODO

### What could be expanded
- TODO

## Environmental & Object Interaction
### Achievements
The digital twin can accurately compare two full maps of the environment, and it reliably locates changes between the two.
There is a system to filter noise out of the comparison map, involving a blur filter and thresholds to decide what is noise and what is significant change. 
The map comparison algorithm runs extremely fast, as there is no obvious impact on performance and no noticeable delay in response by the navigation.
Functions have been written to convert between Unity coordinates and relative ROS2 navigation coordinates.

### Not completed
- Camera documenting changes
- Multiple change detection and navigation to each change
- TODO

### Challenges faced
- Bottleneck: time to implement the not completed above
- TODO

### What could be improved
- Trial and error in lab sessions, to find the most reliable parameters and tresholds
- TODO

### What could be expanded
- AI obstacle recognition
- Delivering equipment to obstacles as a form of object interaction
- TODO

