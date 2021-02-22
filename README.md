# idp-simulation
Controller code for University of Cambridge Engineering IB IDP Simulation, Group L101.

The task given was to collect 8 boxes in an arena - 4 blue, 4 red. Each box can only be collected by a robot of the same colour. The robots are given 5 minutes to complete this task.

# Usage
![Simulation Footage](https://github.com/TobyBoyne/idp-simulation/blob/master/report/images/simulation.gif)

To run this project, download [Webots](https://cyberbotics.com/). Then, open one of the worlds in the `worlds` folder, and run the simulation.

# Software Approach

### Controllers

There are 3 robots in the simulation. One Shared controller, that facilitates communication between the two Collector controllers. Shared has an internal map of the arena that is updated after either of the Collectors use their sensors. This map is then used to determine the next step that a robot should take, and commands are sent to the robots once they have finished their previous command. A full list of commands can be found in Table 1 in the Appendix.

### Communication

Initially, the Shared controller sends a command to each of the robots – e.g. (&#39;MOV&#39;, 1.0, 0.5). Once the Collector has completed this command, it returns a &#39;DNE&#39; command which updates the position of the robot in the Shared internal map. The next command is then issued, and this loop repeats until all the blocks have been collected.

The robots can also send other commands to the Shared controller to provide additional information, such as the colour or position of a box.

![Flowchart](https://github.com/TobyBoyne/idp-simulation/blob/master/report/images/flowchart.png)

_Flowchart showing all possible states of the Collector robot. After a task is completed, it will move to the next instruction in the flowchart_

# Code Structure and Algorithms

![Structure](https://github.com/TobyBoyne/idp-simulation/blob/master/report/images/code_structure.png)

_Code structure diagram showing the key attributes and functions of the classes used_

Two of the functions are outlined below:

### Scanning

Function `findClusters(data_points)`

- The current distance sensor reading and compass direction are used to determine a point in space at the end of the distance sensor&#39;s ray
- After a full rotation, have a list of all positions of data\_points
- Use k-means clustering to find the centroids of the boxes, ignoring data points that lie outside the arena (i.e. the walls providing a false-positive) or outside the scanning radius
- Return the location of each box in separate &#39;BOX&#39; commands to the Shared controller

### Pathing

Function `findPath(robot, waypoint)`

- Find the vector that the robot will move, `move_vec = waypoint – robot.position`
- If move\_vector is longer than MAX\_DISTANCE, shorten the path
- If the minimum distance between move\_vector and all the known boxes is above a threshold, then the path is clear and can be moved
- Otherwise, rotate move\_vector until a clear path is found
- Return a new waypoint based on rotated vector, `target = robot.position + move_vec`
- If target is too far away from the original waypoint, then run findPath again.

The MAX\_DISTANCE variable is used to ensure that the Collector bot doesn&#39;t move out of the region that has already been scanned.

The Collector robot then moves to target in a straight line. Some control theory is used to ensure it is facing the right direction.

# All commands

| **Description** | **Command** | **Argument 1** | **Argument 2** |
| --- | --- | --- | --- |
| _ **Collector robot to shared controller** _ |
| Let the shared controller know that the current task is completed | DNE | Robot X | Robot Z |
| Tell the shared controller the position of a box | BOX | Box X | Box Z |
| Mark the colour of a box | CLR | Colour (1 red, 0 blue) | Whether a box was found (1 yes, 0 no) |
| _ **Shared controller to collector robot** _ |
| Scan in a circle, and read from the distance sensor | SCN | - | - |
| Move to a specified waypoint | MOV | Target X | Target Z |
| Identify the colour of the box in front | IDN | - | - |
| Do nothing for a specified time | IDL | Time (seconds) | - |
| Close the claws to collect a block | COL | - | - |
| Return home (same as MOV but doesn&#39;t trigger distance sensor) | RTN | Target X | Target Z |
| Open the claws to release a block | RLS | - | - |
