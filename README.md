# Dijkstra
Operates turtlebot3 in ROS

You have to make a file("dijkstra_data.txt").
There is a necessary component.
Data of goals(position.x position.y orientation.z orientation.w).
Separator is a space.

It needs a numbers of node that user entered by keyboard.
Because dynamic allocation is done before reading the file.

The path is generated only on the node and the shortest distance is obtained by dijkstra algorithm.

Turtlebot3 move when user give start and finish nodes.
