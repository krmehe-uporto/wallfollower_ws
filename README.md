# IR - Assignment 1: wall follower robot

Our wall follower robot runs using the Flatland simulation enviroment. This simulation platform has an already implemented differential drive robot, that responds to velocity and rotation messages. 
To run our program:
- install Flatland simulator following their gude (https://github.com/avidbots/flatland)
- download the this repository (include robot node, map, simulation configuration)
- run roscore
- run "catkin_make" to build the packages
- run "roslaunch flatland_server server.launch world_path:=[absolute path to package]/wallfollower_ws/src/wall_robot/src/yamls/world_empty.yaml" to launch simulation enviroment
- launch the robot program: "rosrun wall_robot wall_robot_node"
- to change the map open world_empty.yaml and change the "map:" parameter to the map you want to open. 

There are 3 maps currently in the project: 
- a hand drawn "Ç"
- a bigger "Ç" with thick walls
- a bigger but thin "Ç" shape

The robot after having reached the endpoint, waits for 2 seconds, then respawns to a random location around the center of the map. Every time the robot restarts it starts to save its coordinates to a .CSV file with a timestamp as the title and measures the runtime wich is added at the end of the file.

