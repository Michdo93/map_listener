# map_listener

ROS cpp package which listens and subscribes to the map topic. This mean it saves an occupancy grid map as pgm file.

## Installation

Go to the `catkin_ws/src` folder and clone the repository:

```
cd ~/catkin_ws/src
git clone https://github.com/Michdo93/map_listener.git
cd ~/catkin_ws
catkin_make
```

## Usage

At first you have to run a ROS program which publishes to the topic `map`. Equivalent to

```
rosrun map_server map_saver -f ~/map
```

you can now run:

```
rosrun map_listener mapListener.cpp
```
