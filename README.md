# multimap_utils ROS Package

## Overview
The `multimap_utils` package provides functionality to handle multiple maps and enables seamless navigation across different maps in a robotics environment. This package uses wormhole points to navigate between maps and localize the robot in a target map. The key components of this package are two nodes:
- **`multimap_navigator`**: Handles the navigation between maps and localization using wormhole points.
- **`steer_to_goal`**: Sends a navigation goal to the action server for the robot to reach a specified location in a list of maps.

## Nodes

### `multimap_navigator`
The `multimap_navigator` node is responsible for managing the navigation across multiple maps using wormhole points. It switches between maps, localizes the robot in the target map, and navigates it to specified positions within the map. It provides an action server that listens for goals related to switching maps and navigating between them.

### `steer_to_goal`
The `steer_to_goal` node is a simple client that sends a navigation goal to the `MultiMapGoalActionServer`. It allows the user to input target coordinates and map names from the command line, and it sends this data as a goal to the server. The server processes the goal and navigates the robot to the target position within the specified map.

## Features
- **Multi-map Navigation**: Switches between different maps and ensures the robot is localized in the right map before moving.
- **Wormhole Navigation**: Uses predefined wormhole points to transition the robot between maps.
- **Action Server**: The action server (`MultiMapGoalActionServer`) listens for navigation goals and processes them.
- **Action Client**: The action client (`mm_client`) sends a goal to the action server to move the robot to a specified position on a given map.

## Installation
To use the `multimap_utils` package, ensure that you have ROS installed on your system. Then, follow these steps to set up the package:

1. Clone the repository:
    ```bash
    git clone <repository_url>
    ```
2. Build the package:
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```
3. Source the workspace:
    ```bash
    source devel/setup.bash
    ```

## Usage

### Running the Action Server (`multimap_navigator`)
Start the action server by running the following command:
```bash
rosrun multimap_utils multimap_navigator
```

### Running the Action Client (`steer_to_goal`)
Start the action server by running the following command:
```bash
rosrun multimap_utils steer_to_goal -10 -10 room1
```


## Steps to create the db file:

### Create a New SQLite Database:

If you don't already have a database file, you can create a new one with:
```bash
sqlite3 wormhole_graph.sqlite3
```
### Execute the SQL Script:

Save the SQL script above into a file named seed.sql, and then execute it inside the SQLite terminal like this:
```bash
.read seed.sql
```

### Verify the Data:

After executing the script, you can check that the tables have been populated with the correct data by running:

    SELECT * FROM maps;
    SELECT * FROM wormholes;
    SELECT * FROM map_wormhole_relations;

The timestamps will automatically be set to the current time when the data is inserted.
