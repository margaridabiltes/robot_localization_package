# robot_localization_package

## Description
* ROS2 Particle Filter implementation for robot localization.
* The particle filter estimates the robot's pose (position and orientation) in a known map using odometry and detected features.
* This package is designed to work together with the following packages:
    * [robot_worlds](https://github.com/JMCOliveira02/robot_worlds)
    * [robot_msgs](https://github.com/JMCOliveira02/robot_msgs)

## Installation
1. Clone this repository into the `src` folder of your ROS2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/YourUsername/robot_localization_package.git
    ```
2. Install dependencies:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
3. Build the workspace:
    ```bash
    cd ~/ros2_ws
    colcon build
    ```
4. Source the workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

## Usage
The `pf_launch.py` file launches the particle filter node along with supporting nodes for simulation, visualization, and feature detection. Below are the details of the topics, transforms, and nodes involved.

### Subscribed Topics
* **`/odom`** (`nav_msgs::msg::Odometry`)  
  The estimated odometry of the robot. This topic is used in the **motion update** step of the filter.
* **`/features`** (`robot_msgs::msg::FeatureArray`)  
  The detected features' positions and orientations, from the robot's reference frame. This topic is used in the **measurement update** step of the filter.

### Published Topics
* **`/estimated_pose`** (`geometry_msgs::msg::PoseStamped`)  
  The estimated pose of the robot based on the particle filter.
* **`/particles`** (`visualization_msgs::msg::MarkerArray`)  
  The positions and orientations of all particles in the filter, published as markers for visualization in RViz. Add the topic to RViz to see the particle cloud.

### TF Broadcasts
The particle filter broadcasts the following static transforms to define the relationship between different coordinate frames:
* **`map -> odom`**  
  A static transform published to align the robot's odometry frame (`odom`) with the map frame (`map`).
* **`base_footprint -> lidar2D`**  
  A static transform published to define the position of the lidar sensor relative to the robot's base.

### Launch Files
* **`pf_launch.py`**  
  Launches the following nodes and tools:
  - **Webots Simulation**:
    - Launches the Webots simulator with the specified world file.
  - **Robot Controller**:
    - Loads the robot's URDF and controls the robot in the simulation.
  - **Fake Detector**:
    - Simulates feature detection for the particle filter.
  - **Particle Filter**:
    - Runs the particle filter node for localization.
  - **Map Server**:
    - Provides the map as a service for other nodes.
  - **RViz**:
    - Visualizes the robot, particles, and map.
  - **Teleop** (optional):
    - Allows manual control of the robot using the keyboard.

  Example usage:
    ```bash
    ros2 launch robot_localization_package pf_launch.py
    ```

## Internal Libraries

This package includes the following internal libraries to support the particle filter:

### **MapLoader**
The `MapLoader` is responsible for loading map features from a YAML file into a global feature map. The YAML file is provided by the `robot_worlds` package and is linked in the launch file (`pf_launch.py`).

- **Key Functions**:
  - `loadToGlobalMap(const std::string& yaml_path)`: Loads features from the YAML file into the global feature map.
  - `getGlobalFeatureMap()`: Provides access to the global feature map.

The particle filter uses the `MapLoader` to load and access the map features during the measurement update step.

### **FeatureStruct**
The `FeatureStruct` provides the data structures used to represent map features. These include:
- **`FeatureCorner`**: Represents a corner feature with position and orientation.
- **`FeatureObject`**: Represents an object feature with position, orientation, and associated keypoints.

These structures are used by the particle filter to:
- Represent expected features in the global map.
- Compute likelihoods during the measurement update step.

### **How They Work Together**
1. The `MapLoader` loads features from the YAML file provided by the `robot_worlds` package.
2. The features are stored in a global feature map, which the particle filter uses to compute expected observations for particles.
3. The `FeatureStruct` provides the necessary data structures to represent these features and their properties.

## Dependencies
This package depends on the following ROS2 packages:
* `rclcpp`
* `geometry_msgs`
* `nav_msgs`
* `visualization_msgs`
* `tf2_ros`
* `robot_msgs`
* `nav2_map_server`
* `nav2_lifecycle_manager`
* `rviz2`
* `teleop_twist_keyboard`

## Contributing
Contributions are welcome! Please fork this repository and submit a pull request with your changes.

## License
This project is licensed under the Apache-2.0 License.