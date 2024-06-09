# CheckPoint 9 Mobile Robot Kinematics

<a name="readme-top"></a>

## About The Project
The purpose of this project is create a package that calculates the kinematics of a holonomic robot. Also, using a PID controller the robot performs a eght shaped trajectory.

![This is an image](images/preview.png)

<!-- GETTING STARTED -->
## Getting Started

### Software Prerequisites
* Ubuntu 22.04
* ROS2 Humble


<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- INSTALLATION -->
### Installation
1. Clone the repo:
   ```sh
   cd ~ && \
   git clone https://github.com/pvela2017/The-Construct-CheckPoint-9-Mobile-Robot-Kinematics
   ```
2. Compile the simulation:
   ```sh
   source /opt/ros/humble/setup.bash && \
   cd ~/The-Construct-CheckPoint-9-Mobile-Robot-Kinematics/ros2_ws && \
   colcon build
   ```
     
<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- USAGE -->
## Usage
### Local Simulation
1. Launch the simulation:
   ```sh
   source /opt/ros/humble/setup.bash && \
   source ~/The-Construct-CheckPoint-9-Mobile-Robot-Kinematics/ros2_ws/install/setup.bash && \
   ros2 launch rosbot_xl_gazebo simulation.launch.py
   ```
2. Publish wheels velocities:
   ```sh
   source /opt/ros/humble/setup.bash && \
   source ~/The-Construct-CheckPoint-9-Mobile-Robot-Kinematics/ros2_ws/install/setup.bash && \
   ros2 run wheel_velocities_publisher wheel_velocities_publisher
   ```
3. Launch the kinematic model:
   ```sh
   source /opt/ros/humble/setup.bash && \
   source ~/The-Construct-CheckPoint-9-Mobile-Robot-Kinematics/ros2_ws/install/setup.bash && \
   ros2 launch kinematic_model kinematic_model.launch.py
   ```
4. Eight trajectory:
   ```sh
   source /opt/ros/humble/setup.bash && \
   source ~/The-Construct-CheckPoint-9-Mobile-Robot-Kinematics/ros2_ws/install/setup.bash && \
    ros2 launch eight_trajectory eight_trajectory.launch.py
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- KEYS -->
## Key topics learnt
* Kinematics.
* PID.
