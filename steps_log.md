# # Running the visual self modeling baseline:
Follow the install instructions for the original self modeling paper [here](https://github.com/BoyuanChen/visual_self_modeling)

# Setting up the simulated environment:
- Install the Omniverse launcher
- Install Isaac Sim 4.2.0
- Install ROS2 Humble natively only (Using Ubuntu 22.04) https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html
- Install Isaac ROS workspace
- Clone tactile_msgs & tactile_examples
- Download training environments
- Create isaac sim virtual environment to use their python interpreter https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html
- Create an alias for the isaac sim python interpreter "omni_python"

# SDF generation
`sudo apt-get install build-essential git python3-dev python3-pip libopenexr-dev libxi-dev \
                     libglfw3-dev libglew-dev libomp-dev libxinerama-dev libxcursor-dev`
- Instant ngp https://github.com/NVlabs/instant-ngp

# Config
- The world must be loaded with the robot's contact sensors already on. Load the basic world, put the sensors on the robot, save the scene under a new name, then change the config file to point to this new scene.
- Now, load in the sensors again, this time the sensors can log simulated data.


# Running the basic motor babbling
- omni_python run_sim.py config1
- ros2 run tactile_examples motor_bab_pub
- ros2 run tactile_examples motor_bab_saver

# Useful Resources
[Ros 2 in Standalone Mode](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_python.html)

# Papers
[Instant NGP](https://nvlabs.github.io/instant-ngp/assets/mueller2022instant.pdf)
[COLMAP](https://colmap.github.io/install.html#build-from-source)
[GLOMAP](https://github.com/colmap/glomap)
[Dr. Robot](https://drrobot.cs.columbia.edu/assets/dr-robot.pdf)
[Egocentric self modeling](https://arxiv.org/pdf/2207.03386)