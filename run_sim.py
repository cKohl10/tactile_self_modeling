import isaacsim
import omni
import yaml
import sys
import rclpy

from omni.isaac.core.utils.extensions import enable_extension

# enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")

def run_sim(usd_file_path):
    kit = isaacsim.SimulationApp({"headless": False})

    # Load the USD scene
    if omni.usd.get_context().open_stage(usd_file_path):
        print(f"Successfully loaded scene: {usd_file_path}")
        kit.update()
    else:
        print(f"Failed to load scene: {usd_file_path}")
        kit.close()
        exit()

    # Initialize ROS2
    rclpy.init()
    kit.update()

    while kit.is_running():
        kit.update()

    # Stop the simulation
    rclpy.shutdown()
    omni.timeline.get_timeline_interface().stop()
    kit.close()

if __name__ == "__main__":
    # Get the path to the USD file from the config file as the seccond command line argument
    config_file_path = "configs/" + sys.argv[1] + ".yaml"
    
    with open(config_file_path, "r") as file:
        config = yaml.load(file, Loader=yaml.FullLoader)
    usd_file_path = config["training_env_path"] + "/" + config["scene_name"] + ".usd"
    run_sim(usd_file_path)