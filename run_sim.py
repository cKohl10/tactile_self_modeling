import isaacsim
import omni
import yaml
import sys
import rclpy


def run_sim(usd_file_path):
    kit = isaacsim.SimulationApp({"headless": True})
    load_delay = 3 # Isaac Sim can crash if the scene is loaded too quickly

    # Let the simulation run for 3 seconds
    for i in range(load_delay):
        kit.update()
    
    # Initialize ROS2 first
    rclpy.init()
    
    import omni.isaac.core.utils.extensions as extensions
    
    # enable ROS2 bridge extension
    extensions.enable_extension("omni.isaac.ros2_bridge")
    extensions.enable_extension("contact_ext")

    # Let the simulation run for 3 seconds
    for i in range(load_delay):
        kit.update()

    # Load the USD scene
    if omni.usd.get_context().open_stage(usd_file_path):
        print(f"Successfully loaded scene: {usd_file_path}")
    else:
        print(f"Failed to load scene: {usd_file_path}")
        kit.close()
        exit()

    # Play the simulation
    omni.timeline.get_timeline_interface().play()

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