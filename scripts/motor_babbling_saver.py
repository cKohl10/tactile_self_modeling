import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import csv
import os
from datetime import datetime
import sys

def main(save_dir):
    # Initialize ROS2
    rclpy.init()
    
    # Create a simple node
    node = Node('motor_babbling_saver')
    
    # Initialize variables and structures
    cv_bridge = CvBridge()
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    index = 0
    save_dir = save_dir + f"/motor_babbling_data_{timestamp}"
    os.makedirs(save_dir, exist_ok=True)
    
    # Initialize CSV file
    csv_path = os.path.join(save_dir, 'joint_angles.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'index', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'])
    
    # Create data storage
    joint_angles = None
    
    def joint_callback(msg):
        nonlocal joint_angles
        joint_angles = msg.position
    
    def image_callback(msg, camera_index):
        try:
            cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_path = os.path.join(save_dir, f'camera_{camera_indices[camera_index]}_{index}.png')
            cv2.imwrite(image_path, cv_image)
        except Exception as e:
            print(f'Error saving image: {str(e)}')

    # Set up subscriptions
    joint_sub = node.create_subscription(
        JointState,
        '/franka/command/joint_states',
        joint_callback,
        2
    )
    
    camera_topics = ['/rgb/front', '/rgb/left', '/rgb/right', '/rgb/back', '/rgb/above']
    camera_indices = ['front', 'left', 'right', 'back', 'above']
    image_subs = {}
    
    for camera, idx in zip(camera_topics, camera_indices):
        image_subs[camera] = node.create_subscription(
            Image,
            camera,
            lambda msg, idx=idx: image_callback(msg, idx),
            2
        )
    
    def capture_data():
        nonlocal index
        try:
            # Save joint angles
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S.%f')
            with open(csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, index] + list(joint_angles))
            print(f'Capture {index} saved')
            index += 1
            return True
        except Exception as e:
            print(f'Error saving capture: {str(e)}')
            return False

    # Example usage in a loop
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)  # Process callbacks
            user_input = input("Press Enter to capture (or 'q' to quit): ")
            if user_input.lower() == 'q':
                break
            if capture_data():
                print("Data captured successfully")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Please provide a save directory path as the first argument")
        sys.exit(1)
    
    main(sys.argv[1])
