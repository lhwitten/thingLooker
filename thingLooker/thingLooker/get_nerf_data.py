import rclpy  # ROS2 client library for Python
from rclpy.node import Node  # Base class for creating ROS nodes
from geometry_msgs.msg import Quaternion  # For handling orientations in ROS
import json  # For JSON operations
import numpy as np  # Numerical Python library for array operations
import cv2  # OpenCV library for image processing
from cv_bridge import CvBridge, CvBridgeError  # For converting between ROS and OpenCV image formats
from .angle_helpers import *  # Import helper functions for angle calculations
from sensor_msgs.msg import CompressedImage  # For handling compressed image data in ROS
from geometry_msgs.msg import PoseStamped  # For handling pose data (position + orientation) with a timestamp
from .compare_images import *  # Import functions for image comparison

def wait_for_save_command(node):
    input("Press 'Enter' to shut down...")  # Waits for Enter key to shut down

class data_grabber(Node):
    def __init__(self):
        """
        Initialize the data_grabber node, setting up subscriptions, variables, and a timer.
        """
        super().__init__('data_grabber')  # Initializes the node
        self.bridge = CvBridge()  # Creates a bridge between ROS and CV image formats
        self.shutdown_flag = False  # Flag to indicate shutdown
        # Subscribes to compressed image and pose data topics
        self.create_subscription(CompressedImage, 'camera/image_raw/compressed', self.callback, 10)
        self.create_subscription(PoseStamped, 'device_pose', self.get_odom, 10)
        self.last_cam = None  # Placeholder for the last camera image
        self.image = None  # Current image placeholder
        # Robot's position and orientation placeholders
        self.xpos = self.ypos = self.zpos = self.theta = 0.0
        self.cam_phi = np.pi/16  # Camera's orientation offset
        # Placeholders for last turn position and orientation
        self.xpos_b = self.ypos_b = 0.0
        self.orientation_bench = self.orientation = Quaternion()
        self.wait = False  # Wait flag
        self.w_count = 0  # Wait counter
        self.target_size = None  # Target size placeholder
        self.counter = 0  # Image counter
        # JSON structure for camera model and frames
        self.json = {
            "camera_model": "OPENCV", 
            "fl_x": 506.65, "fl_y": 507.138, 
            "cx": 383.64, "cy": 212.61, 
            "w": 768, "h": 432, 
            "k1": -0.0329, "k2": 0.058, 
            "p1": 0.000255, "p2": 0.0, 
            "aabb_scale": 16, 
            "frames": []
        }
        self.timer = self.create_timer(2.0, self.run_loop)  # Timer for periodic task execution

    def run_loop(self):
        """
        Processe and saves each received image periodically.

        Convert ROS images to OpenCV format, performs rotations, and saves them.
        Also updates the JSON structure with transformation data.
        """
        # Processes images if available
        if self.image:
            try:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(self.image, "passthrough")
                rotated_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
                cv2.imshow("window", rotated_image)
                cv2.waitKey(1)
            except CvBridgeError as e:
                print(e)
            # Saves processed image and updates JSON
            self.file_name = f"/home/jess/ros2_ws/image_data/img{self.counter}.png"
            self.counter += 1
            cv2.imwrite(self.file_name, rotated_image)
            self.transform_as_list = Rt_mat_from_quaternion_44(
                self.orientation.x, self.orientation.y, 
                self.orientation.z, self.orientation.w, 
                self.xpos, self.ypos, self.zpos
            ).tolist()
            self.frame_dict = {"file_path": self.file_name, "transform_matrix": self.transform_as_list}
            self.json["frames"].append(self.frame_dict)
        else:
            print("NO IMAGE")
        self.save_json_to_file()

    def get_odom(self, odom_data):
        """
        Callback function for receiving pose data.

        Args:
            odom_data (PoseStamped): The incoming odometry data containing position and orientation.
        
        Updates the node's position (xpos, ypos, zpos) and orientation based on odometry data.
        """
        # Updates position and orientation from odometry data
        self.xpos, self.ypos, self.zpos = (
            odom_data.pose.position.x, 
            odom_data.pose.position.y, 
            odom_data.pose.position.z
        )
        self.orientation = odom_data.pose.orientation
        self.theta = euler_from_quaternion(
            self.orientation.x, self.orientation.y, 
            self.orientation.z, self.orientation.w
        )

    def callback(self, image_data):
        """
        Callback function for receiving image data from the camera.

        Args:
            image_data (CompressedImage): The incoming image data from the camera.
        
        Updates the current image for further processing.
        """
        self.image = image_data  # Updates current image with incoming data

    def save_json_to_file(self):
        """
        Save json dictionary as json file using json.dump.
        """
        # Saves JSON data to a file
        with open('/home/jess/ros2_ws/transforms.json', 'w') as outfile:
            json.dump(self.json, outfile, indent=4)

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2 client library
    node = data_grabber()  # Create a data_grabber node
    rclpy.spin(node)  # Keep the node running
    node.save_json_to_file()  # Save JSON data before shutdown
    rclpy.shutdown()  # Shutdown ROS2 client library
