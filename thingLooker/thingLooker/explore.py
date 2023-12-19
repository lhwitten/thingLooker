import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import math
import json
import numpy as np
import subprocess
import message_filters
import threading
import cv2
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
import matplotlib.pyplot as plt
from .angle_helpers import *
from PIL import Image as img
import io
from .compare_images import *
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger

def wait_for_save_command(node):
    input("Press 'Enter' to shut down...")

##robot knows where it is
class position_knower(Node):
    def __init__(self):
            super().__init__('position_knower')
            self.bridge = CvBridge()
            self.shutdown_flag = False
            self.create_subscription(CompressedImage, 'camera/image_raw/compressed',self.callback,10)
            #qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=ReliabilityPolicy.SYSTEM_DEFAULT)  # or ReliabilityPolicy.BEST_EFFORT
            self.create_subscription(PoseStamped, 'device_pose', self.get_odom, 10)
            qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=ReliabilityPolicy.SYSTEM_DEFAULT)  # or ReliabilityPolicy.BEST_EFFORT
            #self.create_subscription(Odometry, '/odom', self.get_odom, qos_profile_sensor_data)
            #self.create_subscription()
            self.last_cam = None
            self.image = None
            #camera/image_raw
            #self.vel_pub = self.create_publisher(Twist,'cmd_vel',10)
            #robot position
            self.xpos = 0.0
            self.ypos = 0.0
            self.zpos = 0.0
            self.theta = 0.0

            self.cam_phi = np.pi/16
            #self.angular = 0.0

            #position of last turn
            self.xpos_b = 0.0
            self.ypos_b = 0.0
            self.orientation_bench = Quaternion()

            self.orientation = Quaternion()
            self.wait = False
            self.w_count = 0
            self.target_size = None
            self.counter = 0
            self.json = {"camera_model":"OPENCV","fl_x":506.65,"fl_y":507.138,"cx":383.64,"cy":212.61,"w":768,"h":432,"k1":-0.0329,"k2":0.058,"p1":0.000255,"p2":0.0,"aabb_scale":16,"frames":[]}
            self.timer = self.create_timer(2.0, self.run_loop)

    def run_loop(self):
        #nerf_pic = self.get_nerf_pic(c2w_mat)
        #self.image_compare(c2w_mat)
        if self.image is not None:
            try:
                # Convert the ROS image to an OpenCV image
                cv_image = self.bridge.compressed_imgmsg_to_cv2(self.image,"passthrough")
                print(cv_image)
                cv2.imshow("window",cv_image)
                cv2.waitKey(1)
            except CvBridgeError as e:
                print(e)
            c2w_mat = Rt_mat_from_quaternion(self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w,self.xpos,self.ypos,self.zpos)
            self.image_compare(c2w_mat)
            # Save the image
        else:
            print("NO IMAGE")


    def get_odom(self, odom_data):
        print(odom_data is not None)
        """self.xpos = odom_data.pose.pose.position.x
        self.ypos = odom_data.pose.pose.position.y
        self.orientation = odom_data.pose.pose.orientation"""

        self.xpos = odom_data.pose.position.x
        self.ypos = odom_data.pose.position.y
        self.zpos = odom_data.pose.position.z
        self.orientation = odom_data.pose.orientation


        self.theta = euler_from_quaternion(self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w)

    def callback(self,image_data):
        #print("called")
        #print(image_data is not None)
        self.image = image_data

    def save_json_to_file(self):
        with open('/home/jess/ros2_ws/transforms.json', 'w') as outfile:
            json.dump(self.json, outfile, indent=4)
    
    def image_compare(self,c2w_mat):
        print(f"image in compare? {self.image is True}")
        #camera_img = camera()
        print("NeRF image being collected")
        NeRF_img =  self.get_nerf_pic(c2w_mat)
        print("NeRF image collected")
        size_Nerf = get_image_size(NeRF_img)
        #size_cam = get_image_size(self.last_cam)

        if self.image is not None:
            try:
                # Convert the ROS image to an OpenCV image
                cv_image = self.bridge.compressed_imgmsg_to_cv2(self.image,"passthrough")
                print(cv_image)
                cv2.imshow("window",cv_image)
                cv2.waitKey(1)
                size_cam = cv_image.shape
            except CvBridgeError as e:
                print(e)
        else:
            print("NO PICTUREE")
            pass
        # Determine the smaller size
        target_size = min(size_Nerf, size_cam, key=lambda x: x[0] * x[1])

        # Resize first image
        NeRF_img = resize_image(NeRF_img, target_size)

        
        # Resize second image
        #camera_img = resize_image(self.last_cam, target_size)
        camera_img = resize_image(cv_image, target_size)


        compare_and_visualize_differences(NeRF_img,camera_img)

    def get_nerf_pic(self, c2w_mat):
                 #print(f"c2w_mat: {c2w_mat}")
        list_c2w_mat = c2w_mat.tolist()
        string_c2w = json.dumps(list_c2w_mat)
        #print(f"string_c2w_mat: {str(c2w_mat)}")
        print(f"json: {string_c2w}")
        process = subprocess.Popen(['/home/jess/ros2_ws/run_load_model.sh', string_c2w],
                           stdout=subprocess.PIPE, 
                           stderr=subprocess.PIPE)

        process.communicate()
        #print(f"stdout: {stdout}")
        #rgba = json.loads(received_img_data)
        #print("Received array:", rgba)
        # Convert binary data to an image
        # Python Image Library (PIL / pillow) image
        #image = img.open('/home/jess/ros2_ws/output_image.png')
        image = cv2.imread('/home/jess/ros2_ws/output_image.png')
        return image
        # Now you can process the image as needed, e.g., display it, convert it, etc.
        #image.show()

def main(args=None):
    rclpy.init(args=args)
    node = position_knower()
    rclpy.spin(node)
    node.save_json_to_file()
    rclpy.shutdown()
    