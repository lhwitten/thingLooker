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
from .explore import position_knower
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

def wait_for_save_command(node):
    input("Press 'Enter' to shut down...")

##robot knows where it is
class data_grabber(Node):
    def __init__(self):
            super().__init__('data_grabber')
            self.bridge = CvBridge()
            self.shutdown_flag = False
            self.create_subscription(Image, 'camera/image_raw',self.callback,10)
            qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=ReliabilityPolicy.SYSTEM_DEFAULT)  # or ReliabilityPolicy.BEST_EFFORT
            self.create_subscription(Odometry, '/odom', self.get_odom, qos_profile_sensor_data)
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
            self.json = {"camera_model":"OPENCV","f1_x":506.65,"f1_y":507.138,"cx":383.64,"cy":212.61,"w":432,"h":432,"k1":-0.0329,"k2":0.058,"p1":0.000255,"p2":0.0,"aabb_scale":16,"frames":[]}
            self.timer = self.create_timer(1.0, self.run_loop)

    def run_loop(self):
        #nerf_pic = self.get_nerf_pic(c2w_mat)
        #self.image_compare(c2w_mat)
        if self.image is not None:
            try:
                # Convert the ROS image to an OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(self.image,"passthrough")
                print(cv_image)
                cv2.imshow("window",cv_image)
                cv2.waitKey(1)
            except CvBridgeError as e:
                print(e)
            # Save the image
            self.file_name = "/home/jess/ros2_ws/image_data/img" + str(self.counter) + ".png"
            self.counter += 1
            cv2.imwrite(self.file_name, cv_image)
            cv2.waitKey(1)
            print(f"saved image to {self.file_name}")
            self.transform_as_list = Rt_mat_from_quaternion_44(self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w,self.xpos,self.ypos,self.zpos).tolist()
            self.frame_dict = {"file_path":self.file_name,"transform_matrix":self.transform_as_list}
            self.json["frames"].append(self.frame_dict)
        else:
            print("NO IMAGE")
        self.save_json_to_file()


    def get_odom(self, odom_data):
        self.xpos = odom_data.pose.pose.position.x
        self.ypos = odom_data.pose.pose.position.y
        self.orientation = odom_data.pose.pose.orientation

        self.theta = euler_from_quaternion(self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w)

    def callback(self,image_data):
        print("called")
        print(image_data is not None)
        self.image = image_data

    def save_json_to_file(self):
        with open('/home/jess/ros2_ws/transform.json', 'w') as outfile:
            json.dump(self.json, outfile, indent=4)

def main(args=None):
    rclpy.init(args=args)
    node = data_grabber()
    rclpy.spin(node)
    node.save_json_to_file()
    rclpy.shutdown()