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
import cv2
import matplotlib.pyplot as plt
from .angle_helpers import *
from PIL import Image as img
import io

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

##robot knows where it is
class position_knower(Node):
    def __init__(self):
            super().__init__('position_knower')
            
            self.create_subscription(Odometry, 'odom', self.process_odom, 10)

            self.create_subscription(Image, 'camera/image_raw', self.get_cam, 10)
            self.last_cam = None

            #camera/image_raw
            self.vel_pub = self.create_publisher(Twist,'cmd_vel',10)
            self.create_timer(.05,self.run_loop)

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



    def run_loop(self):
        
        c2w_mat = Rt_mat_from_quaternion(self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w,self.xpos,self.ypos,self.zpos)
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
        image = img.open('/home/jess/ros2_ws/output_image.png')

        # Now you can process the image as needed, e.g., display it, convert it, etc.
        image.show()
        


    def get_cam(self,msg):
        
        self.last_cam = msg
    
    def process_odom(self,msg):

        #PoseWithCovariance.pose.
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y

        #robot_q = msg.pose.pose.orientation
        #w,x,y,z
        #my_quaternion = pyq.Quaternion(robot_q.w,robot_q.x,robot_q.y,robot_q.z)
        #self.orientation = my_quaternion
        self.orientation = msg.pose.pose.orientation

        self.theta = euler_from_quaternion(self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w)


    
    def image_compare(self):
         
         #camera_img = camera()

         NeRF_img = Nerf.forward(self.xpos,self.ypos,self.zpos,THETA,self.cam_phi)

         camera_img, NeRF_img = resize_images(self.last_cam, NeRF_img)


         compare_images()



def main(args=None):
    rclpy.init(args=args)
    node = position_knower()
    rclpy.spin(node)
    rclpy.shutdown()

def quat_angle(q1,q2):
    #angle in degrees between two quaternion objects
    inner_prod =  q1.w *q2.w + q1.x *q2.x + q1.y + q2.y + q1.z * q2.z

    return 180/np.pi * np.arccos(2 * (inner_prod**2) -1)

###
