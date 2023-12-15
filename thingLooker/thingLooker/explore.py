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
from .compare_images import *
from .path_plan import *

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

            self.cam_phi = np.pi/16 #orientation of the camera
            #self.angular = 0.0

            #position of last turn
            self.xpos_b = 0.0
            self.ypos_b = 0.0
            self.orientation_bench = Quaternion()

            self.orientation = Quaternion()
            self.wait = False
            self.w_count = 0

            #whether to use pathplanning or not
            self.path_plan = True

            #path planning parameters
            self.goal_distance = 1 # from geometry, where we want to watch the 
            self.camera_angle = np.pi/8 #radian angle of viewing from the camera

            self.boundary_vertices = np.array([
                [0, 0],
                [1, 8],
                [5, 10],
                [4, 0],
                [0, 0]])
            self.waypoints_ordered = get_waypoint_list(self.boundary_vertices,.4)
            self.waypoints_ordered.pop(0) #get rid of points we're standing at







    def run_loop(self):
        
        c2w_mat = Rt_mat_from_quaternion(self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w,self.xpos,self.ypos,self.zpos)
        #nerf_pic = self.get_nerf_pic(c2w_mat)
        self.image_compare(c2w_mat)

        if (self.path_plan and self.waypoints_ordered):
            #velpub happens here as well
            self.pid_to_waypoint()
        else:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
            #send basically nothing

    
    def pid_to_waypoint(self):

        goal_distance = self.goal_distance

        waypoint = self.waypoints_ordered[0]

        msg = Twist()

        distance, angle = self.pythag(waypoint[0],waypoint[1])
        msg.linear.x = .1*(distance - goal_distance)

        if angle:
            msg.angular.z = .2*angle
        else:
            msg.angular.z = .1

        self.vel_pub.publish(msg)

        angle_threshold = np.pi/32
        if ( (np.abs(distance) < goal_distance*1.1 ) and ( np.abs(angle) < angle_threshold )):
            self.waypoints_ordered.pop(0)
            
            #do image comparison here
            c2w_mat = Rt_mat_from_quaternion(self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w,self.xpos,self.ypos,self.zpos)
            #nerf_pic = self.get_nerf_pic(c2w_mat)
            self.image_compare(c2w_mat)

            #image comparison over

            if self.waypoints_ordered:
                print("waypoint reached, new waypoint added")
            else:
                print("No more waypoints, quitting")


            


    def pythag(self,goal_p_x,goal_p_y):

        distance = math.sqrt( (self.xpos - goal_p_x)**2 +  (self.ypos - goal_p_y)**2)

        #angular_distance = abs(self.ang_bench - self.angular)

        roll_x, pitch_y, yaw_z= euler_from_quaternion(self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w)

        angle_to_waypoint = math.atan2(goal_p_y - self.ypos, goal_p_x - self.xpos)

        angular_distance = angle_to_waypoint - yaw_z


        return distance,angular_distance

    def get_cam(self,msg):
        
        self.last_cam = msg

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


    
    def image_compare(self,c2w_mat):
         
         #camera_img = camera()
        print("NeRF image being collected")
        NeRF_img =  self.get_nerf_pic(c2w_mat)
        print("NeRF image collected")
        size_Nerf = get_image_size(NeRF_img)

        vid = cv2.VideoCapture(0)
        ret, frame = vid.read()     
        print("Got frame")
        #size_cam = get_image_size(self.last_cam)

        print(frame)
        print(type(frame))
        size_cam = get_image_size(frame)

        # Determine the smaller size
        target_size = min(size_Nerf, size_cam, key=lambda x: x[0] * x[1])

        # Resize first image
        NeRF_img = resize_image(NeRF_img, target_size)

        
        # Resize second image
        #camera_img = resize_image(self.last_cam, target_size)
        camera_img = resize_image(frame, target_size)


        compare_and_visualize_differences(NeRF_img,camera_img)



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
