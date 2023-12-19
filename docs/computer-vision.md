# thingLooker: A Spot the Difference Application

- [Home](index.md): Overview
- [Path Planning](path-planning.md): Novel Exploration Heuristic
- [Computer Vision](computer-vision.md): NeRF implementation and image
  comparison details
- [Milestones and Ethics Statement](milestones.md):

## The Goal vs. What We Accomplished

Our goal was to encode a scene of a NeRF at a specific point in time, then
capture a live image of a spot in the scene, and then compare the nerf output to
the live feed (corresponding to the same camera position in the room) to see if
the scene had changed. In other words, we wanted to "spot the difference" in the
place the camera captured from when the nerf was collected to the present.

Specifically, we wanted a Turtlebot to autonomously explore a space and spot the
difference at certain locations.

We managed to do a lot, even if we didn't get it working perfectly. When we
figure out how to get the coordinate frames between the nerf forward pass and
the live image to be the same, we should be good to go, but unfortunately we
didn't figure that out in time for finals because Jess got covid :(.

Here is what we were able to accomplish, despite the fact that Jess had covid
and she had other stuff going on:

| What we set out to do                                                                                                                                                                | What we did                        |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ---------------------------------- |
| Generate NeRFs with custom data / with phone or Pi                                                                                                                                   | We did that!                       |
| Simulate a circle packing exploration heuristic                                                                                                                                      | We did that!                       |
| Creat a script that could run a forward pass through a nerf outside of the nerfstudio scaffolding                                                                                    | We did that! And it was hard, too! |
| Compare live feed to the inference from the forward pass                                                                                                                             | We did that!                       |
| Compare live images and nerf output that correspond to the same pose (meaning they are a picture taken from the same place and therefore will be comparable for spot the difference) | We're a little stuck on this part! |
| Create a script that will spot the difference between two images (specifically one from the live feed and one from the forward pass)                                                 | We did that!                       |
| Control a turtle bot                                                                                                                                                                 | We did that!                       |
| Use its odometry data to generate nerf output                                                                                                                                        | We did that!                       |

All in all, we're pretty proud of what we were able to do, especially in the
face of several challenges.

## Setup - Nerfstudio + ROS

Our architecture consists of two repos because we blend two very different
software tools: nerfstudio and ROS. Nerfstudio runs in a conda environment while
ROS runs at the system level. Ros involves packages and buidling and sourcing
and a system python version whereas nerfstudio does not. Therefore, we had to be
clever about integrating the two technologies so we could take the data recieved
from ROS and use it to get nerf output, which would then be used by the ROS node
again for further use. Specifically, we had to run a subprocess in our main
**explore.py** script to run a bash script that would activate the nerfstudio
conda environment and then run the load_model script to do the forward pass
through the NeRF Network. We had to do this because ROS does not play nice with
conda environments.

## Code Architecture

Our code architecture consists of the following scripts:

- **explore.py** is our main script, that does the nerf comparison.
- **explore.py** calls **angle_helpers.py** to turn the odometry/pose data it
  recieves into a format that the NeRF can take as input. The NeRF uses a pose
  as input to generate the corresponding image in the encoded scene.
- The format it takes as input is a 3x4 transformation matrix:

```python
# [+X0 +Y0 +Z0 X]
# [+X1 +Y1 +Z1 Y]
# [+X2 +Y2 +Z2 Z]
# [0.0 0.0 0.0 1] (this row is assumed in the forward pass)
```

- A prerequisite to running explore is acquiring data to train the NeRF. We use
  the script **get_nerf_data.py** to get the **transforms.json** file used to
  train the NeRF.
- We then use nerfstudio's ns-train CL command pointing to the
  **transforms.json** file to train the NeRF:

```
conda activate nerfstudio3
ns-train nerfacto --data /path/to/transforms.json
```

- In **explore.py**, we run another script, **load_model.py**, as a subprocess.
  The script runs a forward pass through the NeRF and produces a 2D RGBA image
  as output. The RGBA image is written to memory.
- To run **load_model.py**, first the subprocess runs **run_load_model.sh**
  because the nerfstudio conda environment has to be activated before the NeRF
  model can be loaded. But, the nerfstudio conda environment cannot be active
  while the ROS node is run at first, hence the need for the subprocess approach
  that will close the environment upon termination.
- Then, that rgba image is loaded by **explore.py** and then image comparison is
  done using functions housed in the **compare_images.py** script.
- To get data from the iPhone we use as our camera, we used
  **arkit_data_streamer**, an external repository. To activate the stream, we
  had to open the app on the iPhone and start streaming the data. Both the
  iPhone and the laptop running the **explore.py** script have to be on the same
  WiFi network, specifically the Olin Robotics WiFi network. We also had to
  launch the arkit server to stream data from the phone to ROS topics that the
  computer could subscribe to:

```shell
(base) jess@jess-Precision-5770:~/ros2_ws$ ros2 launch arkit_data_streamer launch_data_servers.py
```

So, all in all, this project requires 3 repositories to run:

- A modified version of **nerfstudio** (with the proprietary **load_model.py**
  script added to the models directory)
- The **arkit_data_streamer** repository, housed in a ros2_ws directory. This
  manages the pose and camera data recieved from the iPhone
- The **thingLooker** repository, housed in the ros2_ws directory as well. This
  has all of the code related to converting the data into the right formats,
  generating nerf data, generating output from the nerf, and finally, doing the
  comparison between the live feed and the corresponding encoded scene image
  from the NeRF.

and 6 scripts:

- **explore.py** (ros2ws/thingLooker)
- **load_model.py** (nerfstudio/models)
- **run_load_model.sh** (ros2ws/thingLooker)
- **compare_images.py** (ros2ws/thingLooker)
- **get_nerf_data.py** (ros2ws/thingLooker)
- **angle_helpers.py** (ros2ws/thingLooker)

and this doesn't include the autonomous exploration code.

# The Pieces

The next sections will describe the code and use-case for each component of the
project. We decided to separate them because they stand-alone as projects in
addition to contributing to the overarching goal.

## Generate NeRFs with custom data / with phone or Pi

### Overview

We wanted to be able to generate NeRFs using our own poses. The poses we wanted
to generate NeRFs from were an iphone camera pose and the odometry data from the
turtle bot. We were able to do both of these in our get_nerf_data script.

### Steps

get_nerf_data does the following:

- Acquires odometry data (from the iphone right now, we had implemented the
  odometry data from the robot functionality but found that it didn't map to the
  camera's pose directly and therefore it was easier to just use the camera's
  pose instead.
- Collecs an image from the phone
- Rotates the image 90 degrees
- Saves the image to a file
- Converts the raw odometry data to the 4x4 transformation matrix that
  nerfstudio expects their poses to be in
- Writes the camera intrinsics, camera pose and image file path to a json file
  that corresponds to nerfstudio's desired format

### Usage

Essentially, the way we used this was we built the node, ran it, and then ran
the ARKit app on Jess's iphone to capture images. To train NeRFs, its important
to capture images with significant overlap and also varied orientations and
positions. We usually try to collect about 300 images, but reasonable we could
collect a lot more (about 2000) without running out of compute to generate even
better NeRFs. We would display the images as they were collected to keep track
of what was being captured and to verify that we were varying camera pose and
orientation. We also would open RViz2 to see whether our phone's TF was in the
right place, as sometimes it would get de-calibrated and veer far from the
origin. Something we still haven't figured out is when the origin gets
designated by the ARKit app.

### Implementation

Here is the code itself:

```python
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
        self.image = image_data  # Updates current image with incoming data

    def save_json_to_file(self):
        # Saves JSON data to a file
        with open('/home/jess/ros2_ws/transforms.json', 'w') as outfile:
            json.dump(self.json, outfile, indent=4)

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2 client library
    node = data_grabber()  # Create a data_grabber node
    rclpy.spin(node)  # Keep the node running
    node.save_json_to_file()  # Save JSON data before shutdown
    rclpy.shutdown()  # Shutdown ROS2 client library

```

### Output

Here is what the json file looks like:

```json
{
    "camera_model": "OPENCV",
    "fl_x": 721.498,
    "fl_y": 721.498,
    "cx": 512.8072,
    "cy":383.15,
    "w": 865,
    "h":  1153,
    "k1": -0.0329,
    "k2": 0.058,
    "p1": 0.000255,
    "p2": 0.0,
    "aabb_scale": 16,
    "frames": [
        {
            "file_path": "/home/jess/ros2_ws/image_data/img0.png",
            "transform_matrix": [
                [
                    0.4023027214248577,
                    0.03935649539660654,
                    0.9146603668052149,
                    -0.124712
                ],
                [
                    -0.009478503855119141,
                    0.99920106377164,
                    -0.03882514806528731,
                    0.148908
                ],
                [
                    -0.9154576332626332,
                    0.006949850913500215,
                    0.4023543478992843,
                    0.290508
                ],
                [
                    0.0,
                    0.0,
                    0.0,
                    1.0
                ]
            ]
        },
```

Here is what the image directory looks like:

## Create a Script that Could Run a Forward Pass through a NeRF Outside of the Nerfstudio Scaffolding

### Overview

We wrote a script that given a pose and a nerf config file, would run a forward
pass through the NeRF. There was no infrastructure to do this easily in
NeRFStudio so we created it outselves by loading our config as a model instance
and then performing inference using the pose as input. Also something to note is
that load_model.py, the script that performs the described functionality, lives
inside the nerfstudio repo to make imports easier, not the thingLooker repo. The
script is run as a subprocess in our explore.py file, which actually does our
image comparison. Explore is our main script. Load_model can be found
[here](https://github.com/jes-bro/nerfstudio), in our nerfstudio repo.

### Steps

- Parse the CL arguments to get transform
- Convert transform to right format for processing
- Call get_rgb_from_pose_transform() with config pointing to trained NeRF and
  pose
- Generate camera object and camera ray bundle corresponding to pose
- Pass that into NeRF for forward pass
- Call model.get_rgba_image() to extract the image corresponding to the pose
- Display the figure for verification
- Save the image to a file

### Usage

This script is used to run a forward pass through the nerf. The function can run
standalone as well. When we were testing it initially, rather than parsing CL
input we just provided the input at the bottom of the script and called the
get_rgba_from_pose_transform() with inputs. In our explore context, the function
is called when we need a NeRF output that corresponds to the pose recieved from
the iPhone through the arkit repo.

### Implementation

Here is the load_model.py implementation:

```python
import numpy as np
import sys
import torch
import json
from os import chdir
from nerfstudio.utils.eval_utils import eval_setup
from pathlib import Path
from nerfstudio.cameras import cameras
import matplotlib.pyplot as plt

def get_rgb_from_pose_transform(load_config, camera_to_world, fx= torch.Tensor([1400.]), fy=torch.Tensor([1400.]), cx=torch.Tensor([960.]), cy=torch.Tensor([960.])):
    # Specify that forward pass should be on GPU
    device = torch.device("cuda")
    # Make sure c2w matrix is stored on GPU
    camera_to_world = camera_to_world.to(device)
    # Create a path object out of the config so eval_setup can use it as an input
    load_config_path = Path(load_config)
    # Get pipeline frome eval_setup. We get pipeline so we can extract the model
    config, pipeline, checkpoint_path, step = eval_setup(load_config_path)
    # Extract the model from the pipeline
    model = pipeline.model
    # Specify that the forward pass is from one cameras worth of pose data (I don't even know what it would mean to have more)
    camera_indices = torch.tensor([0], dtype=torch.int32)
    # Port that tensor to the GPU
    camera_indices = camera_indices.to(device)
    # Create a cameras object
    camera = cameras.Cameras(camera_to_world, fx, fy, cx, cy)
    # Generate rays from the cameras object- this is the actual input to the NeRF model
    rays = camera.generate_rays(camera_indices)
    # Pass rays into model to get NeRF output
    outputs = model.get_outputs_for_camera_ray_bundle(rays)
    # Get the rgba image from the NeRF output
    rgba_image = model.get_rgba_image(outputs)
    # Return it
    return rgba_image

if __name__ == '__main__':
    chdir("/home/jess")
    test_path = "/home/jess/outputs/poly_data/nerfacto/2023-12-16_041906" #"/home/jess/outputs/home/jess/outputs/new_stuff/nerfacto/2023-12-16_024242/config.yml"
    # Parse the CL arguments to extract the camera to world pose transform
    string_c2w = sys.argv[1]
    # Load with json (as Python list)
    list_c2w = json.loads(string_c2w)
    # Convert to numpy array
    nparray_c2w = np.array([list_c2w])
    # Convert to tensor
    tensor_c2w = torch.Tensor([nparray_c2w]).squeeze()
    # Get RGBA image
    rgba_image = get_rgb_from_pose_transform(test_path, tensor_c2w, torch.Tensor([1400.]), torch.Tensor([1400.]), torch.Tensor([960.]), torch.Tensor([960.]))
    # Port it to CPU
    rgba_image_cpu = rgba_image.cpu()
    # Convert to numy arrray again
    rgba_numpy = rgba_image_cpu.numpy()
    # Show the image to verify it came through
    plt.imshow(rgba_image_cpu)
    # Save the image to a file
    plt.savefig('/home/jess/ros2_ws/output_image.png', bbox_inches='tight', pad_inches=0)
    # Turn off axes so they don't show up in the picture
    plt.axis('off')
    # Close the plot cleanly
    plt.close()
```

### Output

Here is sample output from the forward pass:

## Compare Live Feed to the Inference from the Forward Pass

### Overview

The explore.py script generates an RGBA image from the NeRF that should
correspond to the current pose from the live feed. Right now the pose doesn't
correspond because of an issue with coordinate frame misalignment, but the image
is generated from the nerf and looks decent, ie. right orientation, scene,
resolution, etc. just in the wrong spot. The script does succeed in retrieving
the NeRF output image, retrieving the live feed, and calling the required
scripts to generate a comparison between those things.

### Steps

- Use callbacks to get live image from ROS and the live camera pose
- Convert the ROS image to an OpenCV image
- Generate camera to world transform
- Call image compare function
- Get NeRF Image
- Call function that compares live feed to NeRF image

### Usage

We run this script to do our actual image comparison. It puts all of the pieces
together. Once the NeRF output and the live image are in the same coordinate
frame, with the same origin, it should work.

### Implementation

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
import json
import numpy as np
import subprocess
import cv2
from cv_bridge import CvBridge, CvBridgeError
from .angle_helpers import *
import io
from .compare_images import *
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage

##robot knows where it is
class position_knower(Node):
    def __init__(self):
            super().__init__('position_knower')
            self.bridge = CvBridge()
            self.shutdown_flag = False
            # This is the subscriber for the iPhone camera feed
            self.create_subscription(CompressedImage, 'camera/image_raw/compressed',self.callback,10)
            # This is the pose of the iPhone camera
            self.create_subscription(PoseStamped, 'device_pose', self.get_odom, 10)
            self.last_cam = None
            self.image = None
            self.xpos = 0.0
            self.ypos = 0.0
            self.zpos = 0.0
            self.theta = 0.0
            self.cam_phi = np.pi/16
            self.xpos_b = 0.0
            self.ypos_b = 0.0
            self.orientation = Quaternion()
            self.wait = False
            self.w_count = 0
            self.target_size = None
            self.counter = 0
            self.json = {"camera_model":"OPENCV","fl_x":506.65,"fl_y":507.138,"cx":383.64,"cy":212.61,"w":768,"h":432,"k1":-0.0329,"k2":0.058,"p1":0.000255,"p2":0.0,"aabb_scale":16,"frames":[]}
            self.timer = self.create_timer(2.0, self.run_loop)

    def run_loop(self):
        if self.image is not None:
            try:
                # Convert the ROS image to an OpenCV image
                cv_image = self.bridge.compressed_imgmsg_to_cv2(self.image,"passthrough")
                print(cv_image)
                cv2.imshow("window",cv_image)
                cv2.waitKey(1)
            except CvBridgeError as e:
                print(e)
            # Generate camera to world transform
            c2w_mat = Rt_mat_from_quaternion(self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w,self.xpos,self.ypos,self.zpos)
            self.image_compare(c2w_mat)
            # Save the image
        else:
            print("NO IMAGE")


    def get_odom(self, odom_data):
        # Retrieve odom data/pose data from phone
        self.xpos = odom_data.pose.position.x
        self.ypos = odom_data.pose.position.y
        self.zpos = odom_data.pose.position.z
        self.orientation = odom_data.pose.orientation
        self.theta = euler_from_quaternion(self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w)

    def callback(self,image_data):
        # Retrieve live feed from iPhone
        self.image = image_data

    def save_json_to_file(self):
        # Save the information in the json dictionary to an actual json file
        with open('/home/jess/ros2_ws/transforms.json', 'w') as outfile:
            json.dump(self.json, outfile, indent=4)

    def image_compare(self,c2w_mat):
        print(f"image in compare? {self.image is True}")
        print("NeRF image being collected")
        NeRF_img =  self.get_nerf_pic(c2w_mat)
        print("NeRF image collected")
        size_Nerf = get_image_size(NeRF_img)

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
        camera_img = resize_image(cv_image, target_size)

        # Call function from compare_images
        compare_and_visualize_differences(NeRF_img,camera_img)

    # Start the subprocess and retrieve the image from the nerf that
    # corresponds to the pose
    def get_nerf_pic(self, c2w_mat):
        list_c2w_mat = c2w_mat.tolist()
        string_c2w = json.dumps(list_c2w_mat)
        process = subprocess.Popen(['/home/jess/ros2_ws/run_load_model.sh', string_c2w],
                           stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE)

        process.communicate()
        image = cv2.imread('/home/jess/ros2_ws/output_image1.png')
        return image

def main(args=None):
    rclpy.init(args=args)
    node = position_knower()
    rclpy.spin(node)
    node.save_json_to_file()
    rclpy.shutdown()

```

## Create a script that will spot the difference between two images (specifically one from the live feed and one from the forward pass)

### Overview

We compared images from two sources, test sources included webcams, Pi cameras,
and NeRF output. We changed the format of all images into cv2 images and then
resized them so they could be compared against each other. We used two image
comparison approaches, one with simple SSIM comparison which directly compares
pixel regions on an image and another which uses sentence transformers to encode
and then robustly compare two images together. It became apparent that image
preprocessing was necessary, first to remove the blur from ill defined regions
of NeRF output using laplacians and then to threshold similarity in the SSIM
approach, ensuring that we do not detect regions of low difference. We decided
to use the SSIM approach to visualize the differences between NeRFs and camera
images while we found the sentence transformer approach to be better for
detecting the differences themselves as it is more robust to rotation and
shifting. The script we use to do this is **compare_images.py**.

### Steps

    * input NeRF and camera image
    * resize both images
    * replace blurry regions
    * `compare_images()`
    * Convert Images to Grayscale
    * Threshold Difference regions
    * Visualize differences

### Implementation

Here is the implementation:

```python
from skimage.metrics import structural_similarity
import cv2
import numpy as np
from sentence_transformers import SentenceTransformer, util
from PIL import Image
import glob
import os
import matplotlib.pyplot as plt

import skimage


def image_compare_SSIM(img1,img2):
    """
    A Basic Image comparison approach which directly compares images to find regions of difference.
    Thresholding is implemented so that difference finding is adjustable. Not robust to small turns.
    """

    # Convert images to grayscale
    first_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    second_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    # Compute SSIM between two images
    score, diff = structural_similarity(first_gray, second_gray, full=True)
    print("Similarity Score: {:.3f}%".format(score * 100))

    # The diff image contains the actual image differences between the two images
    # and is represented as a floating point data type so we must convert the array
    # to 8-bit unsigned integers in the range [0,255] before we can use it with OpenCV
    diff = (diff * 255).astype("uint8")

    # Threshold the difference image, followed by finding contours to
    # obtain the regions that differ between the two images
    thresh = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    # Highlight differences
    mask = np.zeros(img1.shape, dtype='uint8')
    filled = img2.copy()

    for c in contours:
        area = cv2.contourArea(c)
        if area > 100:

            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(img1, (x, y), (x + w, y + h), (36,255,12), 2)
            cv2.rectangle(img2, (x, y), (x + w, y + h), (36,255,12), 2)
            cv2.drawContours(mask, [c], 0, (0,255,0), -1)
            cv2.drawContours(filled, [c], 0, (0,255,0), -1)

    cv2.imshow('first', img1)
    cv2.imshow('second', img2)
    cv2.imshow('diff', diff)
    cv2.imshow('mask', mask)
    cv2.imshow('filled', filled)
    cv2.waitKey()

def compare_images_DenseV(img1,img2):
    """
    Another image comparison implementation. This implementation uses sentence transformers to
    compare images and does not produce visual output but is more robust to small turns.
    """

    img1 = Image.fromarray(cv2.cvtColor(img1, cv2.COLOR_BGR2RGB))
    img2 = Image.fromarray(cv2.cvtColor(img2, cv2.COLOR_BGR2RGB))
    # Load the OpenAI CLIP Model
    print('Loading CLIP Model...')
    model = SentenceTransformer('clip-ViT-B-32')

    # Next we compute the embeddings
    encoded_images = model.encode([img1,img2], batch_size=128, convert_to_tensor=True, show_progress_bar=True)

    # Now we run the clustering algorithm. This function compares images aganist
    # all other images and returns a list with the pairs that have the highest
    # cosine similarity score
    processed_images = util.paraphrase_mining_embeddings(encoded_images)
    print('Finding duplicate images...')
    # Filter list for duplicates. Results are triplets (score, image_id1, image_id2) and is scorted in decreasing order
    # A duplicate image will have a score of 1.00
    # It may be 0.9999 due to lossy image compression (.jpg)
    score = processed_images[0][0]

    # Output the top X duplicate images
    #score, image_id1, image_id2 = duplicates[0]
    print("\nScore: {:.3f}%".format(score * 100))


def get_camera():
    """
    A script that tests image comparison by using the laptop webcam.
    """

    # define a video capture object
    vid = cv2.VideoCapture(0)

    ret, frame = vid.read()
    print("Got frame")
    # Display the resulting frame
    #cv2.imshow('frame', frame)

    while(True):

        # Capture the video frame
        # by frame
        ret, frame = vid.read()

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        if key == ord('c'):
            frame1 = frame
            print("frame1 set")

        if key == ord('v'):
            frame2 = frame
            print("frame2 set")

        if key == ord('b'):
            compare_and_visualize_differences(frame1,frame2)

    # After the loop release the cap objectq
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

def compare_and_visualize_differences(img1, img2, min_contour_area=100):
    """
    A Basic Image comparison approach which directly compares images to find regions of difference.
    Thresholding is implemented so that difference finding is adjustable. Not robust to small turns.

    A version of SSIM similarity comparison with a thresholding on the minimum size of difference to detect.
    """

    #img1,img2 = replace_blurry_regions(img1,img2,blur_threshold=65)

    # Convert images to grayscale
    first_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    second_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    # Compute SSIM between two images
    score, diff = structural_similarity(first_gray, second_gray, full=True)
    print("Similarity Score: {:.3f}%".format(score * 100))

    # Normalize the difference image to the range [0,255] and convert to uint8
    diff = (diff * 255).astype("uint8")

    # Threshold the difference image, followed by finding contours to obtain the regions of difference
    thresh = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize mask to highlight differences
    mask = np.zeros(img1.shape, dtype='uint8')
    filled = img2.copy()

    # Loop over the contours
    for c in contours:
        area = cv2.contourArea(c)
        # Only consider coUpdate README.mdntours with area greater than the specified threshold
        if area > min_contour_area:
            x, y, w, h = cv2.boundingRect(c)
            # Draw rectangles around differences
            cv2.rectangle(img1, (x, y), (x + w, y + h), (36, 255, 12), 2)
            cv2.rectangle(img2, (x, y), (x + w, y + h), (36, 255, 12), 2)
            # Fill in the mask and filled images with contours
            cv2.drawContours(mask, [c], 0, (0, 255, 0), -1)
            cv2.drawContours(filled, [c], 0, (0, 255, 0), -1)

    # Display the images
    print("DISPLAYING IMAGES")
    cv2.imshow('first', img1)
    cv2.imshow('second', img2)
    cv2.imshow('diff', diff)
    cv2.imshow('mask', mask)
    cv2.imshow('filled', filled)
    cv2.waitKey(0)
    #cv2.destroyAllWindows()

#blur functions

def detect_blurry_regions(image, threshold=100):
    """
    Detect blurry regions in an image using the Laplacian method.
    Returns a mask indicating blurry regions.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    laplacian = cv2.Laplacian(gray, cv2.CV_64F)
    laplacian_var = laplacian.var()

    # If variance is less than the threshold, it's considered blurry
    mask = np.where(laplacian_var < threshold, 0, 1).astype('uint8')


    visualize = True
    if visualize:
        plt.figure(figsize=(10, 4))
        plt.subplot(1, 2, 1)
        plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        plt.title('Original Image')
        plt.axis('off')
        plt.subplot(1, 2, 2)
        plt.imshow(laplacian, cmap='gray')
        plt.title('Laplacian Gradient')
        plt.axis('off')
        plt.show()

    return mask

def replace_blurry_regions(img1, img2, blur_threshold=100):
    """
    Detects blurry regions in both images and replaces those regions with black pixels in both images.
    Deals with ill defined regions of a NeRF.
    """
    # Detect blurry regions in both images
    mask1 = detect_blurry_regions(img1, blur_threshold)
    mask2 = detect_blurry_regions(img2, blur_threshold)

    # Combine masks to identify all blurry regions in both images
    combined_mask = np.bitwise_or(mask1, mask2)

    # Apply combined mask to both images
    img1[combined_mask == 0] = [0, 0, 0]
    img2[combined_mask == 0] = [0, 0, 0]

    return img1, img2

def get_image_size(img):
    """
    Return the size (width, height) of the image at the given path.

    :param image_path: Path to the image.
    :return: A tuple (width, height).
    """
    #img = cv2.imread(image_path)
    if isinstance(img, np.ndarray):
        return img.shape[1], img.shape[0]
    else:
        return img.size[1], img.size[0]  # Width, Height

def resize_image(img, size):
    """
    Resize an image to the given size and save it to the output path.

    :param input_path: Path to the input image.
    :param output_path: Path to save the resized image.
    :param size: A tuple (width, height) for the target size.
    """
    resized_img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)

    return resized_img
```

## Manipulate the Odometry / Pose Data so it is Interpretable by the NeRF

### Overview

Odometry data is sent by the turtlebot as a quaternion orientation and an x,y,z
coordinate - bundled together as a pose. NeRF forward pass input and training
protocol require that this orientation and position data be formatted in 4x4 or
3x4 matrix containing the combined rotation matrix using euler angles from 3
axes and a column vector containing the translational information. The script we
use to do this is **angle_helpers.py**.

### Steps

- Receive odometry
- update the last recorded pose
- Convert Quaternion to euler angles
- construct roll, pitch, and yaw rotation matrices from euler angles
- multiply rotation matrices together
- append the translation column vector to the right side of the matrix
- Optional: apply camera transformation
- return the camera to world transformation matrix

### Usage

This camera to world matrix is used in conjunction with an image as input a NeRF
for training as well as by itself as input to a NeRF's forward pass.

### Implementation

Here is the angle_helpers implementation:

```python
import math
import numpy as np

def euler_from_quaternion(x, y, z, w):
    """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
    """
    # Calculate roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    # Calculate pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    # Calculate yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # Returns the Euler angles in radians

def Rt_mat_from_quaternion(x,y,z,w,xpos,ypos,zpos):
    """
    Create a Camera to World matrix using an input quaternion orientation and x,y,z pose.

    Output is in [R |T] format with the translation parameters in a right side 3x1 column while
    the combined rotation matrix is a 3x3 matrix on the left.
    """

    roll, pitch, yaw = euler_from_quaternion(x, y, z, w)

    # Compute rotation matrices for each Euler angle
    Rz_yaw = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw),  math.cos(yaw), 0],
        [0,             0,             1]
    ])

    Ry_pitch = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0,              1,              0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    Rx_roll = np.array([
        [1, 0,               0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll),  math.cos(roll)]
    ])

    # The rotation matrix is the product of individual rotation matrices
    # Combine the rotation matrices and add the translation vector
    R = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
    R = np.hstack([R, np.array([[xpos], [ypos], [zpos]])])

    return R

def Rt_mat_from_quaternion_44(x,y,z,w,xpos,ypos,zpos):
    """Create a Camera to World matrix using an input quaternion orientation and x,y,z pose.

    Output is in [R |T] format with the translation parameters in a right side 3x1 column while
    the combined rotation matrix is a 3x3 matrix on the left. The final output is multiplied by
    a camera transform.
    """

    roll, pitch, yaw = euler_from_quaternion(x, y, z, w)
    # Compute rotation matrices for each Euler angle
    # Same as in Rt_mat_from_quaternion function
    Rz_yaw = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw),  math.cos(yaw), 0],
        [0,             0,             1]
    ])

    Ry_pitch = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0,              1,              0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    Rx_roll = np.array([
        [1, 0,               0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll),  math.cos(roll)]
    ])

    R = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
    R = np.hstack([R, np.array([[xpos], [ypos], [zpos]])])

    R = np.vstack([R, np.array([0, 0, 0, 1])])

    return R




def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # Compute quaternion components
    q = [0] * 4
    q[0] = cy * cp * sr - sy * sp * cr
    q[1] = sy * cp * sr + cy * sp * cr
    q[2] = sy * cp * cr - cy * sp * sr
    q[3] = cy * cp * cr + sy * sp * sr

    return q
```

Odometry data is sent by the iPhone as a quaternion orientation and an x,y,z
coordinate - bundled together as a pose. NeRF forward pass input and training
protocol require that this orientation and position data be formatted in 4x4 or
3x4 axes and a column vector containing the translational information.

This camera to world matrix is used in conjunction with an image as input a NeRF
for training as well as by itself as input to a NeRF's forward pass.
