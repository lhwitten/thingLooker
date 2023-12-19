# thingLooker: A Spot the Difference Application

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

## Simulate a circle packing exploration heuristic

### Goal

Create a heuristic algorithm which enables a robot to autonomously explore a
known, bounded space more efficiently than conventional methods.

There is a large body of work exploring how to get a robot to autonomously
pathplan to drive over an entire space, this problem is known as coverage.
Because we do not need our robot to stand in every location, as is the case with
lawnmower approaches, but instead to just look at every location, we can be less
meticulous in our movement. Because our Neural Network approach is relatively
costly to produce images, a secondary goal is to create an algorithm which can
view an entire space in a relatively small number of image captures.

### Theory of Efficacy

The initial assumption of this heuristic is that the robot in use is
nonholonomic and utilizes a camera which is effictively one directional with
some viewing angle. There is therefore some penalty in time in completing some
full rotation to look at an entire scene. Note, this approach would still work
with a 360 degree camera though traditional coverage methods may be more
effective in that scenario. One can imagine starting at a point and packing
cones of vision, defined by the angle of the camera and whatever effective
distance your algorithm can make meaningful comparisons at. If one tries to
literally pack consecutive cones then a couple issues arise, namely there are no
algorithms for packing cones in arbitrary shapes and even if there were it would
be incredibly time consuming to move to a certain pose and rotate to an
orientation orientation if two packed cones are not easily reachable from each
other. Circle packing is able to encode the process of packing cones in a
solvable manner while allowing more flexibility for optimal pathing since a
single circle can represent any orientation of a cone.

![Circle and Cone Packing](docs/images/circle_cones_to_pack.jpg)

The circles used are inscribed circles in the vision cones. It is possible to
exactly find the radius of the inscribed circle using the angle of the vision
cone and the radius using geometry.

![Circle and Cone Packing](docs/images/circle_together.jpg)

Packing circles within a space and then iterating through, viewing each circle
in order ensures that the majority of a space is viewed by the robot.
Additionally, because the robot must turn and move to get to the proper viewing
distance and orientation to view a circle, more of the space than prescribed can
be viewed and analyzed.

### Implementation

The steps to implement this algorithm include describing the space to explore
geometrically using a series of vertices as well as inputting the necessary size
circle to pack (based on the viewing angle and distance). The algorithm will
them pack circles within the closed polygon and output a list of circle centers.
The traveling salesman problem is then solved on these points to find a shortest
path route between the waypoints starting at the robot start location. The
final, ordered list of points is then iterated through with the robot PID
navigating to an orientation and distance which view the circle center. When the
robot is sufficiently close and oriented towards a waypoint, the next waypoint
is chosen and navigated towards. The robot may either perform its computer
vision tasks continuously while navigating or only while directly facing a
waypoint.

The circle packer works by a smaller area within the larger polygon where
placing a circle will always be allowable. Once this area is found, the
algorithm will start in a corner and place a circle. The algorithm will then use
depth first search to place circles 2 radii away from the base circle and will
continue until no more valid circles may be placed within a space. This approach
works on both convex and concave polygons. The paper on this circle packing
technique uses depth first search but we found that using breadth first search
resulted in better circle packings. DFS is on the left while BFS is on the
right.

![Circle and Cone Packing](docs/images/dfs_vs_bfs.png)

A path created by our pathplanning algorithm.

![Circle and Cone Packing](docs/images/final_path.png)

## Creat a script that could run a forward pass through a NeRF outside of the nerfstudio scaffolding

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

    - input NeRF and camera image
    - resize both images
    - replace blurry regions
    - `compare_images()`
      - Convert Images to Grayscale
      - Threshold Difference regions
      - visualize differences

  ### Usage


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
