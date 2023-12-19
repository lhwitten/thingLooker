# thingLooker: A Spot the Difference Application 
## The Goal vs. What We Accomplished
Our goal was to encode a scene of a NeRF at a specific point in time, then capture a live image of a spot in the scene, and then compare the nerf output to the live feed (corresponding to the same camera position in the room) to see if the scene had changed. In other words, we wanted to "spot the difference" in the place the camera captured from when the nerf was collected to the present. 

Specifically, we wanted a Turtlebot to autonomously explore a space and spot the difference at certain locations. 

We managed to do a lot, even if we didn't get it working perfectly. When we figure out how to get the coordinate frames between the nerf forward pass and the live image to be the same, we should be good to go, but unfortunately we didn't figure that out in time for finals because Jess got covid :(. 

Here is what we were able to accomplish, despite the fact that Jess had covid and she had other stuff going on: 

| What we set out to do | What we did |
| ------ | ------ |
| Generate NeRFs with custom data / with phone or Pi | We did that! |
| Simulate a circle packing exploration heuristic | We did that! |
| Creat a script that could run a forward pass through a nerf outside of the nerfstudio scaffolding | We did that! And it was hard, too! |
| Compare live feed to the inference from the forward pass | We did that! |
| Compare live images and nerf output that correspond to the same pose (meaning they are a picture taken from the same place and therefore will be comparable for spot the difference) | We're a little stuck on this part! |
| Create a script that will spot the difference between two images (specifically one from the live feed and one from the forward pass) | We did that! |
| Control a turtle bot | We did that! |
| Use its odometry data to generate nerf output | We did that! |

All in all, we're pretty proud of what we were able to do, especially in the face of several challenges. 

## Setup - Nerfstudio + ROS


# The Pieces
The next sections will describe the code and use-case for each component of the project. We decided to separate them because they stand-alone as projects in addition to contributing to the overarching goal. 

## Generate NeRFs with custom data / with phone or Pi

### Overview

We wanted to be able to generate NeRFs using our own poses. The poses we wanted to generate NeRFs from were an iphone camera pose and the odometry data from the turtle bot. We were able to do both of these in our get_nerf_data script. 

### Steps
get_nerf_data does the following: 

* Acquires odometry data (from the iphone right now, we had implemented the odometry data from the robot functionality but found that it didn't map to the camera's pose directly and therefore it was easier to just use the camera's pose instead. 
* Collecs an image from the phone
* Rotates the image 90 degrees
* Saves the image to a file
* Converts the raw odometry data to the 4x4 transformation matrix that nerfstudio expects their poses to be in
* Writes the camera intrinsics, camera pose and image file path to a json file that corresponds to nerfstudio's desired format

### Usage 

Essentially, the way we used this was we built the node, ran it, and then ran the ARKit app on Jess's iphone to capture images. 
To train NeRFs, its important to capture images with significant overlap and also varied orientations and positions. We usually try to collect about 300 images, 
but reasonable we could collect a lot more (about 2000) without running out of compute to generate even better NeRFs. We would display the images as they were collected to keep track of what was being captured and to verify that we 
were varying camera pose and orientation. We also would open RViz2 to see whether our phone's TF was in the right place, as sometimes it would get de-calibrated and veer far from the origin. Something we still haven't figured out is when
the origin gets designated by the ARKit app. 

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

## Creat a script that could run a forward pass through a nerf outside of the nerfstudio scaffolding

## Compare live feed to the inference from the forward pass 

## Create a script that will spot the difference between two images (specifically one from the live feed and one from the forward pass) 

## Use turtlebots odometry to generate nerf output

## Compare live images and nerf output that correspond to the same pose (meaning they are a picture taken from the same place and therefore will be comparable for spot the difference)


[![N|Solid](https://cldup.com/dTxpPi9lDf.thumb.png)](https://nodesource.com/products/nsolid)

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

Dillinger is a cloud-enabled, mobile-ready, offline-storage compatible,
AngularJS-powered HTML5 Markdown editor.

- Type some Markdown on the left
- See HTML in the right
- ✨Magic ✨

## Features

$$ x = b $$

$ x = b $

$$
x = b

$$

- Import a HTML file and watch it magically convert to Markdown
- Drag and drop images (requires your Dropbox account be linked)
- Import and save files from GitHub, Dropbox, Google Drive and One Drive
- Drag and drop markdown and HTML files into Dillinger
- Export documents as Markdown, HTML and PDF

Markdown is a lightweight markup language based on the formatting conventions
that people naturally use in email.
As [John Gruber] writes on the [Markdown site][df1]

> The overriding design goal for Markdown's
> formatting syntax is to make it as readable
> as possible. The idea is that a
> Markdown-formatted document should be
> publishable as-is, as plain text, without
> looking like it's been marked up with tags
> or formatting instructions.

This text you see here is *actually- written in Markdown! To get a feel
for Markdown's syntax, type some text into the left window and
watch the results in the right.

## Tech

Dillinger uses a number of open source projects to work properly:

- [AngularJS] - HTML enhanced for web apps!
- [Ace Editor] - awesome web-based text editor
- [markdown-it] - Markdown parser done right. Fast and easy to extend.
- [Twitter Bootstrap] - great UI boilerplate for modern web apps
- [node.js] - evented I/O for the backend
- [Express] - fast node.js network app framework [@tjholowaychuk]
- [Gulp] - the streaming build system
- [Breakdance](https://breakdance.github.io/breakdance/) - HTML
to Markdown converter
- [jQuery] - duh

And of course Dillinger itself is open source with a [public repository][dill]
 on GitHub.

## Installation

Dillinger requires [Node.js](https://nodejs.org/) v10+ to run.

Install the dependencies and devDependencies and start the server.

```sh
cd dillinger
npm i
node app
```

For production environments...

```sh
npm install --production
NODE_ENV=production node app
```

## Plugins

Dillinger is currently extended with the following plugins.
Instructions on how to use them in your own application are linked below.

| What we set out to do | What we did |
| ------ | ------ |
| Generate NeRFs with custom data / with phone or Pi | We did that! |
| Simulate a circle packing exploration heuristic | We did that! |
| Creat a script that could run a forward pass through a nerf outside of the nerfstudio scaffolding | We did that! And it was hard, too! |
| Compare live feed to the inference from the forward pass | We did that! |
| Compare live images and nerf output that correspond to the same pose (meaning they are a picture taken from the same place and therefore will be comparable for spot the difference) | We're a little stuck on this part! |
| Create a script that will spot the difference between two images (specifically one from the live feed and one from the forward pass) | We did that! |

## Development

Want to contribute? Great!

Dillinger uses Gulp + Webpack for fast developing.
Make a change in your file and instantaneously see your updates!

Open your favorite Terminal and run these commands.

First Tab:

```sh
node app
```

Second Tab:

```sh
gulp watch
```

(optional) Third:

```sh
karma test
```

#### Building for source

For production release:

```sh
gulp build --prod
```

Generating pre-built zip archives for distribution:

```sh
gulp build dist --prod
```

## Docker

Dillinger is very easy to install and deploy in a Docker container.

By default, the Docker will expose port 8080, so change this within the
Dockerfile if necessary. When ready, simply use the Dockerfile to
build the image.

```sh
cd dillinger
docker build -t <youruser>/dillinger:${package.json.version} .
```

This will create the dillinger image and pull in the necessary dependencies.
Be sure to swap out `${package.json.version}` with the actual
version of Dillinger.

Once done, run the Docker image and map the port to whatever you wish on
your host. In this example, we simply map port 8000 of the host to
port 8080 of the Docker (or whatever port was exposed in the Dockerfile):

```sh
docker run -d -p 8000:8080 --restart=always --cap-add=SYS_ADMIN --name=dillinger <youruser>/dillinger:${package.json.version}
```

> Note: `--capt-add=SYS-ADMIN` is required for PDF rendering.

Verify the deployment by navigating to your server address in
your preferred browser.

```sh
127.0.0.1:8000
```

## License

MIT

**Free Software, Hell Yeah!**

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [dill]: <https://github.com/joemccann/dillinger>
   [git-repo-url]: <https://github.com/joemccann/dillinger.git>
   [john gruber]: <http://daringfireball.net>
   [df1]: <http://daringfireball.net/projects/markdown/>
   [markdown-it]: <https://github.com/markdown-it/markdown-it>
   [Ace Editor]: <http://ace.ajax.org>
   [node.js]: <http://nodejs.org>
   [Twitter Bootstrap]: <http://twitter.github.com/bootstrap/>
   [jQuery]: <http://jquery.com>
   [@tjholowaychuk]: <http://twitter.com/tjholowaychuk>
   [express]: <http://expressjs.com>
   [AngularJS]: <http://angularjs.org>
   [Gulp]: <http://gulpjs.com>

   [PlDb]: <https://github.com/joemccann/dillinger/tree/master/plugins/dropbox/README.md>
   [PlGh]: <https://github.com/joemccann/dillinger/tree/master/plugins/github/README.md>
   [PlGd]: <https://github.com/joemccann/dillinger/tree/master/plugins/googledrive/README.md>
   [PlOd]: <https://github.com/joemccann/dillinger/tree/master/plugins/onedrive/README.md>
   [PlMe]: <https://github.com/joemccann/dillinger/tree/master/plugins/medium/README.md>
   [PlGa]: <https://github.com/RahulHP/dillinger/blob/master/plugins/googleanalytics/README.md>
