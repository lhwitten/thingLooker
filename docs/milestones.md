# Milestones and Ethics Statement

## Navigation

- [Home](index.md): Overview
- [Path Planning](path-planning.md): Novel Exploration Heuristic
- [Computer Vision](computer-vision.md): NeRF implementation and image
  comparison details
- [Milestones and Ethics Statement](milestones.md):

## Ethics Statement

NeRF spot the difference is a computer vision project which leverages neural
radiance fields and autonomous robot path planning to explore and analyze a
space. Our hope is that because of constraints we set on the problem, it would
be incredibly inefficient for our work to be used in contexts of warfare or
espionage. The requirement of the path planning heuristic to be exploring an
already known space and the requirement of the NeRF to be pre trained on many
images of an already explored space particularly limit the scope of application
for this work.

The most nefarious direct use case of our work would likely be search and
destroy operations. It could be possible for a military unit to equip a robot
with a weapon and then use our algorithm to spot differences in a space, finding
people within a building or complex. There are privacy concerns as well
regarding NeRFs. Creating a full 3D representation of a space without the
owner’s consent is entirely possible using a NeRF and the framework we created
for this project could allow for NeRF data to be collected autonomously. We
believe that these two concerns are the primary ethical shortfalls of this
project as our work seemingly cannot be used to affect people economically,
environmentally, or through the propagation of bias.

To alleviate possible unethical uses of this technology we do not license our
work to be used by military organizations or on any robots which carry weapons
or other methods of harm. Additionally we require the consent of the owner of a
space to train a NeRF on collected images, though this not currently
enforceable.

## Milestone 1

- **Milestone 1 Goals and Accomplishments:**

  - **Goal:** We will take many overlapping pictures of the comprobo classroom.
    We will do this by mounting a phone to the neato and manually taking many
    many pictures of the room from the Neato's perspective.

    - **Task Achieved:** We have fully finished data collection of the Olin ART
      lab. This is a more reasonably sized space than the Comprobo room and will
      allow us to more quickly collect data and control the environment.

  - **Goal:** We will then train a NeRF to encode the scene. The infrastructure
    for doing this already exists.

    - **Task Status:** This task is not yet accomplished though the concept is
      proven. Please see our computer vision project for visuals.

  - **Goal:** Once we know our pose in the NeRF frame, we will query a function
    that takes in a camera pose as input and outputs a 2D Image from the encoded
    nerf scene.

    - **Bottleneck:** Bottlenecked by Goal 2.

  - **Goal:** Use an OpenCV spot the difference function to compare the NeRF
    generated image to the actual image from the Neato’s camera. If there’s a
    difference, the terminal will print “Difference spotted”!
    - **Task Achieved:** We have scripts which will compare the differences
      between two images. We have two different implementations currently, one
      which is more robust to changes but does not output visuals, and one
      implementation which is less robust but will output exact difference masks
      between images.
    - **Comparison Stretch:** Some implementation of blur detection and removal
      from low data regions in a NeRF has been started.

## Milestone 2

- **Goal:** Encode a scene with a NeRF

  - **Status:** Completed

- **Goal:** Drive the Turtlebot while collecting odometry data from it

  - **Status:** Completed

- **Goal:** Use pose data from the odometry to create a Camera to World
  transform

  - **Status:** Completed

- **Goal:** Use the transform to retrieve an image from the NeRF

  - **Status:** Partially Complete
  - **Note:** The transform does not perfectly correspond between the NeRF and
    the Turtlebot and needs to be changed.

- **Goal:** Compare a NeRF to an actual image

  - **Status:** Partially Complete
  - **Note:** The NeRF pipeline works with the image comparison code, but
    because the poses do not match up, meaningful comparison cannot be
    completed.

- **Goal:** Pathplan an arbitrary space using the heuristic algorithm
  - **Status:** Partially Complete
  - **Note:** The circle packing algorithm works but the traveling salesman
    solver requires debugging.
