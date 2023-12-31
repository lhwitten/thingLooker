# NeRF Spot The Difference

Computational Robotics Final for Jessica Brown and Luke Witten

## Navigation

- [Home](index.md): Overview
- [Path Planning](path-planning.md): Novel Exploration Heuristic
- [Computer Vision](computer-vision.md): NeRF implementation and image
  comparison details
- [Milestones and Ethics Statement](milestones.md):

## About This Project

Our Project uses Neural Radiance Fields, image comparison, and a pathplanning
heuristic to autonomously explore a space twice and spot differences between the
two scenes. This requires training a NeRF on the first pass of the scene,
passing in images and odometry data to train the neural network. On the second
run through the environment, the robot takes pictures of the environment and
uses computer vision to compare those pictures to representations of the
previous scene generated by the NeRF. In both cases, the robot uses an
exploration heuristic based on circle packing to efficiently view the entire
space.
