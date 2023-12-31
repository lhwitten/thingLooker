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
