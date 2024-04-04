# Importing  library
from sensor_msgs.msg import JointState

# Importing numpy for mathematical operations and List for type hinting
import numpy as np
from typing import List

# Indexing values for left and right
LEFT = 0
RIGHT = 1

# Indexing values for X, Y, and THETA
X = 0
Y = 1
THETA = 2

# This function calculates the change in wheel angles and time
def calculate_wheel_change(new_joint_states: JointState, prev_joint_states: JointState) -> List[float]:
    """
    Calculate change in wheel angles and time
    
    Args:
        new_joint_states: New joint states
        prev_joint_states: Previous joint states
    
    Returns:
        List[float]: Change in left wheel angle [rad], change in right wheel angle [rad], change in time [s]
    """
    
    # Calculate the change in left and right wheel angles
    delta_wheel_l = new_joint_states.position[LEFT] - prev_joint_states.position[LEFT]
    delta_wheel_r = new_joint_states.position[RIGHT] - prev_joint_states.position[RIGHT]
    
    # Calculate the change in time
    delta_time = new_joint_states.header.stamp.to_sec() - prev_joint_states.header.stamp.to_sec()
    
    # Data validation for NaN values
    if np.isnan(delta_wheel_l):
        delta_wheel_l = 0.0
    if np.isnan(delta_wheel_r):
        delta_wheel_r = 0.0

    return (delta_wheel_l, delta_wheel_r, delta_time)


# This function calculates the linear and angular displacement
def calculate_displacement(delta_wheel_l: float, delta_wheel_r: float, wheel_radius: float, wheel_separation: float) -> List[float]:
    """
    Calculate linear and angular displacement
    
    Args:
        delta_wheel_l: Change in left wheel angle [rad]
        delta_wheel_r: Change in right wheel angle [rad]
        wheel_radius: Wheel radius [m]
        wheel_separation: Wheel separation [m]
    
    Returns:
        List[float]: Linear displacement [m], angular displacement [rad]
    """
    
    # Calculate linear displacement
    delta_s = (wheel_radius * delta_wheel_r + wheel_radius * delta_wheel_l) / 2
    
    # Calculate angular displacement
    delta_theta = (wheel_radius * delta_wheel_r - wheel_radius * delta_wheel_l) / wheel_separation

    return (delta_s, delta_theta)


# This function calculates the new pose based on previous pose and displacement
def calculate_pose(prev_pose: List[float], delta_s: float, delta_theta: float) -> List[float]:
    """
    Calculate new pose based on previous pose and displacement
    
    Args:
        prev_pose: Input pose in format (x, y, theta) [m, m, rad]
        delta_s: Linear displacement [m]
        delta_theta: Angular displacement [rad]
    
    Returns:
        List[float]: Output pose in format (x, y, theta) [m, m, rad]
    """
    
    # Unpack previous pose values
    x_prev, y_prev, theta_prev = prev_pose
    
    # Calculate new x, y, and theta
    x = x_prev + delta_s * np.cos(theta_prev + (delta_theta/2))
    y = y_prev + delta_s * np.sin(theta_prev + (delta_theta/2))
    theta = theta_prev + delta_theta
    
    # New pose
    pose = [x, y, theta]

    return pose
