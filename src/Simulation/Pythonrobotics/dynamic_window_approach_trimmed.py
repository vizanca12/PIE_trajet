"""

Mobile robot motion planning sample with Dynamic Window Approach

This module implements the Dynamic Window Approach (DWA) for mobile robot navigation.
DWA is a local motion planning algorithm that evaluates multiple trajectories in a
velocity space (dynamic window) to select the optimal control inputs that avoid
obstacles while heading towards the goal.

The algorithm works by:
1. Computing the dynamic window based on robot constraints and current state
2. Sampling velocity commands within this window
3. Predicting trajectories for each sample
4. Evaluating trajectories using cost functions (goal distance, speed, obstacle avoidance)
5. Selecting the trajectory with minimum cost

Visualization is provided using Pygame for real-time animation.

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math
from enum import Enum

import pygame
import numpy as np
import sys
sys.path.insert(0, "./Pythonrobotics") # Pour les autres imports

# IMPORTANT LA CONFIG EST FAITE A PARTIR DE CE FICHIER IMPORTE
from simulation_parameters import *


def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control

    Computes the optimal velocity commands (linear and angular) by evaluating
    trajectories within the dynamic window and selecting the one with minimum cost.

    Args:
        x: Current robot state [x, y, yaw, v, omega]
        config: Configuration object with simulation parameters
        goal: Target position [gx, gy]
        ob: Obstacle positions array

    Returns:
        u: Optimal control inputs [v, omega]
        trajectory: Predicted trajectory for the optimal inputs
    """
    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory


class RobotType(Enum):
    """Enumeration for different robot shapes used in collision detection."""
    circle = 0
    rectangle = 1


class Config:
    """
    Simulation parameter class containing all configuration parameters for DWA.

    This class holds robot physical constraints, cost function weights, obstacle
    positions, and visualization settings used throughout the simulation.
    """

    def __init__(self):
        # Robot kinematic constraints
        self.max_speed = vehicle_max_speed  # [m/s] Maximum linear velocity
        self.min_speed = vehicle_min_speed  # [m/s] Minimum linear velocity (allows backward motion)
        self.max_yaw_rate = vehicle_max_yaw_rate  # [rad/s] Maximum angular velocity
        self.max_accel = vehicle_max_accel  # [m/ss] Maximum linear acceleration
        self.max_delta_yaw_rate = vehicle_max_delta_yaw_rate  # [rad/ss] Maximum angular acceleration
        self.v_resolution = sampling_v_resolution  # [m/s] Resolution for velocity sampling
        self.yaw_rate_resolution = sampling_yaw_rate_resolution  # [rad/s] Resolution for yaw rate sampling
        self.dt = 0.1  # [s] Time step for motion prediction
        self.predict_time = 3.0  # [s] Prediction horizon for trajectory evaluation

        # Cost function weights
        self.to_goal_cost_gain = 0.15  # Weight for goal-oriented cost
        self.speed_cost_gain = 1.0  # Weight for preferring higher speeds
        self.obstacle_cost_gain = 1.0  # Weight for obstacle avoidance
        self.robot_stuck_flag_cons = 0.001  # Threshold to detect robot stuck condition

        # Robot shape parameters
        self.robot_type = RobotType.circle  # Shape type for collision detection
        self.robot_radius = 1.0  # [m] Radius for circular robot collision check
        self.robot_width = vehicle_width  # [m] Width for rectangular robot collision check
        self.robot_length = vehicle_length  # [m] Length for rectangular robot collision check

        # Obstacle positions [x(m), y(m), ...]
        self.ob = np.array(buoy_positions)

        # Pygame visualization parameters
        self.scale = scale  # Pixels per meter for display scaling
        self.window_width = window_width  # Window width in pixels
        self.window_height = window_height  # Window height in pixels

    @property
    def robot_type(self):
        """Get the current robot type for collision detection."""
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        """Set the robot type, ensuring it's a valid RobotType enum value."""
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


config = Config()


def motion(x, u, dt):
    """
    Differential drive motion model for robot kinematics.

    Updates the robot state based on velocity commands using the bicycle model.
    State vector: [x, y, yaw, v, omega]

    Args:
        x: Current state [x, y, yaw, v, omega]
        u: Control inputs [linear_velocity, angular_velocity]
        dt: Time step

    Returns:
        Updated state vector
    """
    x[2] += u[1] * dt  # Update yaw angle
    x[0] += u[0] * math.cos(x[2]) * dt  # Update x position
    x[1] += u[0] * math.sin(x[2]) * dt  # Update y position
    x[3] = u[0]  # Update linear velocity
    x[4] = u[1]  # Update angular velocity

    return x


def calc_dynamic_window(x, config):
    """
    Calculate the dynamic window based on current state and constraints.

    The dynamic window is the set of achievable velocities in the next time step,
    constrained by robot physical limits and current motion state.

    Args:
        x: Current robot state [x, y, yaw, v, omega]
        config: Configuration parameters

    Returns:
        dw: Dynamic window [v_min, v_max, omega_min, omega_max]
    """
    # Static constraints from robot specifications
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]

    # Dynamic constraints from current motion (acceleration limits)
    Vd = [x[3] - config.max_accel * config.dt,  # Min velocity reachable
          x[3] + config.max_accel * config.dt,  # Max velocity reachable
          x[4] - config.max_delta_yaw_rate * config.dt,  # Min yaw rate reachable
          x[4] + config.max_delta_yaw_rate * config.dt]  # Max yaw rate reachable

    # Intersection of static and dynamic constraints
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    Predict the trajectory for given constant velocity commands over prediction time.

    Simulates the robot motion forward in time using the kinematic model to
    generate a sequence of states that would result from applying constant
    linear velocity v and angular velocity y.

    Args:
        x_init: Initial state [x, y, yaw, v, omega]
        v: Linear velocity command
        y: Angular velocity command
        config: Configuration parameters

    Returns:
        trajectory: Array of predicted states over time
    """
    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    Evaluate all possible trajectories in the dynamic window and select the optimal one.

    Samples velocity commands within the dynamic window, predicts trajectories,
    evaluates each using cost functions, and selects the trajectory with minimum cost.
    Includes a stuck detection mechanism to prevent the robot from getting trapped.

    Args:
        x: Current robot state
        dw: Dynamic window [v_min, v_max, omega_min, omega_max]
        config: Configuration parameters
        goal: Target position [gx, gy]
        ob: Obstacle positions

    Returns:
        best_u: Optimal control inputs [v, omega]
        best_trajectory: Predicted trajectory for optimal inputs
    """
    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # Sample velocity commands within the dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

            trajectory = predict_trajectory(x_init, v, y, config)

            # Calculate costs for this trajectory
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)

            final_cost = to_goal_cost + speed_cost + ob_cost

            # Update best trajectory if this one has lower cost
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory

                # Stuck detection: if robot is nearly stopped and facing goal,
                # force rotation to avoid getting stuck
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[3]) < config.robot_stuck_flag_cons:
                    best_u[1] = -config.max_delta_yaw_rate

    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
    Calculate obstacle avoidance cost for a trajectory.

    Returns infinity if collision detected, otherwise returns 1/min_distance
    to encourage keeping distance from obstacles.

    Args:
        trajectory: Predicted trajectory states
        ob: Obstacle positions array
        config: Configuration parameters

    Returns:
        Cost value (infinity for collision, 1/min_distance otherwise)
    """
    ox = ob[:, 0]  # Obstacle x coordinates
    oy = ob[:, 1]  # Obstacle y coordinates

    # Calculate distances from all trajectory points to all obstacles
    dx = trajectory[:, 0] - ox[:, None]  # Broadcasting for distance matrix
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)  # Euclidean distances

    # Collision detection based on robot shape
    if config.robot_type == RobotType.rectangle:
        # Transform obstacles to robot's local coordinate frame
        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])  # Shape for broadcasting

        # Transform obstacle positions to robot frame
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])

        # Check if any obstacle is inside robot's rectangular bounds
        upper_check = local_ob[:, 0] <= config.robot_length / 2
        right_check = local_ob[:, 1] <= config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2
        left_check = local_ob[:, 1] >= -config.robot_width / 2

        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")  # Collision detected

    elif config.robot_type == RobotType.circle:
        # Simple circular collision check
        if np.array(r <= config.robot_radius).any():
            return float("Inf")  # Collision detected

    # No collision: return inverse of minimum distance (higher cost for closer obstacles)
    min_r = np.min(r)
    return 1.0 / min_r


def calc_to_goal_cost(trajectory, goal):
    """
    Calculate the cost for heading towards the goal based on angular difference.

    Computes the angle between the goal direction and the robot's final heading,
    normalized to [-pi, pi] range.

    Args:
        trajectory: Predicted trajectory
        goal: Target position [gx, gy]

    Returns:
        Angular cost (smaller when robot is facing towards goal)
    """
    # Calculate desired heading towards goal
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)

    # Calculate difference between desired and actual heading
    cost_angle = error_angle - trajectory[-1, 2]

    # Normalize angle to [-pi, pi]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost


def draw_arrow(screen, x, y, yaw, config, length=0.5, width=0.1):  # pragma: no cover
    """
    Draw an arrow indicating the robot's heading direction.

    Args:
        screen: Pygame screen surface
        x, y: Robot position in meters
        yaw: Robot orientation in radians
        config: Configuration with scale and window parameters
        length: Arrow length in meters
        width: Arrow width (not used in current implementation)
    """
    start_x = x * config.scale
    start_y = config.window_height - y * config.scale
    end_x = start_x + length * config.scale * math.cos(yaw)
    end_y = start_y - length * config.scale * math.sin(yaw)
    pygame.draw.line(screen, (0, 0, 0), (start_x, start_y), (end_x, end_y), 2)


def draw_robot(screen, x, y, yaw, config):  # pragma: no cover
    """
    Draw the robot on the screen based on its shape type.

    For circular robots: draws a circle with a direction line.
    For rectangular robots: draws a rotated rectangle.

    Args:
        screen: Pygame screen surface
        x, y: Robot position in meters
        yaw: Robot orientation in radians
        config: Configuration parameters
    """
    if config.robot_type == RobotType.rectangle:
        # Define rectangle corners in local frame
        half_length = config.robot_length / 2 * config.scale
        half_width = config.robot_width / 2 * config.scale
        points = [
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width)
        ]

        # Apply rotation transformation (note: cos_yaw negated for correct orientation)
        cos_yaw = -math.cos(yaw)
        sin_yaw = math.sin(yaw)
        rotated_points = []
        for px, py in points:
            rx = px * cos_yaw - py * sin_yaw + x * config.scale
            ry = px * sin_yaw + py * cos_yaw + (config.window_height - y * config.scale)
            rotated_points.append((rx, ry))

        pygame.draw.polygon(screen, (0, 0, 255), rotated_points)

    elif config.robot_type == RobotType.circle:
        # Draw circular robot
        center_x = x * config.scale
        center_y = config.window_height - y * config.scale
        radius = int(config.robot_radius * config.scale)
        pygame.draw.circle(screen, (0, 0, 255), (int(center_x), int(center_y)), radius)

        # Draw direction indicator line
        end_x = center_x + radius * math.cos(yaw)
        end_y = center_y - radius * math.sin(yaw)
        pygame.draw.line(screen, (0, 0, 0), (center_x, center_y), (end_x, end_y), 2)
