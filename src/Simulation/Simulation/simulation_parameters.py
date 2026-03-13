import os # Pour les chemins de fichiers
import math
import numpy as np

"""
This file contains common basic parameters for most displays
"""


#################### Simulation settings ####################
show_animation = True
max_simulation_time = 100.0
simulation_height = (-80, 50)  # in meters (= limite réelle minimale)
simulation_width = (-80, 80) # in meters  (= limite réelle minimale)

sampling_v_resolution = 0.01  # [m/s] Resolution for velocity sampling
sampling_yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s] Resolution for yaw rate sampling

# Inital state
init_x = 0.0 # in meters
init_y = 40.0 # in meters
init_yaw = np.radians(20.0) # in radians
init_v = 0.0 # in m/s
init_omega = 0.0 # in rad/s (=\dot{yaw})

#################### Map settings ####################
# Buoy parameters
buoy_positions = [(0, 0), (10.25, -54.75)] # in meters
buoy_radius = 1.5  # in meters
# Vehicle parameters
vehicle_length = 4.5  # meters
vehicle_width = 2.0   # meters
vehicle_max_speed = 10.0  # [m/s] Maximum linear velocity
vehicle_min_speed = -1  # [m/s] Minimum linear velocity (allows backward motion)
vehicle_max_yaw_rate = 45.0 * math.pi / 180.0  # [rad/s] Maximum angular velocity
vehicle_max_accel = 1  # [m/ss] Maximum linear acceleration
vehicle_max_delta_yaw_rate = 45.0 * math.pi / 180.0  # [rad/ss] Maximum angular acceleration

# Starting zone parameters
start_zone = ((-5, 45), (5, 35)) # x_top_left_x, y_top_left_y, x_bottom_right_x, y_bottom_right_y  (in meters)

##################### Pygames settings ####################
# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 100, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
ORANGE = (255, 165, 0)
YELLOW = (255, 255, 0)
DARK_RED = (139, 0, 0)
PURPLE = (128, 0, 128)
CYAN = (0, 255, 255)

# Pygame visualization settings
window_width = 1200 # pixels (= taille de la fenêtre)
window_height = 800 # pixels (= taille de la fenêtre)
border_offset_top = 0  # pixels (= margin around the simulation area)
border_offset_left = 210  # pixels
border_offset_right = 0  # pixels
border_offset_bottom = 0  # pixels
background_image_path = os.path.join("Images","Port de monaco 2.png")
background_image_zero_position_local = (575, 417)  # pixels
# Il faut ajuster cette position pour que l'origine (0,0) de la simulation corresponde au bon point sur l'image
# L'origine (0,0) de la simulation est la bouée la plus proche du coin supérieur gauche de l'image
background_image_scale_local = 100.0/20.0

    #----------------- Scaling the window  -----------------#        
#compute scale factor to convert meters to pixels
scale_height = (window_height - border_offset_top - border_offset_bottom) // (simulation_height[1] - simulation_height[0])  # pixels per meter
scale_width = (window_width - border_offset_left - border_offset_right)  // (simulation_width[1] - simulation_width[0])  # pixels per meter
scale = min(scale_height, scale_width) # pixels per meter
# Scale factor for background image to match simulation scale
factor = scale / background_image_scale_local
# Center the simulation area in the window
center = ((simulation_width[1] + simulation_width[0]) // 2, (simulation_height[1] + simulation_height[0]) // 2) # in meters
center_offset = (window_width // 2 - center[0]*scale + border_offset_left/scale, - window_height // 2 + center[1]*scale + border_offset_bottom)  # in pixels
# ^--- Avec décalage du centre pour laisser les bordures
# Position du fond
background_image_zero_position = (center_offset[0] - background_image_zero_position_local[0] * factor, window_height + center_offset[1] - background_image_zero_position_local[1] * factor)  # pixels