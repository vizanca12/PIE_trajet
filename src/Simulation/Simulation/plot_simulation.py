from classes import *
import pygame
import numpy as np

"""
This file contains plot functions to help reduce the number of lines in the main code
"""

def initialize_background() :
    """
    Load the background once pygame is initialized and shape it to the inner frame
    """

    # Load and scale background image
    background_img = pygame.image.load(background_image_path).convert_alpha() # preserves transparency
    # Scale the background image to match the simulation scale
    original_size = background_img.get_size()
    new_size = (int(original_size[0] * factor), int(original_size[1] * factor))
    background_img = pygame.transform.scale(background_img, new_size)

    # Crop the image to fit within the simulation red rectangle
    red_top_left_x = center_offset[0] + simulation_width[0] * scale
    red_top_left_y = center_offset[1] + window_height - simulation_height[1] * scale
    red_width = (simulation_width[1] - simulation_width[0]) * scale
    red_height = (simulation_height[1] - simulation_height[0]) * scale
    crop_x_start = max(0, int(red_top_left_x - background_image_zero_position[0]))
    crop_y_start = max(0, int(red_top_left_y - background_image_zero_position[1]))
    crop_width = min(int(red_width), background_img.get_width() - crop_x_start)
    crop_height = min(int(red_height), background_img.get_height() - crop_y_start)
    background_img = background_img.subsurface((crop_x_start, crop_y_start, crop_width, crop_height))
    background_image_position = (background_image_zero_position[0] + crop_x_start, background_image_zero_position[1] + crop_y_start)
    return background_img, background_image_position

def draw_starting_zone(screen):
    # Draw starting zone as green rectangle
    start_zone_top_left = (center_offset[0] + start_zone[0][0] * scale, center_offset[1] + window_height - (start_zone[0][1]) * scale)
    start_zone_bottom_right = (center_offset[0] + (start_zone[1][0]) * scale, center_offset[1] + window_height - start_zone[1][1] * scale)
    pygame.draw.rect(screen, GREEN, (start_zone_top_left[0], start_zone_top_left[1], start_zone_bottom_right[0] - start_zone_top_left[0], start_zone_bottom_right[1] - start_zone_top_left[1]))

def draw_reference_path(screen, cx, cy):
    # Draw reference path as red dots
    for i in range(len(cx)):
        cx_p = int(cx[i] * scale)
        cy_p = int(window_height - cy[i] * scale)
        pygame.draw.circle(screen, RED, (center_offset[0] + cx_p, center_offset[1] + cy_p), 2)

def draw_vehicle_trajectory(screen, x, y):
    # Draw vehicle trajectory as blue line
    if len(x) > 1:
        for i in range(1, len(x)):
            tx1 = int(x[i-1] * scale)
            ty1 = int(window_height - y[i-1] * scale)
            tx2 = int(x[i] * scale)
            ty2 = int(window_height - y[i] * scale)
            pygame.draw.line(screen, BLUE, (center_offset[0] + tx1, center_offset[1] + ty1), (center_offset[0] + tx2, center_offset[1] + ty2), 2)

def draw_current_target_point(screen, cx, cy, target_idx):
    # Draw current target point as green circle
    target_x = int(cx[target_idx] * scale)
    target_y = int(window_height - cy[target_idx] * scale)
    pygame.draw.circle(screen, GREEN, (center_offset[0] + target_x, center_offset[1] + target_y), scale)

def draw_vehicle(screen, state):
    """
    Draw the vehicle as a blue rectangle on the Pygame screen.

    The vehicle is represented as a rectangle rotated according to its yaw angle.
    """

    # Convert to pixels
    half_length = vehicle_length / 2
    half_width = vehicle_width / 2
    # Define rectangle corners relative to center
    points = [
        (-half_length, -half_width),
        (half_length, -half_width),
        (half_length, half_width),
        (-half_length, half_width)
    ]
    # Rotation matrix for yaw (note: sin_yaw negated for correct orientation in Pygame)
    cos_yaw = np.cos(state.yaw)
    sin_yaw = -np.sin(state.yaw)
    rotated_points = []
    for px, py in points:
        # Rotate and translate to vehicle position
        rx = (px * cos_yaw - py * sin_yaw + state.x) * scale + center_offset[0]
        ry = (px * sin_yaw + py * cos_yaw + window_height / scale - state.y) * scale + center_offset[1]
        rotated_points.append((rx, ry))
    pygame.draw.polygon(screen, BLUE, rotated_points)

def draw_borders(screen):
    # Draw borders
    top_left = (center_offset[0] + simulation_width[0] * scale, center_offset[1] + window_height - simulation_height[1] * scale)
    top_right = (center_offset[0] + simulation_width[1] * scale,  center_offset[1] + window_height - simulation_height[1] * scale)
    bottom_left = (center_offset[0] + simulation_width[0] * scale, center_offset[1] + window_height - simulation_height[0] * scale)
    bottom_right = (center_offset[0] + simulation_width[1] * scale, center_offset[1] + window_height - simulation_height[0] * scale)
    pygame.draw.polygon(screen, RED, [top_left, top_right, bottom_right, bottom_left], 2)

def display_current_state(screen, state, time, font):
    # Display current state and time
    speed_text = font.render(f"Speed: {state.v * 3.6:.2f} km/h", True, BLACK)
    screen.blit(speed_text, (10, 10))
    x_text = font.render(f"X: {state.x:.2f} m", True, BLACK)
    screen.blit(x_text, (10, 30))
    y_text = font.render(f"Y: {state.y:.2f} m", True, BLACK)
    screen.blit(y_text, (10, 50))
    yaw_text = font.render(f"Yaw: {state.y:.2f} °", True, BLACK)
    screen.blit(yaw_text, (10, 70))
    time_text = font.render(f"Time: {time:.0f} s", True, BLACK)
    screen.blit(time_text, (10, 90))