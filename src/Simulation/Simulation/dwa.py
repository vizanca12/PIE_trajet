# Import utilities
import pygame
import numpy as np
import math
import sys
sys.path.insert(0, "./Pythonrobotics") # Pour les autres imports
import dynamic_window_approach_trimmed as planner
import state # temporaire

#from enum import Enum


# Autres Fichiers
from pygame_manager import * # initialise pygames et la cam
from classes import * # Définit les bouées, les waypoints et le bateau


def main():
    """
    Main simulation function demonstrating Dynamic Window Approach navigation.

    Sets up the simulation environment, runs the control loop with Pygame visualization,
    and displays the final trajectory when the goal is reached.

    Args:
        gx, gy: Goal position coordinates
        robot_type: Shape of the robot (circle or rectangle)
    """

    #################### Initialisation and unique settings ####################

    print(__file__ + " start!!")

    goal = Waypoint(10.0, -60.0, waypoint_type='goal', radius=2) # Goal

    buoys = []
    for buoy in buoy_positions :
        buoys.append(Buoy(buoy[0], buoy[1])) # Buoys


    # Initialize robot state: [x, y, yaw, v, omega]
    x = np.array([init_x, init_y, init_yaw, init_v, init_yaw])

    planner.config.robot_type = planner.RobotType.rectangle
    trajectory = np.array(x)  # Will store complete trajectory history
    ob = planner.config.ob  # Obstacle positions

    

    #################### Main loop ####################

    running = True
    goal_reached = False
    # Main simulation loop
    while running and not goal_reached:
        # Handle Pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        # Compute optimal control inputs using DWA
        goal_position = goal.get_position()
        u, predicted_trajectory = planner.dwa_control(x, planner.config, np.array(goal_position), ob)

        # Update robot state with optimal inputs
        x = planner.motion(x, u, planner.config.dt)
        trajectory = np.vstack((trajectory, x))  # Store state history

        # Render visualization if enabled
        if show_animation:
            screen.fill(WHITE)
            draw_basic_screen(screen)

            for buoy in buoys:
                buoy.draw(screen)

            goal.draw(screen, font)

            temp_x = predicted_trajectory[:, 0]
            temp_y = predicted_trajectory[:, 1]
            draw_vehicle_trajectory(screen, temp_x, temp_y)

            temp_state = state.State(x[0], x[1], x[2], init_v) # Draw robot
            draw_vehicle(screen, temp_state)

            # Draw heading arrow
            planner.draw_arrow(screen, x[0], x[1], x[2], planner.config)

            pygame.display.flip()
            clock.tick(60)  # Limit to 60 FPS

        # Check if goal is reached (robot center within radius of goal)
        dist_to_goal = math.hypot(x[0] - goal_position[0], x[1] - goal_position[1])
        if dist_to_goal <= planner.config.robot_radius:
            print("Goal!!")
            goal_reached = True

    print("Done")

    # Display final trajectory when goal reached
    if show_animation and goal_reached:
        screen.fill(WHITE)
        draw_basic_screen(screen)

        for buoy in buoys:
                buoy.draw(screen)
        draw_basic_screen(screen)

        goal.draw(screen, font)

        temp_x = trajectory[:, 0]
        temp_y = trajectory[:, 1]
        draw_reference_path(screen, temp_x, temp_y)

        pygame.display.flip()

        # Wait for user to close window
        waiting = True
        while waiting:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    waiting = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        waiting = False

    pygame.quit() 

if __name__ == '__main__':
    main()