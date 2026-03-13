from plot_simulation import *
import pygame

"""
Initialise pygames and define more high-level functions
"""

window_name = "DWA with Stanley Control Simulation"
#"Cubic Spine Planner with Stanley Control Simulation"

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((window_width, window_height))
pygame.display.set_caption(window_name)
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)

background_img, background_image_position = initialize_background()

def draw_basic_screen(screen):
    """
    This function draw the basic requirements to print something using only the screen variable
    """

    screen.blit(background_img, background_image_position) # Draw background image             
    draw_borders(screen) # Draw borders of the screen
    draw_starting_zone(screen) # Draw starting zone as green rectangle