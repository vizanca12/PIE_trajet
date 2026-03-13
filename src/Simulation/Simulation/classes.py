from simulation_parameters import *
import pygame

class Buoy:
    """
    Représente une bouée obstacle dans l'environnement
    Utilisé pour la détection de collision
    """
    
    def __init__(self, x, y, radius=buoy_radius, color=ORANGE):
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color
        self.collision = False  # Flag pour visualisation collision
    
    def draw(self, screen):
        """Affichage de la bouée"""
        current_color = DARK_RED if self.collision else self.color

        buoy_x = int(self.x * scale)
        buoy_y = int(window_height - self.y * scale)
        
        # Cercle principal
        pygame.draw.circle(screen, current_color, (center_offset[0] + buoy_x, center_offset[1] + buoy_y), int(self.radius * scale))
        pygame.draw.circle(screen, BLACK, (center_offset[0] + buoy_x, center_offset[1] + buoy_y), int(self.radius * scale), 2)
        
        # Point central
        pygame.draw.circle(screen, WHITE, (center_offset[0] + buoy_x, center_offset[1] + buoy_y), 3)
        
    def check_collision_with_boat(self, boat):
        """
        Détection de collision cercle-rectangle
        Retourne True si collision détectée
        """
        # Test rapide: distance entre centres
        dx = self.x - boat.x
        dy = self.y - boat.y
        distance_center = math.sqrt(dx**2 + dy**2)
        
        if distance_center < self.radius + boat.width / 2:
            self.collision = True
            return True
        
        # Test précis: distance aux coins du bateau
        corners = boat.get_corners()
        for corner in corners:
            dx = self.x - corner[0]
            dy = self.y - corner[1]
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < self.radius:
                self.collision = True
                return True
        
        self.collision = False
        return False
    
    def get_position(self):
        """Retourne (x, y, radius) pour intégration path planning"""
        return (self.x, self.y, self.radius)


class Waypoint:
    """
    Représente un point de navigation (START ou GOAL)
    Utilisé pour définir les objectifs de navigation
    """
    
    def __init__(self, x, y, waypoint_type='goal', radius=1):
        self.x = x
        self.y = y
        self.radius = radius
        self.type = waypoint_type  # 'start' ou 'goal'
        self.reached = False
        
    def draw(self, screen, font):
        """Affichage du waypoint avec label"""
        if self.type == 'start':
            color = GREEN
            text = 'S'
            inner_color = (0, 200, 0)
        else:
            color = RED if not self.reached else YELLOW
            text = 'G'
            inner_color = (200, 0, 0) if not self.reached else (0, 200, 200)       
        
        ax_p = int(self.x * scale)
        ay_p = int(window_height - self.y * scale)
       
        # Cercles concentriques
        pygame.draw.circle(screen, color, (center_offset[0] + ax_p, center_offset[1] + ay_p), self.radius)
        pygame.draw.circle(screen, inner_color, (center_offset[0] + ax_p, center_offset[1] + ay_p), self.radius - 5)
        pygame.draw.circle(screen, BLACK, (center_offset[0] + ax_p, center_offset[1] + ay_p), self.radius, 3)
        
        # Label
        text_surface = font.render(text, True, WHITE)
        text_rect = text_surface.get_rect(center=(center_offset[0] + ax_p, center_offset[1] + ay_p))
        screen.blit(text_surface, text_rect)
        
    def check_reached(self, boat, tolerance=30):
        """Vérifie si le bateau a atteint ce waypoint"""
        dx = self.x - boat.x
        dy = self.y - boat.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < tolerance:
            self.reached = True
            return True
        return False
    
    def get_distance(self, boat):
        """Distance euclidienne au bateau"""
        dx = self.x - boat.x
        dy = self.y - boat.y
        return math.sqrt(dx**2 + dy**2)
    
    def get_position(self):
        """Retourne (x, y) pour intégration path planning"""
        return (self.x, self.y)

class DifferentialBoat:
    """
    Bateau avec entraînement différentiel (2 roues)
    Implémente la cinématique: v = (v_r + v_l)/2, ω = (v_r - v_l)/L
    """
    
    def __init__(self, x, y, theta, wheel_base=40, wheel_radius=10):
        # État courant
        self.x = x
        self.y = y
        self.theta = theta  # Orientation en radians
        
        # État initial (pour reset)
        self.initial_x = x
        self.initial_y = y
        self.initial_theta = theta
        
        # Paramètres physiques
        self.wheel_base = wheel_base      # Distance entre roues (L)
        self.wheel_radius = wheel_radius  # Rayon des roues (r)
        
        # Commandes (vitesses angulaires des roues)
        self.omega_left = 0.0
        self.omega_right = 0.0
        
        # Vitesses du robot
        self.v = 0.0      # Vitesse linéaire
        self.omega = 0.0  # Vitesse angulaire
        
        # Dimensions pour affichage
        self.length = 50
        self.width = 30
        
        # Trajectoire
        self.trail = []
        self.max_trail_length = 300
        
        # Métriques
        self.collision_count = 0
        self.total_distance = 0.0
        
    def set_wheel_velocities(self, omega_left, omega_right):
        """Définit les commandes (vitesses angulaires des roues)"""
        self.omega_left = omega_left
        self.omega_right = omega_right
        
    def update_kinematics(self, dt):
        """
        Mise à jour de la cinématique différentielle
        Équations:
        - v_wheel = r * ω_wheel
        - v = (v_right + v_left) / 2
        - ω = (v_right - v_left) / L
        - dx/dt = v * cos(θ)
        - dy/dt = v * sin(θ)
        - dθ/dt = ω
        """
        # Vitesses linéaires des roues
        v_left = self.wheel_radius * self.omega_left
        v_right = self.wheel_radius * self.omega_right
        
        # Vitesses du robot
        self.v = (v_right + v_left) / 2.0
        self.omega = (v_right - v_left) / self.wheel_base
        
        # Tracking distance
        distance_step = abs(self.v * dt)
        self.total_distance += distance_step
        
        # Intégration (méthode d'Euler)
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt
        
        # Normalisation de l'angle [-π, π]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        
        # Mise à jour trajectoire
        self.trail.append((int(self.x), int(self.y)))
        if len(self.trail) > self.max_trail_length:
            self.trail.pop(0)
    
    def reset_to_start(self):
        """Réinitialise le bateau à sa position initiale"""
        self.x = self.initial_x
        self.y = self.initial_y
        self.theta = self.initial_theta
        self.trail = []
        self.collision_count = 0
        self.total_distance = 0.0
        self.omega_left = 0.0
        self.omega_right = 0.0
        self.v = 0.0
        self.omega = 0.0
    
    def get_corners(self):
        """Calcule les 4 coins du rectangle représentant le bateau"""
        corners = [
            (-self.length / 2, -self.width / 2),
            (self.length / 2, -self.width / 2),
            (self.length / 2, self.width / 2),
            (-self.length / 2, self.width / 2)
        ]
        
        # Rotation + translation
        rotated_corners = []
        for corner in corners:
            x_rot = corner[0] * math.cos(self.theta) - corner[1] * math.sin(self.theta)
            y_rot = corner[0] * math.sin(self.theta) + corner[1] * math.cos(self.theta)
            rotated_corners.append((self.x + x_rot, self.y + y_rot))
        
        return rotated_corners
    
    def draw(self, screen):
        """Affichage du bateau et de sa trajectoire"""
        # Trajectoire
        if len(self.trail) > 1:
            pygame.draw.lines(screen, (100, 200, 100), False, self.trail, 2)
        
        # Corps du bateau
        corners = self.get_corners()
        pygame.draw.polygon(screen, BLUE, corners)
        pygame.draw.polygon(screen, BLACK, corners, 2)
        
        # Indicateur de direction (avant)
        front_x = self.x + (self.length / 2) * math.cos(self.theta)
        front_y = self.y + (self.length / 2) * math.sin(self.theta)
        pygame.draw.circle(screen, RED, (int(front_x), int(front_y)), 5)
        
    def get_state(self):
        """Retourne l'état complet (utile pour RL)"""
        return {
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'v': self.v,
            'omega': self.omega,
            'omega_left': self.omega_left,
            'omega_right': self.omega_right,
            'collision_count': self.collision_count,
            'total_distance': self.total_distance
        }
