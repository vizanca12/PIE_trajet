"""
BlueBoat Trajectory Simulator
Simulador de trajetória para o BlueBoat da BlueRobotics
com definição de waypoints e desvio de obstáculos
"""

import pygame
import math
import random
from dataclasses import dataclass
from typing import List, Tuple, Optional
from enum import Enum

# Inicialização do Pygame
pygame.init()

# Constantes de configuração
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
FPS = 60

# Cores
WATER_COLOR = (30, 100, 150)
BOAT_COLOR = (255, 200, 50)
WAYPOINT_COLOR = (50, 255, 100)
WAYPOINT_REACHED_COLOR = (100, 100, 100)
OBSTACLE_COLOR = (200, 50, 50)
PATH_COLOR = (255, 255, 255, 128)
TRAJECTORY_COLOR = (100, 200, 255)
UI_BG_COLOR = (20, 40, 60)
TEXT_COLOR = (255, 255, 255)
SENSOR_COLOR = (255, 255, 0, 50)


class BoatState(Enum):
    IDLE = "Parado"
    NAVIGATING = "Navegando"
    AVOIDING = "Desviando"
    COMPLETED = "Concluído"


@dataclass
class Vector2:
    x: float
    y: float
    
    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)
    
    def __mul__(self, scalar):
        return Vector2(self.x * scalar, self.y * scalar)
    
    def magnitude(self):
        return math.sqrt(self.x**2 + self.y**2)
    
    def normalize(self):
        mag = self.magnitude()
        if mag > 0:
            return Vector2(self.x / mag, self.y / mag)
        return Vector2(0, 0)
    
    def to_tuple(self):
        return (int(self.x), int(self.y))


@dataclass
class Obstacle:
    position: Vector2
    radius: float
    velocity: Vector2 = None
    
    def __post_init__(self):
        if self.velocity is None:
            self.velocity = Vector2(0, 0)


class BlueBoat:
    """Simulação do BlueBoat da BlueRobotics"""
    
    def __init__(self, x: float, y: float):
        self.position = Vector2(x, y)
        self.velocity = Vector2(0, 0)
        self.heading = 0  # Ângulo em radianos
        self.max_speed = 3.0  # m/s (escala simulada)
        self.turn_rate = 2.0  # rad/s
        self.sensor_range = 100  # pixels
        self.sensor_angle = math.pi / 3  # 60 graus
        self.state = BoatState.IDLE
        self.trajectory: List[Vector2] = []
        
        # Dimensões do barco (proporcional ao BlueBoat real)
        self.length = 40
        self.width = 20
        
    def get_polygon_points(self) -> List[Tuple[int, int]]:
        """Retorna os pontos do polígono do barco"""
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)
        
        # Pontos do barco (proa triangular)
        points = [
            (self.length/2, 0),      # Proa
            (-self.length/2, -self.width/2),  # Popa esquerda
            (-self.length/3, 0),     # Centro traseiro
            (-self.length/2, self.width/2),   # Popa direita
        ]
        
        # Rotaciona e translada os pontos
        transformed = []
        for px, py in points:
            rx = px * cos_h - py * sin_h + self.position.x
            ry = px * sin_h + py * cos_h + self.position.y
            transformed.append((int(rx), int(ry)))
        
        return transformed
    
    def detect_obstacles(self, obstacles: List[Obstacle]) -> List[Tuple[Obstacle, float, float]]:
        """Detecta obstáculos dentro do alcance do sensor"""
        detected = []
        
        for obstacle in obstacles:
            diff = obstacle.position - self.position
            distance = diff.magnitude() - obstacle.radius
            
            if distance < self.sensor_range:
                # Calcula o ângulo relativo ao heading do barco
                angle_to_obstacle = math.atan2(diff.y, diff.x)
                relative_angle = angle_to_obstacle - self.heading
                
                # Normaliza o ângulo
                while relative_angle > math.pi:
                    relative_angle -= 2 * math.pi
                while relative_angle < -math.pi:
                    relative_angle += 2 * math.pi
                
                # Verifica se está no cone do sensor
                if abs(relative_angle) < self.sensor_angle:
                    detected.append((obstacle, distance, relative_angle))
        
        return sorted(detected, key=lambda x: x[1])
    
    def calculate_avoidance_vector(self, obstacles: List[Tuple[Obstacle, float, float]]) -> Optional[Vector2]:
        """Calcula vetor de desvio baseado nos obstáculos detectados"""
        if not obstacles:
            return None
        
        avoidance = Vector2(0, 0)
        
        for obstacle, distance, rel_angle in obstacles:
            if distance < self.sensor_range * 0.8:
                # Força de repulsão inversamente proporcional à distância
                strength = (self.sensor_range - distance) / self.sensor_range
                
                # Direção de desvio (perpendicular ao obstáculo)
                if rel_angle > 0:
                    avoid_angle = self.heading - math.pi/2
                else:
                    avoid_angle = self.heading + math.pi/2
                
                avoidance.x += math.cos(avoid_angle) * strength * 2
                avoidance.y += math.sin(avoid_angle) * strength * 2
        
        return avoidance if avoidance.magnitude() > 0.1 else None
    
    def update(self, target: Optional[Vector2], obstacles: List[Obstacle], dt: float):
        """Atualiza a posição e orientação do barco"""
        self.trajectory.append(Vector2(self.position.x, self.position.y))
        
        # Limita o histórico da trajetória
        if len(self.trajectory) > 500:
            self.trajectory.pop(0)
        
        if target is None:
            self.state = BoatState.IDLE
            self.velocity = Vector2(0, 0)
            return
        
        # Detecta obstáculos
        detected_obstacles = self.detect_obstacles(obstacles)
        avoidance = self.calculate_avoidance_vector(detected_obstacles)
        
        # Calcula direção para o alvo
        to_target = target - self.position
        distance = to_target.magnitude()
        
        if distance < 15:
            return True  # Chegou ao waypoint
        
        # Determina a direção desejada
        if avoidance and detected_obstacles[0][1] < 60:
            self.state = BoatState.AVOIDING
            desired_direction = (to_target.normalize() + avoidance).normalize()
        else:
            self.state = BoatState.NAVIGATING
            desired_direction = to_target.normalize()
        
        # Calcula o ângulo desejado
        desired_heading = math.atan2(desired_direction.y, desired_direction.x)
        
        # Suaviza a rotação
        angle_diff = desired_heading - self.heading
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Aplica rotação limitada
        max_turn = self.turn_rate * dt
        if abs(angle_diff) > max_turn:
            angle_diff = max_turn if angle_diff > 0 else -max_turn
        
        self.heading += angle_diff
        
        # Calcula velocidade (reduz se precisar virar muito)
        speed_factor = 1.0 - min(abs(angle_diff) / math.pi, 0.8)
        speed = self.max_speed * speed_factor
        
        # Atualiza posição
        self.velocity = Vector2(
            math.cos(self.heading) * speed,
            math.sin(self.heading) * speed
        )
        self.position = self.position + self.velocity * dt * 20
        
        return False


class Simulator:
    """Classe principal do simulador"""
    
    def __init__(self):
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("BlueBoat Trajectory Simulator - BlueRobotics")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)
        self.title_font = pygame.font.Font(None, 36)
        
        # Área de simulação (exclui painel lateral)
        self.sim_area = pygame.Rect(0, 0, WINDOW_WIDTH - 250, WINDOW_HEIGHT)
        
        # Elementos da simulação
        self.boat = BlueBoat(100, WINDOW_HEIGHT // 2)
        self.waypoints: List[Vector2] = []
        self.current_waypoint_index = 0
        self.obstacles: List[Obstacle] = []
        
        # Estado da UI
        self.running = True
        self.paused = False
        self.mode = "waypoint"  # "waypoint" ou "obstacle"
        self.show_sensors = True
        self.show_grid = True
        
        # Gera alguns obstáculos iniciais
        self.generate_random_obstacles(5)
    
    def generate_random_obstacles(self, count: int):
        """Gera obstáculos aleatórios"""
        for _ in range(count):
            x = random.randint(200, self.sim_area.width - 100)
            y = random.randint(100, self.sim_area.height - 100)
            radius = random.randint(20, 50)
            self.obstacles.append(Obstacle(Vector2(x, y), radius))
    
    def draw_grid(self):
        """Desenha grade de referência"""
        if not self.show_grid:
            return
        
        grid_color = (40, 80, 120)
        spacing = 50
        
        for x in range(0, self.sim_area.width, spacing):
            pygame.draw.line(self.screen, grid_color, (x, 0), (x, self.sim_area.height), 1)
        
        for y in range(0, self.sim_area.height, spacing):
            pygame.draw.line(self.screen, grid_color, (0, y), (self.sim_area.width, y), 1)
    
    def draw_water_effect(self):
        """Desenha efeito de água"""
        self.screen.fill(WATER_COLOR, self.sim_area)
    
    def draw_trajectory(self):
        """Desenha a trajetória percorrida pelo barco"""
        if len(self.boat.trajectory) > 1:
            points = [p.to_tuple() for p in self.boat.trajectory]
            pygame.draw.lines(self.screen, TRAJECTORY_COLOR, False, points, 2)
    
    def draw_planned_path(self):
        """Desenha o caminho planejado (waypoints)"""
        if len(self.waypoints) > self.current_waypoint_index:
            # Linha do barco até o próximo waypoint
            if self.current_waypoint_index < len(self.waypoints):
                start = self.boat.position.to_tuple()
                end = self.waypoints[self.current_waypoint_index].to_tuple()
                pygame.draw.line(self.screen, (100, 255, 100), start, end, 1)
            
            # Linhas entre waypoints restantes
            for i in range(self.current_waypoint_index, len(self.waypoints) - 1):
                start = self.waypoints[i].to_tuple()
                end = self.waypoints[i + 1].to_tuple()
                pygame.draw.line(self.screen, PATH_COLOR, start, end, 2)
    
    def draw_waypoints(self):
        """Desenha os waypoints"""
        for i, wp in enumerate(self.waypoints):
            if i < self.current_waypoint_index:
                color = WAYPOINT_REACHED_COLOR
            else:
                color = WAYPOINT_COLOR
            
            pygame.draw.circle(self.screen, color, wp.to_tuple(), 10)
            pygame.draw.circle(self.screen, (255, 255, 255), wp.to_tuple(), 10, 2)
            
            # Número do waypoint
            text = self.font.render(str(i + 1), True, (0, 0, 0))
            text_rect = text.get_rect(center=wp.to_tuple())
            self.screen.blit(text, text_rect)
    
    def draw_obstacles(self):
        """Desenha os obstáculos"""
        for obstacle in self.obstacles:
            # Sombra
            shadow_pos = (int(obstacle.position.x + 5), int(obstacle.position.y + 5))
            pygame.draw.circle(self.screen, (20, 20, 40), shadow_pos, int(obstacle.radius))
            
            # Obstáculo
            pygame.draw.circle(self.screen, OBSTACLE_COLOR, obstacle.position.to_tuple(), int(obstacle.radius))
            pygame.draw.circle(self.screen, (255, 100, 100), obstacle.position.to_tuple(), int(obstacle.radius), 3)
            
            # Ícone de perigo
            pygame.draw.line(
                self.screen, (255, 255, 255),
                (obstacle.position.x, obstacle.position.y - obstacle.radius/2),
                (obstacle.position.x, obstacle.position.y + obstacle.radius/4),
                3
            )
            pygame.draw.circle(
                self.screen, (255, 255, 255),
                (int(obstacle.position.x), int(obstacle.position.y + obstacle.radius/2)),
                3
            )
    
    def draw_sensor_cone(self):
        """Desenha o cone do sensor do barco"""
        if not self.show_sensors:
            return
        
        # Cria superfície transparente
        sensor_surface = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
        
        # Desenha o cone do sensor
        cone_points = [self.boat.position.to_tuple()]
        
        num_points = 20
        for i in range(num_points + 1):
            angle = self.boat.heading - self.boat.sensor_angle + (2 * self.boat.sensor_angle * i / num_points)
            x = self.boat.position.x + math.cos(angle) * self.boat.sensor_range
            y = self.boat.position.y + math.sin(angle) * self.boat.sensor_range
            cone_points.append((int(x), int(y)))
        
        pygame.draw.polygon(sensor_surface, (255, 255, 0, 40), cone_points)
        pygame.draw.polygon(sensor_surface, (255, 255, 0, 100), cone_points, 2)
        
        self.screen.blit(sensor_surface, (0, 0))
    
    def draw_boat(self):
        """Desenha o BlueBoat"""
        # Sombra
        shadow_points = [(p[0] + 4, p[1] + 4) for p in self.boat.get_polygon_points()]
        pygame.draw.polygon(self.screen, (20, 20, 40), shadow_points)
        
        # Corpo do barco
        points = self.boat.get_polygon_points()
        pygame.draw.polygon(self.screen, BOAT_COLOR, points)
        pygame.draw.polygon(self.screen, (200, 150, 0), points, 2)
        
        # Indicador de direção
        dir_length = 25
        dir_end = (
            int(self.boat.position.x + math.cos(self.boat.heading) * dir_length),
            int(self.boat.position.y + math.sin(self.boat.heading) * dir_length)
        )
        pygame.draw.line(self.screen, (255, 50, 50), self.boat.position.to_tuple(), dir_end, 3)
    
    def draw_ui_panel(self):
        """Desenha o painel de controle"""
        panel_rect = pygame.Rect(self.sim_area.width, 0, 250, WINDOW_HEIGHT)
        pygame.draw.rect(self.screen, UI_BG_COLOR, panel_rect)
        pygame.draw.line(self.screen, (50, 80, 110), (self.sim_area.width, 0), (self.sim_area.width, WINDOW_HEIGHT), 2)
        
        x = self.sim_area.width + 15
        y = 20
        
        # Título
        title = self.title_font.render("BlueBoat", True, BOAT_COLOR)
        self.screen.blit(title, (x, y))
        y += 30
        
        subtitle = self.font.render("Trajectory Simulator", True, TEXT_COLOR)
        self.screen.blit(subtitle, (x, y))
        y += 50
        
        # Status
        pygame.draw.line(self.screen, (50, 80, 110), (x, y), (x + 220, y), 1)
        y += 15
        
        status_title = self.font.render("STATUS", True, (150, 200, 255))
        self.screen.blit(status_title, (x, y))
        y += 30
        
        # Estado do barco
        state_color = {
            BoatState.IDLE: (150, 150, 150),
            BoatState.NAVIGATING: (100, 255, 100),
            BoatState.AVOIDING: (255, 200, 50),
            BoatState.COMPLETED: (100, 200, 255)
        }
        
        state_text = self.font.render(f"Estado: {self.boat.state.value}", True, state_color[self.boat.state])
        self.screen.blit(state_text, (x, y))
        y += 25
        
        pos_text = self.font.render(f"Pos: ({int(self.boat.position.x)}, {int(self.boat.position.y)})", True, TEXT_COLOR)
        self.screen.blit(pos_text, (x, y))
        y += 25
        
        heading_deg = math.degrees(self.boat.heading) % 360
        heading_text = self.font.render(f"Heading: {heading_deg:.1f}°", True, TEXT_COLOR)
        self.screen.blit(heading_text, (x, y))
        y += 25
        
        speed = self.boat.velocity.magnitude()
        speed_text = self.font.render(f"Velocidade: {speed:.1f} m/s", True, TEXT_COLOR)
        self.screen.blit(speed_text, (x, y))
        y += 40
        
        # Missão
        pygame.draw.line(self.screen, (50, 80, 110), (x, y), (x + 220, y), 1)
        y += 15
        
        mission_title = self.font.render("MISSÃO", True, (150, 200, 255))
        self.screen.blit(mission_title, (x, y))
        y += 30
        
        wp_text = self.font.render(f"Waypoints: {len(self.waypoints)}", True, TEXT_COLOR)
        self.screen.blit(wp_text, (x, y))
        y += 25
        
        current_text = self.font.render(f"Atual: {self.current_waypoint_index + 1}/{len(self.waypoints)}" if self.waypoints else "Atual: -", True, TEXT_COLOR)
        self.screen.blit(current_text, (x, y))
        y += 25
        
        obs_text = self.font.render(f"Obstáculos: {len(self.obstacles)}", True, TEXT_COLOR)
        self.screen.blit(obs_text, (x, y))
        y += 40
        
        # Controles
        pygame.draw.line(self.screen, (50, 80, 110), (x, y), (x + 220, y), 1)
        y += 15
        
        controls_title = self.font.render("CONTROLES", True, (150, 200, 255))
        self.screen.blit(controls_title, (x, y))
        y += 30
        
        # Modo atual
        mode_color = WAYPOINT_COLOR if self.mode == "waypoint" else OBSTACLE_COLOR
        mode_name = "Waypoint" if self.mode == "waypoint" else "Obstáculo"
        mode_text = self.font.render(f"Modo: {mode_name}", True, mode_color)
        self.screen.blit(mode_text, (x, y))
        y += 35
        
        controls = [
            "Click: Adicionar",
            "M: Trocar modo",
            "SPACE: Pausar",
            "R: Reiniciar",
            "C: Limpar waypoints",
            "O: Limpar obstáculos",
            "G: Toggle grade",
            "S: Toggle sensores",
            "ESC: Sair"
        ]
        
        for control in controls:
            ctrl_text = self.font.render(control, True, (180, 180, 180))
            self.screen.blit(ctrl_text, (x, y))
            y += 22
        
        # Status de pausa
        if self.paused:
            y += 20
            pause_text = self.title_font.render("PAUSADO", True, (255, 100, 100))
            self.screen.blit(pause_text, (x + 50, y))
    
    def handle_events(self):
        """Processa eventos de entrada"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_m:
                    self.mode = "obstacle" if self.mode == "waypoint" else "waypoint"
                elif event.key == pygame.K_r:
                    self.reset()
                elif event.key == pygame.K_c:
                    self.waypoints.clear()
                    self.current_waypoint_index = 0
                elif event.key == pygame.K_o:
                    self.obstacles.clear()
                elif event.key == pygame.K_g:
                    self.show_grid = not self.show_grid
                elif event.key == pygame.K_s:
                    self.show_sensors = not self.show_sensors
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Click esquerdo
                    pos = pygame.mouse.get_pos()
                    if pos[0] < self.sim_area.width:  # Dentro da área de simulação
                        if self.mode == "waypoint":
                            self.waypoints.append(Vector2(pos[0], pos[1]))
                        else:
                            radius = random.randint(25, 45)
                            self.obstacles.append(Obstacle(Vector2(pos[0], pos[1]), radius))
                
                elif event.button == 3:  # Click direito - remove item mais próximo
                    pos = pygame.mouse.get_pos()
                    if self.mode == "waypoint" and self.waypoints:
                        closest_idx = min(range(len(self.waypoints)), 
                                         key=lambda i: (self.waypoints[i] - Vector2(pos[0], pos[1])).magnitude())
                        if (self.waypoints[closest_idx] - Vector2(pos[0], pos[1])).magnitude() < 30:
                            self.waypoints.pop(closest_idx)
                            if self.current_waypoint_index > closest_idx:
                                self.current_waypoint_index -= 1
                    elif self.mode == "obstacle" and self.obstacles:
                        closest_idx = min(range(len(self.obstacles)),
                                         key=lambda i: (self.obstacles[i].position - Vector2(pos[0], pos[1])).magnitude())
                        if (self.obstacles[closest_idx].position - Vector2(pos[0], pos[1])).magnitude() < self.obstacles[closest_idx].radius + 20:
                            self.obstacles.pop(closest_idx)
    
    def reset(self):
        """Reinicia a simulação"""
        self.boat = BlueBoat(100, WINDOW_HEIGHT // 2)
        self.current_waypoint_index = 0
    
    def update(self, dt: float):
        """Atualiza a simulação"""
        if self.paused:
            return
        
        # Determina o waypoint alvo
        target = None
        if self.current_waypoint_index < len(self.waypoints):
            target = self.waypoints[self.current_waypoint_index]
        
        # Atualiza o barco
        if target:
            reached = self.boat.update(target, self.obstacles, dt)
            if reached:
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.waypoints):
                    self.boat.state = BoatState.COMPLETED
        else:
            self.boat.update(None, self.obstacles, dt)
    
    def render(self):
        """Renderiza a cena"""
        self.draw_water_effect()
        self.draw_grid()
        self.draw_trajectory()
        self.draw_planned_path()
        self.draw_waypoints()
        self.draw_obstacles()
        self.draw_sensor_cone()
        self.draw_boat()
        self.draw_ui_panel()
        
        pygame.display.flip()
    
    def run(self):
        """Loop principal"""
        print("=" * 50)
        print("BlueBoat Trajectory Simulator")
        print("=" * 50)
        print("\nControles:")
        print("  Click Esquerdo: Adicionar waypoint/obstáculo")
        print("  Click Direito: Remover item próximo")
        print("  M: Alternar modo (waypoint/obstáculo)")
        print("  SPACE: Pausar/Continuar")
        print("  R: Reiniciar simulação")
        print("  C: Limpar waypoints")
        print("  O: Limpar obstáculos")
        print("  G: Mostrar/ocultar grade")
        print("  S: Mostrar/ocultar sensores")
        print("  ESC: Sair")
        print("=" * 50)
        
        while self.running:
            dt = self.clock.tick(FPS) / 1000.0
            
            self.handle_events()
            self.update(dt)
            self.render()
        
        pygame.quit()


if __name__ == "__main__":
    simulator = Simulator()
    simulator.run()
