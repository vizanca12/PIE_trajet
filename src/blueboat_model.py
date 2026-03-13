import math
from dataclasses import dataclass
from typing import List, Tuple, Optional
from enum import Enum

# --- STRUCTURES DE DONNÉES ---

@dataclass
class Vector2:
    x: float
    y: float
    
    def __add__(self, other): return Vector2(self.x + other.x, self.y + other.y)
    def __sub__(self, other): return Vector2(self.x - other.x, self.y - other.y)
    def __mul__(self, scalar): return Vector2(self.x * scalar, self.y * scalar)
    def magnitude(self): return math.sqrt(self.x**2 + self.y**2)
    def normalize(self):
        mag = self.magnitude()
        return Vector2(self.x / mag, self.y / mag) if mag > 0 else Vector2(0, 0)
    def to_tuple(self): return (int(self.x), int(self.y))
    def dist(self, other): return (self - other).magnitude()
    def __repr__(self): return f"({self.x:.1f}, {self.y:.1f})"

class BoatState(Enum):
    IDLE = "Aguardando"
    APPROACHING = "Buscando Entrada"
    TRACKING = "Seguindo Rota"
    AVOIDING = "Desviando"
    COMPLETED = "Missão Concluída"

@dataclass
class Obstacle:
    position: Vector2
    radius: float
    is_buoy: bool = False
    velocity: Optional[Vector2] = None

    def is_moving(self) -> bool:
        return self.velocity is not None and self.velocity.magnitude() > 0

    def step(self, dt: float):
        if self.is_moving():
            self.position = self.position + (self.velocity * dt)

# --- GERADOR DE ROTA ---

def generate_lemniscate_points(p1: Vector2, p2: Vector2, num_points=120, margin=80) -> List[Vector2]:
    """Génère la géométrie du 8"""
    points = []
    midpoint = (p1 + p2) * 0.5
    diff = p2 - p1
    angle = math.atan2(diff.y, diff.x)
    
    width = (diff.magnitude() / 2) + margin
    height = width * 0.6 
    
    for i in range(num_points):
        t = (i / num_points) * 2 * math.pi
        local_x = width * math.cos(t)
        local_y = height * math.sin(2 * t) / 2
        rot_x = local_x * math.cos(angle) - local_y * math.sin(angle)
        rot_y = local_x * math.sin(angle) + local_y * math.cos(angle)
        points.append(Vector2(midpoint.x + rot_x, midpoint.y + rot_y))
        
    return points

# --- LOGIQUE DU BATEAU ---

class BlueBoat:
    def __init__(self, x, y):
        self.position = Vector2(x, y)
        self.velocity = Vector2(0, 0)
        self.heading = 0
        self.collision_radius = 14
        
        self.max_speed = 4.0
        self.turn_rate = 3.5
        self.lookahead_dist = 60 
        
        self.state = BoatState.IDLE
        self.sensor_range = 100
        self.sensor_angle = math.pi / 3
        
        self.active_path: List[Vector2] = []
        self.current_path_index = 0 
        self.has_planned_lap = False
        
        # Contrôle des logs pour ne pas inonder le terminal
        self.log_timer = 0

    def force_replan(self, original_path_template: List[Vector2]):
        """Força o recálculo da rota a partir da posição atual."""
        self.has_planned_lap = False
        self.current_path_index = 0
        self.active_path = []
        self._reorder_path_from_entry(original_path_template)

    def _reorder_path_from_entry(self, original_path: List[Vector2]):
        """Réorganise la route pour commencer au point le plus proche"""
        if not original_path: return

        closest_dist = float('inf')
        entry_idx = 0
        
        for i, p in enumerate(original_path):
            d = self.position.dist(p)
            if d < closest_dist:
                closest_dist = d
                entry_idx = i
        
        part1 = original_path[entry_idx:]
        part2 = original_path[:entry_idx]
        
        # Crée la route fermée
        self.active_path = part1 + part2 + [original_path[entry_idx]]
        
        self.current_path_index = 0
        self.has_planned_lap = True
        
        print(f"\n[PLANEJADOR] Rota calculada!")
        print(f"[PLANEJADOR] Ponto de entrada detectado no índice {entry_idx}.")
        print(f"[PLANEJADOR] Nova rota ativa tem {len(self.active_path)} waypoints.")

    def _find_closest_path_index(self) -> int:
        """Procura o ponto mais próximo perto do progresso atual para evitar saltos na rota."""
        if not self.active_path:
            return 0

        start_idx = max(0, self.current_path_index - 8)
        end_idx = min(len(self.active_path) - 1, self.current_path_index + 24)

        closest_idx = self.current_path_index
        closest_dist = float("inf")

        for index in range(start_idx, end_idx + 1):
            distance = self.position.dist(self.active_path[index])
            if distance < closest_dist:
                closest_dist = distance
                closest_idx = index

        return closest_idx

    def _advance_target_index(self, start_idx: int) -> int:
        """Avança ao longo da polilinha até atingir a distância de lookahead."""
        if not self.active_path:
            return 0

        travelled = 0.0
        target_idx = start_idx

        while target_idx < len(self.active_path) - 1 and travelled < self.lookahead_dist:
            current_point = self.active_path[target_idx]
            next_point = self.active_path[target_idx + 1]
            travelled += current_point.dist(next_point)
            target_idx += 1

        return target_idx

    def get_target_point(self) -> Vector2:
        """Obtient la cible sur la route active"""
        path = self.active_path
        if not path: return self.position

        self.current_path_index = self._find_closest_path_index()
        lookahead_idx = self._advance_target_index(self.current_path_index)
        target_point = path[lookahead_idx]
        
        return target_point

    def update(self, original_path_template: List[Vector2], obstacles: List[Obstacle], dt: float):
        if not original_path_template: return False

        # 1. Planification (Une seule fois)
        if not self.has_planned_lap:
            self._reorder_path_from_entry(original_path_template)

        # 2. Obtient la cible
        target_pos = self.get_target_point()
        
        # État Visuel
        dist_to_path = self.position.dist(target_pos)
        if dist_to_path > self.lookahead_dist * 2:
            self.state = BoatState.APPROACHING 
        else:
            self.state = BoatState.TRACKING

        # 3. Navigation
        nav_vector = (target_pos - self.position).normalize()

        # 4. Évitement Tangentiel Intelligent
        avoidance_vec = Vector2(0, 0)
        max_avoid_strength = 0.0
        lookahead_seconds = 1.5  # prever onde obstáculos móveis estarão

        for obs in obstacles:
            # Para obstáculos móveis, usar posição futura prevista
            if obs.is_moving() and obs.velocity is not None:
                predicted_pos = obs.position + (obs.velocity * lookahead_seconds)
            else:
                predicted_pos = obs.position

            to_obs = predicted_pos - self.position
            dist = to_obs.magnitude() - obs.radius

            if dist < self.sensor_range:
                to_obs_norm = to_obs.normalize()
                # Cone ampliado: detecta obstáculos frontais E laterais (-0.2 ≈ 100°)
                is_threatening = (to_obs_norm.x * nav_vector.x + to_obs_norm.y * nav_vector.y) > -0.2

                if is_threatening:
                    strength = (self.sensor_range - dist) / self.sensor_range
                    max_avoid_strength = max(max_avoid_strength, strength)

                    tangent_left = Vector2(-to_obs_norm.y, to_obs_norm.x)
                    tangent_right = Vector2(to_obs_norm.y, -to_obs_norm.x)

                    dot_left = tangent_left.x * nav_vector.x + tangent_left.y * nav_vector.y
                    dot_right = tangent_right.x * nav_vector.x + tangent_right.y * nav_vector.y

                    best_escape = tangent_left if dot_left > dot_right else tangent_right
                    # Força de desvio maior quando muito próximo
                    push_strength = 4.5 if dist < self.sensor_range * 0.35 else 2.8
                    avoidance_vec = avoidance_vec + (best_escape * strength * push_strength)
                    self.state = BoatState.AVOIDING

                    self.log_timer += 1
                    if self.log_timer > 30:
                        print(f"[SENSOR] Desviando de obstáculo! Distância: {dist:.1f}px")
                        self.log_timer = 0

        # 5. Vecteur Final
        if avoidance_vec.magnitude() > 0:
            # Quanto mais próximo, maior a prioridade do desvio sobre a rota
            blend_factor = min(0.92, 0.55 + max_avoid_strength * 0.42)
            final_dir = (nav_vector * (1.0 - blend_factor) + avoidance_vec * blend_factor).normalize()
        else:
            final_dir = nav_vector

        # 6. Physique
        desired_heading = math.atan2(final_dir.y, final_dir.x)
        angle_diff = desired_heading - self.heading
        while angle_diff > math.pi: angle_diff -= 2*math.pi
        while angle_diff < -math.pi: angle_diff += 2*math.pi
        
        turn_step = self.turn_rate * dt
        if abs(angle_diff) > turn_step:
            angle_diff = turn_step if angle_diff > 0 else -turn_step
        self.heading += angle_diff
        
        # Velocidade mínima de 30% para o barco não parar ao desviar
        raw_speed = self.max_speed * (1.0 - abs(angle_diff)/math.pi)
        speed = max(self.max_speed * 0.30, raw_speed)
        self.velocity = Vector2(math.cos(self.heading), math.sin(self.heading)) * speed
        self.position = self.position + (self.velocity * dt * 20)
        
        # --- LOGIQUE DE FIN DE MISSION ---
        # Ne se termine que si:
        # 1. L'indice de la route a atteint la fin (Lookahead s'est arrêté)
        # 2. Et le bateau est physiquement près du dernier point (< 20px)
        
        index_reached_end = self.current_path_index >= len(self.active_path) - 2
        dist_to_finish_line = self.position.dist(self.active_path[-1])
        
        boat_physically_arrived = dist_to_finish_line < 25
        
        return index_reached_end and boat_physically_arrived