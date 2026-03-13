import math
import os
import random
import sys

import pygame

from blueboat_model import BlueBoat, Obstacle, Vector2

WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
FPS = 60

SIMULATION_WIDTH = (-80.0, 80.0)
SIMULATION_HEIGHT = (-80.0, 50.0)
START_ZONE = ((-5.0, 45.0), (5.0, 35.0))
BUOY_METERS = [Vector2(0.0, 0.0), Vector2(10.25, -54.75)]
COURSE_METERS = [
    Vector2(0.0, 40.0),
    Vector2(10.0, 0.0),
    Vector2(0.0, -20.0),
    Vector2(-2.5, -25.0),
    Vector2(-5.0, -40.0),
    Vector2(10.0, -60.0),
    Vector2(25.0, -40.0),
    Vector2(-10.0, -10.0),
    Vector2(0.1, 10.0),
]

MAP_LOCAL_PIXELS_PER_METER = 100.0 / 20.0
MAP_LOCAL_ORIGIN = Vector2(575.0, 417.0)
MAP_IMAGE_PATH = os.path.join(os.path.dirname(__file__), "Simulation", "Images", "Port de monaco 2.png")

BORDER_OFFSET_LEFT = 210
BORDER_OFFSET_TOP = 0
BORDER_OFFSET_RIGHT = 0
BORDER_OFFSET_BOTTOM = 0

COLOR_PANEL = (10, 16, 25, 195)
COLOR_ROUTE_OUTLINE = (0, 160, 220)
COLOR_TRAIL = (55, 185, 235)
COLOR_BOAT = (25, 105, 180)
COLOR_BOAT_ACCENT = (255, 120, 70)
COLOR_TARGET = (60, 255, 220)
COLOR_BUOY = (255, 130, 40)
COLOR_MOVING_OBS = (220, 60, 60)
COLOR_FIXED_OBS = (245, 190, 70)
COLOR_BORDER = (220, 40, 40)
COLOR_START = (80, 220, 120)
COLOR_TEXT = (240, 245, 250)
COLOR_MUTED = (190, 205, 215)


class ScenarioSimulator:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("BlueBoat - Percurso de Monaco")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 20)
        self.big_font = pygame.font.Font(None, 42)

        self._configure_map_projection()
        self._load_background_image()

        cx, cy = WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2
        self.btn_new = pygame.Rect(cx - 110, cy + 26, 220, 48)
        self.btn_exit = pygame.Rect(cx - 110, cy + 88, 220, 48)
        self.pause = False

        self.reset_scenario()

    def _configure_map_projection(self):
        scale_height = (WINDOW_HEIGHT - BORDER_OFFSET_TOP - BORDER_OFFSET_BOTTOM) // (SIMULATION_HEIGHT[1] - SIMULATION_HEIGHT[0])
        scale_width = (WINDOW_WIDTH - BORDER_OFFSET_LEFT - BORDER_OFFSET_RIGHT) // (SIMULATION_WIDTH[1] - SIMULATION_WIDTH[0])
        self.scale = float(min(scale_height, scale_width))
        self.map_factor = self.scale / MAP_LOCAL_PIXELS_PER_METER

        center_x = (SIMULATION_WIDTH[1] + SIMULATION_WIDTH[0]) / 2.0
        center_y = (SIMULATION_HEIGHT[1] + SIMULATION_HEIGHT[0]) / 2.0
        self.center_offset = Vector2(
            (WINDOW_WIDTH // 2) - center_x * self.scale + BORDER_OFFSET_LEFT / self.scale,
            -(WINDOW_HEIGHT // 2) + center_y * self.scale + BORDER_OFFSET_BOTTOM,
        )
        self.background_origin = Vector2(
            self.center_offset.x - MAP_LOCAL_ORIGIN.x * self.map_factor,
            WINDOW_HEIGHT + self.center_offset.y - MAP_LOCAL_ORIGIN.y * self.map_factor,
        )

    def _load_background_image(self):
        self.background_image = None
        if not os.path.exists(MAP_IMAGE_PATH):
            return

        image = pygame.image.load(MAP_IMAGE_PATH).convert_alpha()
        original_size = image.get_size()
        scaled_size = (
            int(original_size[0] * self.map_factor),
            int(original_size[1] * self.map_factor),
        )
        self.background_image = pygame.transform.smoothscale(image, scaled_size)

    def _meters_to_screen(self, point_meters):
        return Vector2(
            self.center_offset.x + point_meters.x * self.scale,
            self.center_offset.y + WINDOW_HEIGHT - point_meters.y * self.scale,
        )

    def _densify_polyline(self, points, spacing=10.0):
        if len(points) < 2:
            return points[:]

        dense = [points[0]]
        for start, end in zip(points, points[1:]):
            segment = end - start
            distance = segment.magnitude()
            steps = max(1, int(distance / spacing))
            for step in range(1, steps + 1):
                ratio = step / steps
                dense.append(start + (segment * ratio))
        return dense

    def _catmull_rom_point(self, p0, p1, p2, p3, t):
        t2 = t * t
        t3 = t2 * t
        term1 = p1 * 2.0
        term2 = (p2 - p0) * t
        term3 = (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * t2
        term4 = ((p0 * -1.0) + p1 * 3.0 - p2 * 3.0 + p3) * t3
        return (term1 + term2 + term3 + term4) * 0.5

    def _build_smooth_course(self, control_points, samples_per_segment=18):
        if len(control_points) < 2:
            return control_points[:]

        if len(control_points) == 2:
            return self._densify_polyline(control_points, spacing=10.0)

        extended = [control_points[0]] + control_points + [control_points[-1]]
        smooth_points = [control_points[0]]

        for index in range(1, len(extended) - 2):
            p0 = extended[index - 1]
            p1 = extended[index]
            p2 = extended[index + 1]
            p3 = extended[index + 2]

            for sample in range(1, samples_per_segment + 1):
                t = sample / samples_per_segment
                smooth_points.append(self._catmull_rom_point(p0, p1, p2, p3, t))

        return self._densify_polyline(smooth_points, spacing=9.0)

    def _build_course(self):
        screen_points = [self._meters_to_screen(point) for point in COURSE_METERS]
        return self._build_smooth_course(screen_points, samples_per_segment=16)

    def _random_spawn_in_start_zone(self):
        min_x, max_y = START_ZONE[0]
        max_x, min_y = START_ZONE[1]
        spawn_m = Vector2(
            random.uniform(min_x + 0.5, max_x - 0.5),
            random.uniform(min_y + 0.5, max_y - 0.5),
        )
        return self._meters_to_screen(spawn_m)

    def _build_dynamic_obstacles(self):
        if len(self.waypoints) < 40:
            return []

        dynamic_items = []
        candidate_indices = list(range(22, len(self.waypoints) - 20, 12))
        random.shuffle(candidate_indices)
        total = min(3, len(candidate_indices))

        for index in range(total):
            base_idx = candidate_indices[index]
            base = self.waypoints[base_idx]
            lateral_offset = Vector2(random.uniform(-12, 12), random.uniform(-12, 12))
            position = base + lateral_offset

            is_moving = index == 0 or (index == 1 and random.random() > 0.45)
            speed = random.uniform(7.0, 13.0) if is_moving else 0.0
            patrol_target_idx = min(len(self.waypoints) - 1, base_idx + random.randint(10, 24))
            initial_direction = (self.waypoints[patrol_target_idx] - position).normalize()
            velocity = initial_direction * speed if is_moving else None

            dynamic_items.append(
                {
                    "obstacle": Obstacle(position, random.randint(13, 18), False, velocity),
                    "active": False,
                    "revealed": False,
                    "spawn_time": random.uniform(1.5, 4.0),
                    "trigger_distance": random.uniform(90.0, 145.0),
                    "surprise_label": "móvel" if is_moving else "fixo",
                    "patrol_target_idx": patrol_target_idx,
                    "speed": speed,
                    "turn_rate": random.uniform(1.8, 3.0),
                }
            )

        return dynamic_items

    def _update_surprise_obstacles(self, dt):
        self.elapsed_time += dt

        for item in self.dynamic_obstacles:
            obs = item["obstacle"]

            if not item["active"]:
                time_trigger = self.elapsed_time >= item["spawn_time"]
                distance_trigger = self.boat.position.dist(obs.position) <= item["trigger_distance"]
                if time_trigger or distance_trigger:
                    item["active"] = True
                    if not item["revealed"]:
                        item["revealed"] = True
                        print(f"[SURPRESA] Obstáculo {item['surprise_label']} ativado em {obs.position}")

            if item["active"] and obs.is_moving():
                target = self.waypoints[item["patrol_target_idx"]]
                to_target = target - obs.position
                if to_target.magnitude() < 16:
                    item["patrol_target_idx"] = min(len(self.waypoints) - 1, item["patrol_target_idx"] + random.randint(8, 16))
                    target = self.waypoints[item["patrol_target_idx"]]
                    to_target = target - obs.position

                desired_dir = to_target.normalize()
                if obs.velocity is None or obs.velocity.magnitude() < 0.5:
                    obs.velocity = desired_dir * item["speed"]
                else:
                    current_dir = obs.velocity.normalize()
                    blend = min(1.0, item["turn_rate"] * dt)
                    smoothed = (current_dir * (1.0 - blend) + desired_dir * blend).normalize()
                    obs.velocity = smoothed * item["speed"]

                obs.step(dt)

    def _get_active_obstacles(self):
        active_dynamic = [item["obstacle"] for item in self.dynamic_obstacles if item["active"]]
        return self.obstacles + active_dynamic

    def _detect_collision(self, obstacles):
        if self.recovery_cooldown > 0:
            return None

        for obs in obstacles:
            if self.boat.position.dist(obs.position) <= (self.boat.collision_radius + obs.radius):
                return obs
        return None

    def _handle_avoidance_failure(self, hit_obstacle):
        self.failure_flash_timer = 2.4
        self.failure_message = "Colisão! Desviando..."
        self.failure_count += 1
        # Cooldown maior para o barco sair da zona antes de checar de novo
        self.recovery_cooldown = 2.8

        escape = (self.boat.position - hit_obstacle.position).normalize()
        if escape.magnitude() == 0:
            escape = Vector2(math.cos(self.boat.heading), math.sin(self.boat.heading))
        # Empurrar bem para fora do raio do obstáculo
        push_distance = hit_obstacle.radius + self.boat.collision_radius + 30
        self.boat.position = self.boat.position + (escape * push_distance)
        # Dar velocidade na direção de escape para o barco já sair se movendo
        self.boat.velocity = escape * self.boat.max_speed * 0.6
        # NÃO replanejar aqui — deixa o seguimento natural retomar a rota
        # (replan dentro da colisão causava loop: novo plano passava pelo obstáculo)
        print(f"[FALHA] Colisão em {hit_obstacle.position}. Empurrando barco para fora.")

    def reset_scenario(self):
        self.state = "RUNNING"
        self.pause = False
        self.elapsed_time = 0.0
        self.failure_flash_timer = 0.0
        self.failure_message = ""
        self.failure_count = 0
        self.recovery_cooldown = 0.0
        self.boat_trail = []

        print("\n" + "=" * 50)
        print("SCÉNARIO - PARCOURS DE MONACO")
        print("=" * 50)

        self.waypoints = self._build_course()
        self.obstacles = [Obstacle(self._meters_to_screen(point), 14, True) for point in BUOY_METERS]
        self.dynamic_obstacles = self._build_dynamic_obstacles()

        spawn_pos = self._random_spawn_in_start_zone()
        self.boat = BlueBoat(spawn_pos.x, spawn_pos.y)
        self.boat.max_speed = 2.35
        self.boat.turn_rate = 2.7
        self.boat.lookahead_dist = 62
        self.boat.sensor_range = 92

        if len(self.waypoints) > 5:
            heading_vec = (self.waypoints[5] - self.waypoints[0]).normalize()
            self.boat.heading = math.atan2(heading_vec.y, heading_vec.x)

        print(f"[SCÉNARIO] {len(self.waypoints)} points de route chargés sur la carte.")
        print(f"[SCÉNARIO] {len(self.dynamic_obstacles)} obstacles surprises prêts sur le parcours.")
        print(f"[SCÉNARIO] Bateau placé dans la zone de départ en {spawn_pos}")

    def _draw_background(self):
        self.screen.fill((235, 238, 242))
        if self.background_image is not None:
            self.screen.blit(self.background_image, self.background_origin.to_tuple())

        top_left = self._meters_to_screen(Vector2(SIMULATION_WIDTH[0], SIMULATION_HEIGHT[1]))
        bottom_right = self._meters_to_screen(Vector2(SIMULATION_WIDTH[1], SIMULATION_HEIGHT[0]))
        border_rect = pygame.Rect(
            int(top_left.x),
            int(top_left.y),
            int(bottom_right.x - top_left.x),
            int(bottom_right.y - top_left.y),
        )
        pygame.draw.rect(self.screen, COLOR_BORDER, border_rect, 2)

        start_top_left = self._meters_to_screen(Vector2(START_ZONE[0][0], START_ZONE[0][1]))
        start_bottom_right = self._meters_to_screen(Vector2(START_ZONE[1][0], START_ZONE[1][1]))
        start_rect = pygame.Rect(
            int(start_top_left.x),
            int(start_top_left.y),
            int(start_bottom_right.x - start_top_left.x),
            int(start_bottom_right.y - start_top_left.y),
        )
        zone_surface = pygame.Surface((start_rect.width, start_rect.height), pygame.SRCALPHA)
        zone_surface.fill((60, 190, 100, 80))
        self.screen.blit(zone_surface, start_rect.topleft)
        pygame.draw.rect(self.screen, COLOR_START, start_rect, 2)

    def _draw_route(self):
        if len(self.waypoints) < 2:
            return

        route_surface = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
        route_points = [point.to_tuple() for point in self.waypoints]
        pygame.draw.lines(route_surface, (255, 255, 255, 165), False, route_points, 8)
        pygame.draw.lines(route_surface, (*COLOR_ROUTE_OUTLINE, 220), False, route_points, 3)
        for idx in range(0, len(route_points), 14):
            pygame.draw.circle(route_surface, (255, 255, 255, 215), route_points[idx], 2)
        self.screen.blit(route_surface, (0, 0))

    def _draw_boat_trail(self):
        if len(self.boat_trail) > 1:
            pygame.draw.lines(self.screen, COLOR_TRAIL, False, self.boat_trail, 3)
            pygame.draw.lines(self.screen, (255, 255, 255), False, self.boat_trail, 1)

    def _draw_obstacle(self, obs, color, label=None):
        pos = obs.position.to_tuple()
        pygame.draw.circle(self.screen, color, pos, int(obs.radius))
        pygame.draw.circle(self.screen, (25, 25, 30), pos, int(obs.radius), 2)
        pygame.draw.circle(self.screen, (255, 255, 255), pos, 3)

        if obs.is_moving() and obs.velocity is not None:
            end = obs.position + obs.velocity.normalize() * 22
            pygame.draw.line(self.screen, (255, 255, 255), pos, end.to_tuple(), 2)

        if label is not None:
            text = self.small_font.render(label, True, (15, 15, 20))
            self.screen.blit(text, (pos[0] + 10, pos[1] - 10))

    def _draw_hud(self, progress):
        hud_rect = pygame.Rect(18, 18, 290, 152)
        legend_rect = pygame.Rect(18, 184, 290, 116)

        hud_surface = pygame.Surface((hud_rect.width, hud_rect.height), pygame.SRCALPHA)
        hud_surface.fill(COLOR_PANEL)
        self.screen.blit(hud_surface, hud_rect.topleft)

        legend_surface = pygame.Surface((legend_rect.width, legend_rect.height), pygame.SRCALPHA)
        legend_surface.fill(COLOR_PANEL)
        self.screen.blit(legend_surface, legend_rect.topleft)

        speed = self.boat.velocity.magnitude() * 20.0
        active_dynamic = len([item for item in self.dynamic_obstacles if item["active"]])
        lines = [
            "Parcours de Monaco",
            f"Estado: {self.boat.state.value}",
            f"Progresso: {progress}%",
            f"Velocidade: {speed:.1f} px/s",
            f"Falhas: {self.failure_count}",
            f"Obstáculos ativos: {active_dynamic}",
            f"Tempo: {self.elapsed_time:05.1f}s",
            "R: reiniciar | SPACE: pausar",
        ]

        y = hud_rect.y + 14
        for idx, line in enumerate(lines):
            color = COLOR_TEXT if idx == 0 else COLOR_MUTED
            text = self.font.render(line, True, color)
            self.screen.blit(text, (hud_rect.x + 14, y))
            y += 18 if idx == 0 else 17

        legend_items = [
            (COLOR_ROUTE_OUTLINE, "Rota original"),
            (COLOR_TRAIL, "Rastro do barco"),
            (COLOR_BUOY, "Bóias reais"),
            (COLOR_MOVING_OBS, "Obstáculo móvel"),
            (COLOR_FIXED_OBS, "Obstáculo fixo"),
        ]
        y = legend_rect.y + 16
        title = self.font.render("Legenda", True, COLOR_TEXT)
        self.screen.blit(title, (legend_rect.x + 14, y))
        y += 24
        for color, label in legend_items:
            pygame.draw.circle(self.screen, color, (legend_rect.x + 24, y + 8), 7)
            text = self.small_font.render(label, True, COLOR_MUTED)
            self.screen.blit(text, (legend_rect.x + 40, y))
            y += 18

    def _draw_failure_banner(self):
        if self.failure_flash_timer <= 0:
            return

        alpha = 145 + int(55 * math.sin(self.elapsed_time * 7.0))
        banner = pygame.Surface((WINDOW_WIDTH - 140, 54), pygame.SRCALPHA)
        banner.fill((150, 15, 15, alpha))
        position = (70, WINDOW_HEIGHT - 78)
        self.screen.blit(banner, position)

        title = self.big_font.render("FALHA DE DESVIO", True, (255, 245, 245))
        subtitle = self.font.render(self.failure_message, True, (255, 225, 225))
        self.screen.blit(title, title.get_rect(center=(WINDOW_WIDTH // 2, WINDOW_HEIGHT - 56)))
        self.screen.blit(subtitle, subtitle.get_rect(center=(WINDOW_WIDTH // 2, WINDOW_HEIGHT - 28)))

    def draw_boat(self):
        x, y, heading = self.boat.position.x, self.boat.position.y, self.boat.heading
        nose = (x + math.cos(heading) * 22, y + math.sin(heading) * 22)
        left = (x + math.cos(heading + 2.45) * 13, y + math.sin(heading + 2.45) * 13)
        right = (x + math.cos(heading - 2.45) * 13, y + math.sin(heading - 2.45) * 13)
        stern = (x - math.cos(heading) * 12, y - math.sin(heading) * 12)
        hull = [nose, left, stern, right]

        pygame.draw.polygon(self.screen, COLOR_BOAT, hull)
        pygame.draw.polygon(self.screen, (15, 25, 40), hull, 2)
        pygame.draw.circle(self.screen, COLOR_BOAT_ACCENT, (int(nose[0]), int(nose[1])), 4)
        pygame.draw.circle(self.screen, (245, 245, 248), (int(x), int(y)), 4)

        wake_left = (x - math.cos(heading) * 14 + math.cos(heading + 1.5) * 5, y - math.sin(heading) * 14 + math.sin(heading + 1.5) * 5)
        wake_right = (x - math.cos(heading) * 14 + math.cos(heading - 1.5) * 5, y - math.sin(heading) * 14 + math.sin(heading - 1.5) * 5)
        wake_tail = (x - math.cos(heading) * 28, y - math.sin(heading) * 28)
        pygame.draw.line(self.screen, (255, 255, 255), wake_left, wake_tail, 2)
        pygame.draw.line(self.screen, (255, 255, 255), wake_right, wake_tail, 2)

    def draw_finished_overlay(self, mouse_pos):
        overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 95))
        self.screen.blit(overlay, (0, 0))

        title = self.big_font.render("MISSION ACCOMPLIE", True, (235, 255, 235))
        self.screen.blit(title, title.get_rect(center=(WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2 - 52)))

        for button, text in ((self.btn_new, "Rejouer"), (self.btn_exit, "Quitter")):
            color = (45, 145, 85) if button.collidepoint(pygame.mouse.get_pos()) else (30, 50, 65)
            pygame.draw.rect(self.screen, color, button, border_radius=8)
            pygame.draw.rect(self.screen, (220, 235, 245), button, 2, border_radius=8)
            label = self.font.render(text, True, COLOR_TEXT)
            self.screen.blit(label, label.get_rect(center=button.center))

    def run(self):
        while True:
            dt = self.clock.tick(FPS) / 1000.0
            mouse_pos = pygame.mouse.get_pos()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        self.reset_scenario()
                    elif event.key == pygame.K_SPACE and self.state == "RUNNING":
                        self.pause = not self.pause
                if event.type == pygame.MOUSEBUTTONDOWN and self.state == "FINISHED":
                    if self.btn_new.collidepoint(mouse_pos):
                        self.reset_scenario()
                    elif self.btn_exit.collidepoint(mouse_pos):
                        pygame.quit()
                        sys.exit()

            if self.state == "RUNNING" and not self.pause:
                self.recovery_cooldown = max(0.0, self.recovery_cooldown - dt)
                self._update_surprise_obstacles(dt)
                current_obstacles = self._get_active_obstacles()
                finished = self.boat.update(self.waypoints, current_obstacles, dt)

                self.boat_trail.append(self.boat.position.to_tuple())
                if len(self.boat_trail) > 320:
                    self.boat_trail.pop(0)

                hit = self._detect_collision(current_obstacles)
                if hit is not None:
                    self._handle_avoidance_failure(hit)

                if finished:
                    self.state = "FINISHED"
                    print("\n[SUCCÈS] Le bateau est arrivé à la fin du parcours de Monaco!")
                    print("-" * 50)

            self._draw_background()
            self._draw_route()
            self._draw_boat_trail()

            for index, obs in enumerate(self.obstacles, start=1):
                self._draw_obstacle(obs, COLOR_BUOY, label=f"B{index}")

            for item in self.dynamic_obstacles:
                if not item["active"]:
                    continue
                color = COLOR_MOVING_OBS if item["obstacle"].is_moving() else COLOR_FIXED_OBS
                self._draw_obstacle(item["obstacle"], color)

            target = self.boat.get_target_point()
            pygame.draw.line(self.screen, (255, 255, 255), self.boat.position.to_tuple(), target.to_tuple(), 1)
            pygame.draw.circle(self.screen, COLOR_TARGET, target.to_tuple(), 7)
            pygame.draw.circle(self.screen, (20, 30, 35), target.to_tuple(), 7, 2)

            self.draw_boat()

            progress = 0
            if self.boat.active_path:
                progress = int((self.boat.current_path_index / len(self.boat.active_path)) * 100)
            self._draw_hud(progress)

            if self.pause and self.state == "RUNNING":
                pause_text = self.big_font.render("PAUSE", True, COLOR_TEXT)
                self.screen.blit(pause_text, pause_text.get_rect(center=(WINDOW_WIDTH // 2, 50)))

            if self.failure_flash_timer > 0:
                self.failure_flash_timer = max(0.0, self.failure_flash_timer - dt)
                self._draw_failure_banner()

            if self.state == "FINISHED":
                self.draw_finished_overlay(mouse_pos)

            pygame.display.flip()


if __name__ == "__main__":
    ScenarioSimulator().run()