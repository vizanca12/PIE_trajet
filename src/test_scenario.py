import pygame
import random
import math
import sys
from blueboat_model import BlueBoat, Obstacle, Vector2, generate_lemniscate_points

# --- CONFIGURATIONS ---
WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 700
FPS = 60

# Couleurs
COLOR_BG = (15, 25, 40)
COLOR_PATH = (255, 255, 255, 30)
COLOR_TARGET = (0, 255, 255)
COLOR_BOAT = (255, 200, 50)
COLOR_BUOY = (255, 80, 0)
COLOR_UI_BG = (0, 0, 0, 200)

class ScenarioSimulator:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("BlueBoat - Teste de Chegada Sincronizada")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)
        self.big_font = pygame.font.Font(None, 50)
        
        cx, cy = WINDOW_WIDTH//2, WINDOW_HEIGHT//2
        self.btn_new = pygame.Rect(cx-100, cy+20, 200, 50)
        self.btn_exit = pygame.Rect(cx-100, cy+90, 200, 50)
        
        self.reset_scenario()
        
    def reset_scenario(self):
        self.state = "RUNNING"
        self.boat = None
        self.waypoints = []
        self.obstacles = []
        
        print("\n" + "="*50)
        print("GÉNÉRATION D'UN NOUVEAU SCÉNARIO DE TEST")
        print("="*50)
        
        # 1. Bouées
        margin = 180
        b1 = Vector2(random.randint(margin, WINDOW_WIDTH-margin), 
                     random.randint(margin, WINDOW_HEIGHT-margin))
        
        angle = random.uniform(0, 6.28)
        dist = random.uniform(250, 450)
        b2 = b1 + Vector2(math.cos(angle)*dist, math.sin(angle)*dist)
        
        b2.x = max(100, min(WINDOW_WIDTH-100, b2.x))
        b2.y = max(100, min(WINDOW_HEIGHT-100, b2.y))
        
        self.obstacles = [Obstacle(b1, 15, True), Obstacle(b2, 15, True)]
        print(f"[SCÉNARIO] Bouée 1 positionnée en {b1}")
        print(f"[SCÉNARIO] Bouée 2 positionnée en {b2}")
        
        # 2. Route
        self.waypoints = generate_lemniscate_points(b1, b2)
        
        # 3. Bateau
        spawn_type = random.choice(["corner", "random"])
        if spawn_type == "corner":
            spawn_pos = Vector2(random.choice([50, WINDOW_WIDTH-50]), 
                                random.choice([50, WINDOW_HEIGHT-50]))
        else:
            spawn_pos = Vector2(random.randint(50, WINDOW_WIDTH-50),
                                random.randint(50, WINDOW_HEIGHT-50))
            
        self.boat = BlueBoat(spawn_pos.x, spawn_pos.y)
        self.boat.heading = random.uniform(0, 6.28)
        
        print(f"[SCÉNARIO] Bateau lancé en {spawn_pos} (En dehors de la route)")
        print(f"[STATUS] En attente que le bateau trouve la route...")

    def run(self):
        while True:
            dt = self.clock.tick(FPS) / 1000.0
            mouse_pos = pygame.mouse.get_pos()
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit(); sys.exit()
                if event.type == pygame.MOUSEBUTTONDOWN and self.state == "FINISHED":
                    if self.btn_new.collidepoint(mouse_pos): self.reset_scenario()
                    if self.btn_exit.collidepoint(mouse_pos): pygame.quit(); sys.exit()
                if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                    self.reset_scenario()

            if self.state == "RUNNING":
                finished = self.boat.update(self.waypoints, self.obstacles, dt)
                if finished: 
                    self.state = "FINISHED"
                    print("\n[SUCCÈS] Le bateau est arrivé physiquement à la destination finale!")
                    print("-" * 50)

            # --- DESSIN ---
            self.screen.fill(COLOR_BG)
            
            # Route Modèle
            if len(self.waypoints) > 1:
                pts = [p.to_tuple() for p in self.waypoints]
                pygame.draw.lines(self.screen, COLOR_PATH, False, pts, 2)
            
            # Route Active (Que le bateau suit réellement) - Debug en Vert
            if self.boat.active_path:
                active_pts = [p.to_tuple() for p in self.boat.active_path]
                if len(active_pts) > 1:
                    pygame.draw.lines(self.screen, (0, 255, 0), False, active_pts, 1)
                
                # Dessine le Point Final Réel
                last_pt = self.boat.active_path[-1].to_tuple()
                pygame.draw.circle(self.screen, (255, 0, 0), last_pt, 6) # Point Rouge = ARRIVÉE

            # Obstacles
            for obs in self.obstacles:
                pygame.draw.circle(self.screen, COLOR_BUOY, obs.position.to_tuple(), int(obs.radius))
                pygame.draw.circle(self.screen, (255, 255, 255), obs.position.to_tuple(), int(obs.radius), 2)

            # Visée
            target = self.boat.get_target_point()
            pygame.draw.line(self.screen, (255, 255, 255, 50), self.boat.position.to_tuple(), target.to_tuple(), 1)
            pygame.draw.circle(self.screen, COLOR_TARGET, target.to_tuple(), 4)

            self.draw_boat()
            
            # HUD
            status = self.font.render(f"Estado: {self.boat.state.value}", True, (200, 200, 200))
            
            prog_val = 0
            if self.boat.active_path:
                prog_val = int((self.boat.current_path_index / len(self.boat.active_path))*100)
            
            # Affiche si le bateau attend d'arriver
            if prog_val >= 98 and self.state == "RUNNING":
                 prog = self.font.render(f"Progression: {prog_val}% (En attente d'arrivée physique...)", True, (255, 200, 50))
            else:
                 prog = self.font.render(f"Progression: {prog_val}%", True, (200, 200, 200))

            self.screen.blit(status, (20, 20))
            self.screen.blit(prog, (20, 45))

            if self.state == "FINISHED":
                self.draw_gameover(mouse_pos)

            pygame.display.flip()

    def draw_boat(self):
        length = 30
        x, y, h = self.boat.position.x, self.boat.position.y, self.boat.heading
        tip = (x + math.cos(h)*length, y + math.sin(h)*length)
        l = (x + math.cos(h+2.6)*20, y + math.sin(h+2.6)*20)
        r = (x + math.cos(h-2.6)*20, y + math.sin(h-2.6)*20)
        pygame.draw.polygon(self.screen, COLOR_BOAT, [tip, l, r])

    def draw_gameover(self, mouse):
        overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
        overlay.fill(COLOR_UI_BG)
        self.screen.blit(overlay, (0,0))
        
        txt = self.big_font.render("MISSION ACCOMPLIE", True, (100, 255, 100))
        self.screen.blit(txt, txt.get_rect(center=(WINDOW_WIDTH//2, WINDOW_HEIGHT//2 - 60)))
        
        for btn, text in [(self.btn_new, "Nouvelle Route"), (self.btn_exit, "Quitter")]:
            col = (60, 180, 100) if btn.collidepoint(mouse) else (50, 50, 50)
            pygame.draw.rect(self.screen, col, btn, border_radius=8)
            t = self.font.render(text, True, (255, 255, 255))
            self.screen.blit(t, t.get_rect(center=btn.center))

if __name__ == "__main__":
    ScenarioSimulator().run()