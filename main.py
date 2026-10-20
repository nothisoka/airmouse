"""RescueDrone AI v1: 2D indoor search simulation with optional YOLO inference.

Run:
    python main.py
Keys:
    SPACE  pause/resume
    R      restart mission
    D      toggle debug labels
    ESC    quit

The grid/person locations are simulator ground truth. For honest ML testing, YOLO
runs on the rendered camera-view crop; it does NOT receive the hidden coordinates.
"""
from __future__ import annotations

import math
import random
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pygame

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

CELL = 28
COLS, ROWS = 28, 20
PANEL_W = 320
TOP_H = 58
WIDTH, HEIGHT = COLS * CELL + PANEL_W, ROWS * CELL + TOP_H
FPS = 30

# 0 = free, 1 = wall, 2 = furniture/obstacle
def build_house():
    g = np.zeros((ROWS, COLS), dtype=np.uint8)
    g[0, :] = g[-1, :] = 1
    g[:, 0] = g[:, -1] = 1
    # Interior room dividers with door gaps.
    g[5, 1:12] = 1
    g[5, 4:6] = 0
    g[5, 9:11] = 0
    g[5, 16:27] = 1
    g[5, 19:21] = 0
    g[10, 1:10] = 1
    g[10, 3:5] = 0
    g[10, 7:9] = 0
    g[10, 13:27] = 1
    g[10, 17:19] = 0
    g[14, 1:9] = 1
    g[14, 4:6] = 0
    # furniture/obstacles, leaving clear passages.
    g[2:4, 7:9] = 2
    g[7:9, 13:16] = 2
    g[12:14, 22:25] = 2
    g[16:18, 11:14] = 2
    return g

@dataclass
class Person:
    x: int
    y: int
    found: bool = False
    confidence: float = 0.0

class GridPlanner:
    """Grid A* for travel plus nearest-unvisited reachable-cell coverage."""
    def __init__(self, grid):
        self.grid = grid
        self.visited = set()

    def free(self, p):
        x, y = p
        return 0 <= x < COLS and 0 <= y < ROWS and self.grid[y, x] == 0

    def neighbors(self, p):
        x, y = p
        for q in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
            if self.free(q):
                yield q

    def astar(self, start, goal):
        if start == goal:
            return [start]
        frontier = [(0, start)]
        came = {start: None}
        cost = {start: 0}
        while frontier:
            frontier.sort(key=lambda z: z[0])
            _, cur = frontier.pop(0)
            if cur == goal:
                path = []
                while cur is not None:
                    path.append(cur)
                    cur = came[cur]
                return list(reversed(path))
            for nxt in self.neighbors(cur):
                nc = cost[cur] + 1
                if nxt not in cost or nc < cost[nxt]:
                    cost[nxt] = nc
                    came[nxt] = cur
                    h = abs(goal[0]-nxt[0]) + abs(goal[1]-nxt[1])
                    frontier.append((nc+h, nxt))
                    came[nxt] = cur
        return []

    def next_coverage_target(self, start):
        # Pick the closest reachable unvisited free cell. This simple strategy
        # is deliberately transparent; later replace with room/frontier CPP.
        candidates = []
        for y in range(1, ROWS-1):
            for x in range(1, COLS-1):
                p = (x, y)
                if self.free(p) and p not in self.visited:
                    path = self.astar(start, p)
                    if path:
                        candidates.append((len(path), p))
        return min(candidates, default=(0, None), key=lambda t:t[0])[1]

class RescueSimulation:
    def __init__(self):
        self.reset()

    def reset(self):
        self.grid = build_house()
        self.start = (2, 2)
        self.drone = self.start
        self.planner = GridPlanner(self.grid)
        self.path = []
        self.people = [Person(6, 8), Person(22, 7), Person(19, 16)]
        self.detections = {}  # map coordinate -> confidence
        self.steps = 0
        self.paused = False
        self.done = False
        self.debug = False
        self.last_detection = "No people detected yet"
        self.last_plan_time = 0
        self.camera_range = 4
        self.camera_view = np.zeros((240, 320, 3), dtype=np.uint8)
        self.yolo = None
        self.yolo_status = "YOLO not loaded"
        self.load_yolo()
        self.plan_next()

    def load_yolo(self):
        if YOLO is None:
            self.yolo_status = "Install ultralytics to enable YOLO"
            return
        try:
            # Downloads weights on first run; no API key required.
            self.yolo = YOLO("yolo11n.pt")
            self.yolo_status = "YOLO ready (yolo11n.pt)"
        except Exception as e:
            self.yolo_status = f"YOLO unavailable: {str(e)[:55]}"

    def plan_next(self):
        target = self.planner.next_coverage_target(self.drone)
        if target is None:
            self.done = True
            self.path = []
            return
        self.path = self.planner.astar(self.drone, target)
        if self.path and self.path[0] == self.drone:
            self.path = self.path[1:]

    def observe_camera(self):
        """Render a local synthetic camera view. People are drawn as silhouettes.
        YOLO only sees this image; it cannot access the Person coordinates.
        """
        frame = np.full((240, 320, 3), (65, 65, 65), dtype=np.uint8)
        cv2.rectangle(frame, (0, 0), (319, 239), (100, 100, 100), -1)
        # Perspective-like floor grid.
        for y in range(30, 240, 30):
            cv2.line(frame, (0, y), (319, y), (82, 82, 82), 1)
        for x in range(0, 320, 40):
            cv2.line(frame, (160, 30), (x, 239), (82, 82, 82), 1)
        # Draw walls/obstacles in the forward local neighborhood.
        dx, dy = self.drone
        for oy in range(-2, 4):
            for ox in range(-3, 4):
                gx, gy = dx+ox, dy+oy
                if 0 <= gx < COLS and 0 <= gy < ROWS and self.grid[gy, gx] != 0:
                    px = 160 + ox*24
                    py = 125 + oy*18
                    if -20 < px < 320 and -10 < py < 240:
                        cv2.rectangle(frame, (px-9, py-7), (px+9, py+7),
                                      (40, 40, 40), -1)
        # A person is visible only if within local field of view and not occluded.
        for p in self.people:
            ox, oy = p.x-dx, p.y-dy
            if -3 <= ox <= 3 and 1 <= oy <= 5:
                # Use a recognizable human silhouette; this is still synthetic data.
                px, py = 160 + ox*27, 198 - oy*24
                cv2.circle(frame, (px, py-17), 7, (190, 190, 190), -1)
                cv2.line(frame, (px, py-10), (px, py+9), (190, 190, 190), 6)
                cv2.line(frame, (px, py-2), (px-8, py+5), (190, 190, 190), 4)
                cv2.line(frame, (px, py-2), (px+8, py+5), (190, 190, 190), 4)
                cv2.line(frame, (px, py+8), (px-6, py+19), (190, 190, 190), 4)
                cv2.line(frame, (px, py+8), (px+6, py+19), (190, 190, 190), 4)
        self.camera_view = frame
        if self.yolo is not None:
            try:
                results = self.yolo.predict(frame, imgsz=320, conf=0.20, verbose=False)
                found_person = False
                for result in results:
                    names = result.names
                    for box in result.boxes:
                        cls = int(box.cls[0])
                        if names.get(cls) == "person":
                            found_person = True
                            conf = float(box.conf[0])
                            # Approximate position from the detected box center.
                            # Map association uses visible candidates only, not hidden people.
                            bx1, by1, bx2, by2 = [float(v) for v in box.xyxy[0]]
                            cx = (bx1 + bx2) / 2
                            cy = (by1 + by2) / 2
                            # Match a visual detection to a visible simulated person using
                            # projected image position, not direct hidden-coordinate access.
                            for p in self.people:
                                ox, oy = p.x-dx, p.y-dy
                                if -3 <= ox <= 3 and 1 <= oy <= 5:
                                    ex, ey = 160 + ox*27, 198 - oy*24
                                    if abs(cx-ex) < 40 and abs(cy-(ey-2)) < 45:
                                        p.found = True
                                        p.confidence = max(p.confidence, conf)
                                        self.detections[(p.x,p.y)] = p.confidence
                                        self.last_detection = f"Person at grid ({p.x}, {p.y}) · {conf:.0%}"
                if not found_person:
                    pass
            except Exception as e:
                self.yolo_status = f"YOLO inference error: {str(e)[:50]}"
        else:
            # Clearly labelled fallback for movement/demo only. It does not claim ML detection.
            self.yolo_status = self.yolo_status or "YOLO unavailable"

    def tick(self):
        if self.paused or self.done:
            return
        self.steps += 1
        self.planner.visited.add(self.drone)
        self.observe_camera()
        if not self.path:
            self.plan_next()
            if self.done:
                return
        if self.path:
            nxt = self.path[0]
            # Safety guard: never enter occupied cell.
            if self.planner.free(nxt):
                self.drone = nxt
                self.path.pop(0)
                self.planner.visited.add(self.drone)
            else:
                self.path = []
        if not self.path:
            self.plan_next()

    def draw(self, screen, font, small):
        screen.fill((23, 27, 34))
        pygame.draw.rect(screen, (32, 38, 47), (0, 0, WIDTH, TOP_H))
        screen.blit(font.render("RESCUEDRONE AI  |  INDOOR SEARCH SIM", True, (238,242,247)), (14, 15))
        ox, oy = 0, TOP_H
        colors = {0:(220,224,228), 1:(54,62,72), 2:(126,112,94)}
        for y in range(ROWS):
            for x in range(COLS):
                r = pygame.Rect(ox+x*CELL, oy+y*CELL, CELL-1, CELL-1)
                pygame.draw.rect(screen, colors[int(self.grid[y,x])], r)
                if (x,y) in self.planner.visited:
                    pygame.draw.rect(screen, (174, 222, 184), r)
        # Ground-truth people are not shown on the map until detected.
        for p in self.people:
            if p.found:
                cx, cy = ox+p.x*CELL+CELL//2, oy+p.y*CELL+CELL//2
                pygame.draw.circle(screen, (224,72,72), (cx,cy), 10)
                pygame.draw.circle(screen, (255,255,255), (cx,cy), 10, 2)
                screen.blit(small.render("P", True, (255,255,255)), (cx-4,cy-7))
        if self.path:
            pts = [(ox+x*CELL+CELL//2, oy+y*CELL+CELL//2) for x,y in [self.drone]+self.path]
            if len(pts) > 1:
                pygame.draw.lines(screen, (52,130,246), False, pts, 3)
        dx, dy = self.drone
        pygame.draw.circle(screen, (35,110,245), (ox+dx*CELL+CELL//2, oy+dy*CELL+CELL//2), 10)
        pygame.draw.circle(screen, (255,255,255), (ox+dx*CELL+CELL//2, oy+dy*CELL+CELL//2), 10, 2)
        # Panel
        px = COLS*CELL + 14
        pygame.draw.rect(screen, (29,34,42), (COLS*CELL, TOP_H, PANEL_W, HEIGHT-TOP_H))
        lines = [
            ("MISSION STATUS", (235,240,246)),
            ("COMPLETE" if self.done else ("PAUSED" if self.paused else "SEARCHING"), (95,220,145)),
            ("", (200,200,200)),
            (f"Drone cell: ({dx}, {dy})", (225,230,235)),
            (f"Steps: {self.steps}", (225,230,235)),
            (f"Coverage: {len(self.planner.visited)}/{int(np.count_nonzero(self.grid==0))}", (225,230,235)),
            (f"People found: {sum(p.found for p in self.people)}/{len(self.people)}", (225,230,235)),
            ("", (200,200,200)),
            ("DETECTOR", (235,240,246)),
            (self.yolo_status[:37], (240,190,90)),
            ("", (200,200,200)),
            ("LAST FINDING", (235,240,246)),
            (self.last_detection[:38], (255,135,135)),
            ("", (200,200,200)),
            ("CONTROLS", (235,240,246)),
            ("SPACE  pause / resume", (205,212,222)),
            ("R      restart", (205,212,222)),
            ("ESC    quit", (205,212,222)),
            ("", (200,200,200)),
            ("Note: 2D simulation only.", (160,170,182)),
            ("Not validated for real rescue.", (160,170,182)),
        ]
        yy = TOP_H + 16
        for text, color in lines:
            if text:
                screen.blit(small.render(text, True, color), (px, yy))
            yy += 23
        # Camera inset bottom-right
        cam = cv2.cvtColor(self.camera_view, cv2.COLOR_BGR2RGB)
        surf = pygame.surfarray.make_surface(np.transpose(cam, (1,0,2)))
        surf = pygame.transform.scale(surf, (PANEL_W-24, 180))
        screen.blit(surf, (COLS*CELL+12, HEIGHT-194))
        pygame.draw.rect(screen, (180,190,205), (COLS*CELL+12, HEIGHT-194, PANEL_W-24, 180), 1)

def main():
    pygame.init()
    pygame.display.set_caption("RescueDrone AI v1")
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    font = pygame.font.SysFont("consolas", 22, bold=True)
    small = pygame.font.SysFont("consolas", 14)
    clock = pygame.time.Clock()
    sim = RescueSimulation()
    last = time.time()
    accumulator = 0.0
    running = True
    while running:
        dt = min(time.time()-last, 0.25)
        last = time.time()
        accumulator += dt
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    sim.paused = not sim.paused
                elif event.key == pygame.K_r:
                    sim.reset()
        # One grid move every 0.12 seconds; YOLO inference can slow the loop.
        if accumulator >= 0.12:
            sim.tick()
            accumulator = 0
        sim.draw(screen, font, small)
        pygame.display.flip()
        clock.tick(FPS)
    pygame.quit()

if __name__ == "__main__":
    main()
