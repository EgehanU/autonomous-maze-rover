from __future__ import annotations

from typing import Any

import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle, FancyBboxPatch
import random
import time
from collections import deque
import math

Cell = tuple[int, int]
BoolMaze = npt.NDArray[np.bool_]
IntMaze = npt.NDArray[np.int_]


class MazeExplorer:
    def __init__(self) -> None:
        # robot body (cm)
        self.ROBOT_LENGTH: int = 27
        self.ROBOT_WIDTH: int = 17
        self.CELL_SIZE: int = 55

        # movement noise 
        self.movement_error: float = 0.25
        self.position_drift: float = 4.0
        self.false_movement_chance: float = 0.15
        self.orientation_error: float = 0.15

        self.FORWARD_ONLY: bool = True

        # world and state
        self.maze: BoolMaze | None = None
        self.known_maze: IntMaze | None = None
        self.robot_pos: list[int] = [0, 0]
        self.robot_real_pos: list[float] = [0.0, 0.0]
        self.robot_theta: float = 0.0
        self.visited_cells: set[Cell] = set()
        self.exploration_path: list[Cell] = []

        # exploration keeping data
        self.frontiers: set[Cell] = set()
        self.explored_cells: set[Cell] = set()
        self.wall_map: dict[Any, Any] = {}
        self.movement_history: list[Any] = []

        self.total_distance_traveled: int = 0
        self.exploration_steps: int = 0
        self.algorithm_used: str = ""
        self.belief_state: dict[Cell, float] = {}
        self.reward_weights: dict[str, float] = {'exploration': 10, 'travel_cost': -1, 'time_penalty': -0.1}

        self.fig: Figure | None = None
        self.ax: Axes | None = None

        self.start_time: float | None = None
        self.exploration_complete: bool = False

    def normalize_angle(self, angle: float) -> float:
        # wrap radians to -180 to 180
        angle_deg = math.degrees(angle)
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg <= -180:
            angle_deg += 360
        return math.radians(angle_deg)

    def generate_maze(self, width: int, height: int, wall_probability: float = 0.25) -> BoolMaze:
        # N,E,S,W walls
        maze = np.zeros((height, width, 4), dtype=bool)

        # Border walls
        for x in range(width):
            maze[0, x, 0] = True
            maze[height - 1, x, 2] = True
        for y in range(height):
            maze[y, 0, 3] = True
            maze[y, width - 1, 1] = True

        # internal walls
        for y in range(height):
            for x in range(width):
                if x < width -1 and random.random() < wall_probability:
                    maze[y, x, 1] = True
                    maze[y, x + 1, 3] = True
                if y < height -1 and random.random() < wall_probability:
                    maze[y, x, 2] = True
                    maze[y + 1, x, 0] = True

        # open a few corridors
        for _ in range(int(width * height * 0.1)):
            y = random.randint(1, height - 2)
            x = random.randint(1, width - 2)
            direction = random.randint(0, 3)
            if direction == 0 and maze[y, x, 0]:
                maze[y, x, 0] = False
                maze[y - 1, x, 2] = False
            elif direction == 1 and maze[y, x, 1]:
                maze[y, x, 1] = False
                maze[y, x + 1, 3] = False
        return maze

    def initialize_known_maze(self, width: int, height: int) -> None:
        # 0 unknown, 1 wall, 2 free
        self.known_maze = np.zeros((height, width, 4), dtype=int)
        self.wall_map = {}
        for x in range(width):
            self.known_maze[0, x, 0] = 1
            self.known_maze[height - 1, x, 2] = 1
        for y in range(height):
            self.known_maze[y, 0, 3] = 1
            self.known_maze[y, width - 1, 1] = 1

    def can_move(self, from_cell: Cell, to_cell: Cell) -> bool:
        # this cjeck adjency and if there is  a wall
        fy, fx = from_cell
        ty, tx = to_cell
        if abs(fx - tx) + abs(fy - ty) != 1:
            return False
        if tx < 0 or ty < 0 or tx >= self.maze.shape[1] or ty >= self.maze.shape[0]:
            return False

        # check wall in the real maze
        if tx > fx:      # east
            return not self.maze[fy, fx, 1]
        elif tx < fx:    # west
            return not self.maze[fy, fx, 3]
        elif ty > fy:    # south
            return not self.maze[fy, fx, 2]
        else:            # north
            return not self.maze[fy, fx, 0]

    def sense_environment(self, cell: Cell) -> None:
        # update local knowledge around current cell
        y, x = cell
        h, w = self.maze.shape[:2]

        for direction in (0, 1, 2, 3):
            if self.maze[y, x, direction]:
                self.known_maze[y, x, direction] = 1
                if direction == 0 and y > 0:
                    self.known_maze[y - 1, x, 2] = 1
                elif direction == 1 and x < w - 1:
                    self.known_maze[y, x + 1, 3] = 1
                elif direction == 2 and y < h - 1:
                    self.known_maze[y + 1, x, 0] = 1
                elif direction == 3 and x > 0:
                    self.known_maze[y, x - 1, 1] = 1
            else:
                self.known_maze[y, x, direction] = 2

        #Add lrotiers (adjacent, not yet explored, actually reachable)
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w:
                if (ny, nx) not in self.explored_cells and self.can_move(cell, (ny, nx)):
                    self.frontiers.add((ny, nx))

        # If alreadt explored, then drop
        self.frontiers = {f for f in self.frontiers if f not in self.explored_cells}

    def get_known_neighbors(self, cell: Cell) -> list[Cell]:
        y, x = cell
        neighbors: list[Cell] = []
        h, w = self.known_maze.shape[:2]
        for dy, dx, wall_dir in [(-1, 0, 0), (1, 0, 2), (0, -1, 3),(0, 1, 1)]:
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and self.known_maze[y, x, wall_dir] != 1:
                neighbors.append((ny, nx))
        return neighbors

    def calculate_distance(self, a: Cell, b: Cell) -> int:
        # manhattan
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def modified_bfs_with_tsp(self) -> tuple[list[Cell], int]:
        #Bfs-ish exploration, greedily pick nearest frontier
        self.algorithm_used = "Modified BFS + TSP"
        current_cell = tuple(self.robot_pos)
        self.explored_cells.add(current_cell)
        self.sense_environment(current_cell)

        exploration_order = [current_cell]
        total_travel_distance = 0

        while self.frontiers:
            current_pos = tuple(self.robot_pos)
            self.frontiers = {f for f in self.frontiers if f not in self.explored_cells}

            if not self.frontiers:
                h, w = self.maze.shape[:2]
                for y in range(h):
                    for x in range(w):
                        if (y, x) not in self.explored_cells and (y, x) not in self.visited_cells:
                            path = self.find_path_to_cell(current_pos, (y, x))
                            if path:
                                self.frontiers.add((y, x))
                if not self.frontiers:
                    break

            nearest_frontier = min(self.frontiers, key=lambda f: self.calculate_distance(current_pos,f))
            self.frontiers.remove(nearest_frontier)

            path = self.find_path_to_cell(current_pos, nearest_frontier)
            if path:
                for next_cell in path[1:]:
                    if self.move_robot_to_cell(next_cell):
                        exploration_order.append(next_cell)
                        total_travel_distance += 1
                        here = tuple(self.robot_pos)
                        if here not in self.explored_cells:
                            self.explored_cells.add(here)
                            self.sense_environment(here)
                        self.visualize_exploration_progress()
                        time.sleep(0.05)

            self.exploration_steps += 1

        return exploration_order, total_travel_distance

    def pomdp_exploration(self) -> tuple[list[Cell], int]:
        # frontier choice via a tiny reward model
        self.algorithm_used = "POMDP + Travel Cost"
        current_cell = tuple(self.robot_pos)
        self.explored_cells.add(current_cell)
        self.sense_environment(current_cell)

        exploration_order = [current_cell]
        total_travel_distance = 0
        self.initialize_belief_state()

        while self.frontiers:
            current_pos = tuple(self.robot_pos)
            self.frontiers = {f for f in self.frontiers if f not in self.explored_cells}

            if not self.frontiers:
                h, w = self.maze.shape[:2]
                for y in range(h):
                    for x in range(w):
                        if (y, x) not in self.explored_cells and (y, x) not in self.visited_cells:
                            path = self.find_path_to_cell(current_pos, (y, x))
                            if path:
                                self.frontiers.add((y, x))
                if not self.frontiers:
                    break

            best_frontier = self.select_best_frontier_pomdp(current_pos)
            if best_frontier is None:
                break

            self.frontiers.remove(best_frontier)
            path = self.find_path_to_cell(current_pos, best_frontier)
            if path:
                for next_cell in path[1:]:
                    if self.move_robot_to_cell(next_cell):
                        exploration_order.append(next_cell)
                        total_travel_distance += 1
                        here = tuple(self.robot_pos)
                        if here not in self.explored_cells:
                            self.explored_cells.add(here)
                            self.sense_environment(here)
                            self.update_belief_state(here)
                        self.visualize_exploration_progress()
                        time.sleep(0.05)

            self.exploration_steps += 1

        return exploration_order, total_travel_distance

    def initialize_belief_state(self) -> None:
        # 0.5 prob everywhere
        h, w = self.maze.shape[:2]
        for y in range(h):
            for x in range(w):
                self.belief_state[(y, x)] = 0.5

    def update_belief_state(self, cell: Cell) -> None:
        # updating the beluef state
        self.belief_state[cell] = 1.0
        y, x = cell
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            ny, nx = y + dy, x + dx
            if 0 <= ny < self.maze.shape[0] and 0 <= nx < self.maze.shape[1]:
                if self.can_move(cell, (ny, nx)):
                    self.belief_state[(ny, nx)] = min(1.0,self.belief_state.get((ny, nx), 0.5) + 0.2)

    def select_best_frontier_pomdp(self, current_pos: Cell) -> Cell | None:
        if not self.frontiers:
            return None
        best_frontier, best_score = None, float('-inf')
        for f in self.frontiers:
            d = self.calculate_distance(current_pos, f)
            belief = self.belief_state.get(f, 0.5)
            score = (self.reward_weights['exploration']* belief+self.reward_weights['travel_cost'] * d + self.reward_weights['time_penalty'] * self.exploration_steps)
            if score > best_score:
                best_score = score
                best_frontier = f
        return best_frontier

    def find_path_to_cell(self, start: Cell, goal: Cell) -> list[Cell] | None:
        # plain BFS on reachable edges (respecting real maze walls)
        if start == goal:
            return [start]
        queue = deque([(start, [start])])
        visited = {start}
        while queue:
            current, path = queue.popleft()
            for nb in self.get_known_neighbors(current):
                if self.can_move(current,nb) and nb not in visited:
                    new_path = path + [nb]
                    if nb == goal:
                        return new_path
                    queue.append((nb, new_path))
                    visited.add(nb)
        return None

    def move_robot_to_cell(self, target_cell: Cell) -> bool:
        current_cell = tuple(self.robot_pos)
        if not self.can_move(current_cell, target_cell):
            return False

        # random false step, this is added to simualte real environment, however might be unnecessary ehnce the purpose is
        # checking the algorithms
        if self.simulate_false_movement():
            return False

        # face target and go
        required_theta = self.get_required_orientation(current_cell, target_cell)
        self.orient_robot(required_theta)

        self.robot_pos = list(target_cell)
        self.robot_real_pos[0] = target_cell[1] *self.CELL_SIZE + self.CELL_SIZE / 2
        self.robot_real_pos[1]= target_cell[0] * self.CELL_SIZE + self.CELL_SIZE / 2

        _ = self.simulate_movement_error()  # keep noise model, ignore flag here

        self.visited_cells.add(tuple(self.robot_pos))
        self.total_distance_traveled += 1
        return True

    def get_required_orientation(self, from_cell: Cell, to_cell: Cell) -> float:
        fy, fx = from_cell
        ty, tx = to_cell
        if tx > fx:      # E
            return math.pi / 2
        elif tx < fx:   # W
            return -math.pi / 2
        elif ty < fy:   # N
            return 0.0
        else:            #S
            return math.pi

    def orient_robot(self, target_theta: float) -> None:
        # rotate in 90 degree steps with a small error
        turn_angle = self.calculate_turn_angle(self.robot_theta, target_theta)
        if abs(turn_angle) > 0.1:
            turns_needed = abs(round(math.degrees(turn_angle) / 90))
            if turns_needed == 0 and abs(turn_angle) > 0.1:
                turns_needed = 1
            direction = 'ccw' if turn_angle > 0 else 'cw'
            for _ in range(turns_needed):
                self.simulate_turn_90(direction)

    def calculate_turn_angle(self, current_theta: float, target_theta: float) -> float:
        #minumun signed diff  back to radians
        c = math.degrees(current_theta)
        t = math.degrees(target_theta)
        diff = t - c
        if diff > 180:
            diff -= 360
        elif diff <= -180:
            diff += 360
        return math.radians(diff)

    def simulate_turn_90(self, direction: str) -> None:
        # 90 degree + random error
        if direction == 'cw':
            target = self.normalize_angle(self.robot_theta - math.pi / 2)
        else:
            target = self.normalize_angle(self.robot_theta + math.pi / 2)
        orientation_error= random.uniform(-self.orientation_error, self.orientation_error)
        self.robot_theta = self.normalize_angle(target + orientation_error)
        time.sleep(0.05)

    def simulate_false_movement(self) -> bool:
        # sometimes the robot moves to a random reachable neighbor
        if random.random() < self.false_movement_chance:
            cur = tuple(self.robot_pos)
            options: list[Cell] = []
            for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                ny, nx = cur[0] + dy, cur[1] + dx
                if 0 <= ny < self.maze.shape[0] and 0<= nx < self.maze.shape[1]:
                    if self.can_move(cur, (ny, nx)):
                        options.append((ny, nx))
            if options:
                false_cell = random.choice(options)
                self.robot_pos = list(false_cell)
                self.robot_real_pos[0] = false_cell[1] * self.CELL_SIZE + self.CELL_SIZE / 2
                self.robot_real_pos[1] = false_cell[0] * self.CELL_SIZE + self.CELL_SIZE / 2
                self.robot_theta = self.normalize_angle(
                    self.robot_theta + random.uniform(-math.pi / 3, math.pi / 3)
                )
                self.visited_cells.add(tuple(self.robot_pos))
                return True
        return False

    def simulate_movement_error(self) -> bool:
        # add a bit of xy drift and a small heading wobble
        error = False
        if random.random() < self.movement_error:
            dx = random.uniform(-self.position_drift,self.position_drift)
            dy = random.uniform(-self.position_drift, self.position_drift)
            self.robot_real_pos[0] += dx
            self.robot_real_pos[1] += dy

            new_cell_x = int(self.robot_real_pos[0] // self.CELL_SIZE)
            new_cell_y = int(self.robot_real_pos[1] // self.CELL_SIZE)
            new_cell_x = max(0, min(self.maze.shape[1] - 1, new_cell_x))
            new_cell_y = max(0, min(self.maze.shape[0] - 1, new_cell_y))

            if (new_cell_y, new_cell_x) != tuple(self.robot_pos):
                if self.can_move(tuple(self.robot_pos), (new_cell_y, new_cell_x)):
                    self.robot_pos = [new_cell_y, new_cell_x]
                    error = True
                else:
                    # snap back center of current cell
                    self.robot_real_pos[0]= self.robot_pos[1] * self.CELL_SIZE + self.CELL_SIZE/2
                    self.robot_real_pos[1] = self.robot_pos[0] * self.CELL_SIZE + self.CELL_SIZE/2

        if random.random() <self.movement_error * 0.7:
            self.robot_theta = self.normalize_angle(
                self.robot_theta + random.uniform(-self.orientation_error * 0.5,
                                                  self.orientation_error * 0.5)
            )
        return error

    def visualize_exploration_progress(self) -> None:
        if self.fig is None:
            self.fig, self.ax = plt.subplots(figsize=(14, 10))

        self.ax.clear()
        h, w = self.maze.shape[:2]

        for y in range(h):
            for x in range(w):
                if (y, x) in self.explored_cells:
                    color = 'lightgreen'
                elif (y, x) in self.frontiers:
                    color = 'orange'
                elif (y, x) in self.visited_cells:
                    color = 'lightblue'
                else:
                    color = 'lightgray'
                if (y, x) == tuple(self.robot_pos):
                    color = 'red'

                rect = Rectangle((x * self.CELL_SIZE, y * self.CELL_SIZE),
                                 self.CELL_SIZE, self.CELL_SIZE,
                                 facecolor=color, edgecolor='black', linewidth=0.5)
                self.ax.add_patch(rect)

                if self.maze[y, x, 0]:
                    self.ax.plot([x * self.CELL_SIZE, (x + 1) * self.CELL_SIZE],
                                 [y * self.CELL_SIZE, y * self.CELL_SIZE], 'k-', linewidth=3)
                if self.maze[y, x, 1]:
                    self.ax.plot([(x + 1) * self.CELL_SIZE, (x + 1) * self.CELL_SIZE],
                                 [y * self.CELL_SIZE, (y + 1) * self.CELL_SIZE], 'k-', linewidth=3)
                if self.maze[y, x, 2]:
                    self.ax.plot([x * self.CELL_SIZE, (x + 1) * self.CELL_SIZE],
                                 [(y + 1) * self.CELL_SIZE, (y + 1) * self.CELL_SIZE], 'k-', linewidth=3)
                if self.maze[y, x, 3]:
                    self.ax.plot([x * self.CELL_SIZE, x * self.CELL_SIZE],
                                 [y * self.CELL_SIZE, (y + 1) * self.CELL_SIZE], 'k-', linewidth=3)

        cx = self.robot_real_pos[0]
        cy = self.robot_real_pos[1]
        body = FancyBboxPatch(
            (cx - self.ROBOT_WIDTH / 2, cy - self.ROBOT_LENGTH / 2),
            self.ROBOT_WIDTH, self.ROBOT_LENGTH,
            boxstyle="round,pad=1", facecolor='darkred', alpha=0.8, edgecolor='black'
        )
        transform = plt.matplotlib.transforms.Affine2D().rotate_around(cx, cy, self.robot_theta) + self.ax.transData
        body.set_transform(transform)
        self.ax.add_patch(body)

        L = 15
        ax2 = cx + L * math.sin(self.robot_theta)
        ay2 = cy - L * math.cos(self.robot_theta)
        self.ax.annotate('', xy=(ax2, ay2), xytext=(cx, cy),
                         arrowprops=dict(arrowstyle='->', color='white', lw=3))

        self.ax.set_xlim(-5, w * self.CELL_SIZE + 5)
        self.ax.set_ylim(-5, h * self.CELL_SIZE + 5)
        self.ax.set_aspect('equal')
        self.ax.invert_yaxis()

        explored_pct = (len(self.explored_cells) / (h * w)) * 100
        self.ax.set_title(f'{self.algorithm_used} - Exploration: {explored_pct:.1f}% '
                          f'({len(self.explored_cells)}/{h * w})')

        legend_elements = [
            patches.Patch(color='lightgreen', label='Explored'),
            patches.Patch(color='orange', label='Frontiers'),
            patches.Patch(color='lightblue', label='Visited'),
            patches.Patch(color='darkred', label='Robot'),
            patches.Patch(color='lightgray', label='Unknown')
        ]
        self.ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1, 1))

        plt.tight_layout()
        plt.pause(0.01)

    def reset_exploration_state(self) -> None:
        self.visited_cells = set()
        self.explored_cells =set()
        self.frontiers = set()
        self.wall_map ={}
        self.movement_history = []
        self.total_distance_traveled = 0
        self.exploration_steps = 0
        self.belief_state= {}
        h, w = self.maze.shape[:2]
        self.initialize_known_maze(w, h)

    def print_final_statistics(self) -> None:
        maze_size = self.maze.shape[0] * self.maze.shape[1]
        explored_pct = (len(self.explored_cells) / maze_size) * 100
        print(f"Algorithm: {self.algorithm_used} | Explored: {len(self.explored_cells)}/{maze_size} "
        f"({explored_pct:.1f}%) | Distance: {self.total_distance_traveled} | Steps: {self.exploration_steps}")


    def prepare_run(self, maze: BoolMaze, maze_size_known: bool, start_rc: Cell = (0, 0)) -> None:
        # set the maze and init knowledge
        self.maze = maze
        h, w = self.maze.shape[:2]

        
        sy, sx = start_rc # start position
        self.robot_pos = [sy, sx]
        self.robot_real_pos =[sx * self.CELL_SIZE + self.CELL_SIZE / 2, sy * self.CELL_SIZE + self.CELL_SIZE / 2]
        self.robot_theta = 0.0
        self.visited_cells = {(sy, sx)}

        # known vs unknown
        if maze_size_known:
            self.initialize_known_maze(w, h)
        else:
            buf = max(w, h) + 10  # oversized canvas when the size is unknwon 
            self.initialize_known_maze(buf, buf)

        self.start_time = time.time()
        self.visualize_exploration_progress()

    def run_one(self, algo_name: str) -> None:
        # run one algorithm and block until window closed
        if algo_name == "bfs":
            self.modified_bfs_with_tsp()
        else:
            self.pomdp_exploration()
        self.print_final_statistics()
        plt.show()   # block here and close window to continue
        plt.close('all')
        self.fig = None
        self.ax = None


    def run_exploration_simulation(self) -> None:
        print("1) Single RUn")
        print("2) 4 runs on small map (7x5): ")
        print("3)4 runs on big map (10x12):")

        sel = input("\ 1,2,3: ").strip()
        while sel not in ("1", "2", "3"):
            sel = input("Please enter 1, 2, or 3: ").strip()

        if sel == "1":
            self.single_run_menu()
        elif sel =="2":
            self.batch_runs(map_size="small")
        else:
            self.batch_runs(map_size="big")

    def single_run_menu(self) -> None:
        print("\nMaps:")
        
        print("small 7x5")
        print("big 10x12")
        
        msel = input("Choose 1/2: ").strip()
        
        while msel not in ("1", "2"):
            msel = input("Enter 1 or 2: ").strip()
        if msel == "1":
            maze = self.generate_maze(7, 5, 0.2)
        else:
            maze = self.generate_maze(12, 10, 0.25)

        # known ro unknown, choose bitte
        print("\nMaze size knowledge:")
        print("1. Known")
        print("2. Unknown")
        ksel = input("Choose 1/2:").strip()
        while ksel not in ("1", "2"):
            ksel = input("Enter 1 or 2: ").strip()
        maze_size_known = (ksel == "1")

        # start cell
        h, w = maze.shape[:2]
        def valid_rc(s: str) -> bool:
            parts = [p.strip() for p in s.split(",")]
            if len(parts) != 2: return False
            a, b = parts
            return (a.isdigit() and b.isdigit() and 0 <= int(a) <h and 0<= int(b) < w)

        
        sy, sx = 0, 0
        
        # choose algorithm
        print("\nAlgorithm:")
        print("1) Modified BFS + TSP")
        print("2) POMDP + Travel Cost")
        asel = input("Choose 1/2: ").strip()
        while asel not in ("1", "2"):
            asel = input("1 or 2: ").strip()

        self.reset_for_new_session()
        self.prepare_run(maze, maze_size_known, (sy, sx))
        if asel == "1":
            self.run_one("bfs")
        else:
            self.run_one("pomdp")



    def batch_runs(self, map_size: str = "small") -> None:
        # in order to fasten up the process it uses one random generated map for all 4 runs,
        # close one window to see the other one
        if map_size == "small":
            maze = self.generate_maze(7, 5, 0.2)
            label = "7x5"
        else:
            maze = self.generate_maze(12, 10, 0.25)
            label = "10x12"

        print(f"\nBatch on {label} map:")
        print("Order: BFS_known → POMDP_known → BFS_unknown → POMDP_unknown")
        configs = [("bfs",   True),("pomdp", True), ("bfs",   False), ("pomdp", False),]


        for algo, known in configs:
            self.reset_for_new_session()
            # Start pos is 0,0, code can also do other one but for our purpose, unnecessary
            self.prepare_run(maze, known, (0, 0))
            print(f"\nRunning {algo.upper()} | {'KNOWN' if known else 'UNKNOWN'} | start (0,0)")
            self.run_one(algo)


    def reset_for_new_session(self) -> None:
        # wipe figure and state between independent runs
        if self.fig is not None:
            plt.close(self.fig)
            
        self.fig, self.ax = None, None
        self.visited_cells = set()
        self.explored_cells = set()
        self.frontiers = set()
        self.wall_map = {}
        self.movement_history = []
        self.total_distance_traveled = 0
        self.exploration_steps = 0
        self.belief_state = {}
        self.algorithm_used = ""
        self.start_time = None


if __name__ == "__main__":
    explorer = MazeExplorer()
    explorer.run_exploration_simulation()
