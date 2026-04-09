from __future__ import annotations

import math
import numpy as np
import numpy.typing as npt
import json
import os
import time
from collections import deque
from datetime import datetime

from src.config.config import *
from src.control.odometry_and_control import *
from src.sensors.imu_filtered import start_heading_tracker, get_heading
from src.control.controller import *

Cell = tuple[int, int]
MazeGrid = npt.NDArray[np.int_]

class HardwareMazeExplorer:
    def __init__(self) -> None:
        # The same logic as the solver, sizes
        self.ROBOT_LENGTH: int = 27
        self.ROBOT_WIDTH: int = 17
        self.CELL_SIZE: int = 57

        # Position thresholds (I keep these loose for hardware), in this code there is also addition of goinf back when collision occurs or about to occur

        self.FORWARD_POSITION_LIMIT: float = 10.0
        self.BACKWARD_POSITION_LIMIT: float = 15.0
        self.SIDE_POSITION_LIMIT: float = 15.0

        # IR wall detection
        self.WALL_DETECTION_THRESHOLD: float = 48.0
        self.SAFE_WALL_DISTANCE: float = 11.0
        self.IR_READINGS_PER_DIRECTION: int = 3

        # Servo angles left, middle, right
        self.SERVO_LEFT: int = 63
        self.SERVO_MIDDLE: int = 34
        self.SERVO_RIGHT: int = 5

        self.IMU_CHECK_THRESHOLD: float = 10.0
        self.expected_heading: float = 0.0

        # Maze state
        self.maze_known: bool = True
        self.maze_size: tuple[int, int] | None = None
        self.known_maze: MazeGrid | None = None
        self.start_cell: Cell = (0, 0)  #always start at (0,0)

    
        self.explored_cells: set[Cell] = set()
        self.frontiers: set[Cell] = set()
        self.current_cell: Cell = (0, 0)
        self.exploration_path: list[Cell] = []
        self.total_distance_traveled: int = 0
        self.exploration_steps: int = 0

        # Algorithm bits
        self.algorithm_used: str = ""
        self.belief_state: dict[Cell, float] = {}
        self.reward_weights: dict[str, float] = {'exploration': 10, 'travel_cost': -1, 'time_penalty': -0.1}

        # Movement tuning
        self.movement_speed: int = 110
        self.turn_speed: int = 110
        self.base_move_time: float = 2.5

        # Recovery
        self.consecutive_failures: int = 0
        self.max_failures: int = 3

        # For the maze with the unknown size
        self.maze_offset_y: int = 0
        self.maze_offset_x: int = 0

    def normalize_angle_180(self, angle: float) -> float:
        while angle > 180:
            angle -= 360
        while angle <= -180:
            angle += 360
        return angle

    def get_imu_heading_normalized(self) -> float:
        raw = get_heading()
        return raw - 360 if raw > 180 else raw

    def log_status_brief(self) -> None:
        # print cell and heading 
        cell = self.real_position_to_cell()
        imu = self.get_imu_heading_normalized()
        print(f"Cell: {cell} | IMU: {imu:.1f}°")

    def real_position_to_cell(self, x: float | None = None, y: float | None = None) -> Cell:
        # convert position from odomery to cell index
        if x is None or y is None:
            x, y, _ = self.get_position()
        cell_x = int(round((-28.5 -x) / self.CELL_SIZE))
        cell_y = int(round((y -28.5) / self.CELL_SIZE))
        return (cell_y, cell_x)

    def cell_to_array_coords(self, cell_y: int, cell_x: int) -> Cell:
        if self.maze_known:
            return cell_y, cell_x
        return cell_y + self.maze_offset_y,cell_x + self.maze_offset_x

    def array_to_cell_coords(self, array_y: int, array_x: int) -> Cell:
        if self.maze_known:
            return array_y, array_x
        return array_y -self.maze_offset_y, array_x - self.maze_offset_x

    def cell_to_real_position(self, cell_y: int, cell_x: int) -> tuple[float, float]:
        # the values of 28.5 are there because of the robots camera point is taken as the reference
        rx = -28.5 - (cell_x * self.CELL_SIZE)
        ry = 28.5 + (cell_y * self.CELL_SIZE)
        return rx, ry

    def get_position(self) -> tuple[float, float, float]:
        read_data()
        return position["x"], position["y"], math.degrees(position["theta"])

    def initialize_hardware(self) -> None:
        # Imu takes the first position as zero angle
        start_heading_tracker()
        update_speed(new_speed=self.movement_speed, new_turn=self.turn_speed)
        # initial cell is always assumed as 0,0 with facing northh
        position["x"] = -28.5
        position["y"] = 28.5
        position["theta"] = 0
        self.expected_heading = 0
        send_command(SET_SERVO_ANGLE, servo_angle=self.SERVO_MIDDLE)
        time.sleep(0.3)
        self.log_status_brief()

    def initialize_maze(self, width: int | None = None, height: int | None = None) -> None:
        if self.maze_known and width and height:
            self.maze_size = (height,width)
            self.known_maze =np.zeros((height, width, 4),dtype=int)
            # Outer boundaries are walls.
            for x in range(width):
                self.known_maze[0, x, 0] = 1
                self.known_maze[height - 1, x, 2] = 1
            for y in range(height):
                self.known_maze[y,0, 3] = 1
                self.known_maze[y, width - 1,1] = 1
        else:
            # Unknown maze: I start with a centered expandable array.
            initial = 40
            self.known_maze = np.full((initial, initial, 4), -1, dtype=int)
            self.maze_offset_y = initial // 2
            self.maze_offset_x = initial // 2
        # At start I assume there is a south wall behind me in known-maze mode.
        if self.maze_known:
            self.known_maze[0,0, 2] = 1

    def check_position_in_cell(self, target_cell: Cell) -> str:
        cx, cy, _ =self.get_position()
        tx, ty = self.cell_to_real_position(target_cell[0], target_cell[1])
        dx = cx - tx
        dy = cy - ty
        if dy > self.FORWARD_POSITION_LIMIT:
            return "too_forward"
        if dy < -self.BACKWARD_POSITION_LIMIT:
            return "too_backward"
        if abs(dx) > self.SIDE_POSITION_LIMIT:
            return "too_side"
        return "good"

    def detect_wall_with_ir(self, num_readings: int = 3) -> bool | None:
        vals: list[float] = []
        for _ in range(num_readings):
            d = read_filtered_data() # uses  a second median filter on the master side, the ir sensor is too noisy
            if d is not None:
                vals.append(d)
            time.sleep(0.05)
        if not vals:
            return None
        avg = sum(vals)/ len(vals)
        return avg <self.WALL_DETECTION_THRESHOLD

    def scan_walls_in_cell(self, cell: Cell) -> dict[str, bool | None]:
        walls: dict[str, bool | None] = {}
        skip_back_wall = (cell == (0, 0))
        cur_head = self.get_imu_heading_normalized()
        #front
        send_command(SET_SERVO_ANGLE, servo_angle=self.SERVO_MIDDLE); time.sleep(0.2)
        front = self.detect_wall_with_ir(self.IR_READINGS_PER_DIRECTION)
        # Left
        send_command(SET_SERVO_ANGLE, servo_angle=self.SERVO_LEFT); time.sleep(0.2)
        left = self.detect_wall_with_ir(self.IR_READINGS_PER_DIRECTION)
        #Right
        send_command(SET_SERVO_ANGLE, servo_angle=self.SERVO_RIGHT); time.sleep(0.2)
        right = self.detect_wall_with_ir(self.IR_READINGS_PER_DIRECTION)
        # Back to middle
        send_command(SET_SERVO_ANGLE, servo_angle=self.SERVO_MIDDLE); time.sleep(0.2)

        # Map servo directions -> world based on current heading.
        if -45 <= cur_head <= 45:  # North
            walls['north'] = front
            walls['west'] = left
            walls['east'] = right
            walls['south'] = True if skip_back_wall else None
        elif 45 < cur_head <= 135:  # West
            walls['west'] = front
            walls['south'] = left
            walls['north'] = right
            walls['east'] = None if not skip_back_wall else False
        elif -135 <= cur_head < -45:  # east
            walls['east'] = front
            walls['north'] = left
            walls['south'] = right
            walls['west'] = None if not skip_back_wall else False
        else:  # Facing South
            walls['south'] = front
            walls['east'] = left
            walls['west'] = right
            walls['north'] = None

        self.update_maze_walls(cell, walls)
        self.log_status_brief()
        return walls

    def update_maze_walls(self, cell: Cell, walls: dict[str, bool | None]) -> None:
        y, x = cell
        ay, ax = self.cell_to_array_coords(y, x)
        if not (0 <= ay < self.known_maze.shape[0] and 0 <= ax< self.known_maze.shape[1]):
            return
        # N,E,S,W -> 0,1,2,3, by this updating wall info
        if 'north' in walls and walls['north'] is not None:
            self.known_maze[ay, ax, 0] = 1 if walls['north'] else 0
        if 'east' in walls and walls['east'] is not None:
            self.known_maze[ay, ax, 1] = 1 if walls['east'] else 0
        if 'south' in walls and walls['south'] is not None:
            self.known_maze[ay, ax, 2] = 1 if walls['south'] else 0
        if 'west' in walls and walls['west'] is not None:
            self.known_maze[ay, ax, 3] = 1 if walls['west'] else 0
            
        H, W = self.known_maze.shape[:2]
        if ay > 0 and self.known_maze[ay, ax, 0] != -1:
            self.known_maze[ay - 1, ax, 2] = self.known_maze[ay, ax, 0]
        if ax < W - 1 and self.known_maze[ay, ax, 1] != -1:
            self.known_maze[ay, ax + 1, 3] = self.known_maze[ay, ax, 1]
        if ay < H - 1 and self.known_maze[ay, ax, 2] != -1:
            self.known_maze[ay + 1, ax, 0] = self.known_maze[ay, ax, 2]
        if ax > 0 and self.known_maze[ay, ax, 3] != -1:
            self.known_maze[ay, ax - 1, 1] = self.known_maze[ay, ax, 3]

    def get_accessible_neighbors(self, cell: Cell) -> list[Cell]:
        y, x = cell
        nbrs: list[Cell] = []
        ay, ax = self.cell_to_array_coords(y, x)
        if not (0 <= ay < self.known_maze.shape[0] and 0 <= ax < self.known_maze.shape[1]):
            return nbrs
        directions = [((1, 0), 0), ((0, 1), 1), ((-1, 0), 2), ((0, -1), 3)]
        for (dy, dx), widx in directions:
            ny, nx = y + dy, x + dx
            if self.maze_known:
                if not (0 <= ny < self.maze_size[0] and 0 <= nx < self.maze_size[1]):
                    continue
            else:
                nay, nax = self.cell_to_array_coords(ny, nx)
                if not (0 <= nay < self.known_maze.shape[0] and 0 <= nax < self.known_maze.shape[1]):
                    nbrs.append((ny, nx))  # still a frontier candidate
                    continue
            wall_status = self.known_maze[ay, ax, widx]
            if wall_status != 1:  # open 0 or unknown -1
                nbrs.append((ny, nx))
        return nbrs

    def get_distance_to_cell_center(self, target_cell: Cell) -> float:
        cx, cy, _ = self.get_position()
        tx, ty = self.cell_to_real_position(target_cell[0], target_cell[1])
        return math.hypot(tx - cx, ty - cy)

    def correct_heading_with_imu(self) -> None:# same logic as the solver 
        cur = self.get_imu_heading_normalized()
        err = self.normalize_angle_180(self.expected_heading - cur)
        if abs(err) > self.IMU_CHECK_THRESHOLD:
            rotate(err)
            time.sleep(0.3)
        self.log_status_brief()

    def move_to_adjacent_cell(self, target_cell: Cell) -> bool:
        current_cell = self.real_position_to_cell()
        dy = target_cell[0] - current_cell[0]
        dx = target_cell[1] - current_cell[1]
        if dy == 1:
            target_heading = 0; direction = "NORTH"
        elif dy == -1:
            target_heading = 180; direction = "SOUTH"
        elif dx == 1:
            target_heading = -90; direction = "EAST"
        elif dx == -1:
            target_heading = 90; direction = "WEST"
        else:
            return False
        cur_imu = self.get_imu_heading_normalized()
        err = self.normalize_angle_180(target_heading - cur_imu)
        if abs(err) > 15:
            rotate(err); time.sleep(0.6)
        self.expected_heading = target_heading
        # Stop flag check before moving
        read_data()
        if position.get("stop_flag", False):
            self.handle_collision()
            return False

        # depending on the direction move forward for some time then apply heading
        move_for_time(MOVE_FORWARD, duration_seconds=self.base_move_time)
        time.sleep(0.3)

        read_data()
        if position.get("stop_flag", False): # sent by due
            self.handle_collision()
            return False

        # Light heading touch-up and status line
        self.correct_heading_with_imu()

        self.current_cell =self.real_position_to_cell()
        self.total_distance_traveled += 1

        final_d = self.get_distance_to_cell_center(target_cell)
        ok = (self.current_cell == target_cell) or (final_d <=20.0)
        self.consecutive_failures = 0 if ok else self.consecutive_failures+ 1
        return True  # after a certain number of consecutive failures the failure becomes unavoidable so jsut stop the thing

    def handle_collision(self) -> None:
        # the new addition is basically when collision occurs or about toccur, robot makes backward motion
        # stop, go back, IMU apply
        send_command(MOVE_STOP)
        time.sleep(0.3)
        move_for_time(MOVE_BACKWARD, 1.0) 
        time.sleep(0.3)
        self.correct_heading_with_imu()
        cell = self.real_position_to_cell()
        head = self.get_imu_heading_normalized()
        if -45 <= head <= 45:
            w = 0
        elif 45 < head <= 135:
            w = 3
        elif -135 <= head < -45:
            w = 1
        else:
            w = 2
        y, x = cell
        ay, ax = self.cell_to_array_coords(y, x)
        if 0 <= ay < self.known_maze.shape[0] and 0 <= ax < self.known_maze.shape[1]:
            self.known_maze[ay, ax, w] = 1
        self.consecutive_failures += 1
        self.log_status_brief()

    def find_path_to_cell(self, start: Cell, goal: Cell) -> list[Cell] | None:
        if start == goal:
            return [start]
        q = deque([(start, [start])])
        vis = {start}
        while q:
            cur, path = q.popleft()
            for nb in self.get_accessible_neighbors(cur):
                if nb not in vis:
                    npth = path + [nb]
                    if nb == goal:
                        return npth
                    q.append((nb, npth))
                    vis.add(nb) # returns a list of cell from start to tatgeted cell
        return None

    def update_frontiers(self, cell: Cell) -> None:
        # after scanning the wall update the frontiers
        # cell which are not been visited before 
        # are adjacent to open or unknwon one
        # if size known no need to consider outsiders
        self.frontiers.discard(cell)
        y,x = cell
        ay, ax = self.cell_to_array_coords(y, x)
        directions = [((1, 0),0), ((0,1), 1), ((-1, 0), 2),((0, -1), 3)]
        for (dy, dx), widx in directions:
            ny, nx = y + dy, x + dx
            if self.maze_known:
                if not (0 <= ny < self.maze_size[0] and 0 <= nx < self.maze_size[1]):
                    continue
                nay, nax = ny, nx
            else:
                nay, nax = self.cell_to_array_coords(ny,nx)
                if not (0 <= nay < self.known_maze.shape[0] and 0<= nax < self.known_maze.shape[1]):
                    if (ny, nx) not in self.explored_cells:
                        self.frontiers.add((ny, nx))
                    continue
            wall = self.known_maze[ay, ax, widx]
            if wall == 1:
                continue
            if (ny, nx) not in self.explored_cells:
                self.frontiers.add((ny,nx))

    def modified_bfs_with_tsp(self) -> None:
        # not totally TSP, due to manhattan but
        # for a smaller maze might work
        self.algorithm_used = "Modified BFS + TSP"
        self.explored_cells.add(self.current_cell)
        self.scan_walls_in_cell(self.current_cell)
        self.update_frontiers(self.current_cell)
        while self.frontiers:
            cur = self.real_position_to_cell()
            target = min(self.frontiers, key=lambda f: abs(f[0]-cur[0]) + abs(f[1]-cur[1])) # pick the nearest frontier by Manhattan
            path = self.find_path_to_cell(cur, target)
            if path and len(path) > 1:
                for i in range(1, len(path)):
                    step_cell = path[i]
                    if not self.move_to_adjacent_cell(step_cell):
                        self.handle_collision()
                        break
                    read_data()
                    if position.get("stop_flag",False):
                        self.handle_collision()
                        break
                    time.sleep(0.3)
                final = self.real_position_to_cell()
                if final not in self.explored_cells:
                    self.explored_cells.add(final)
                    self.scan_walls_in_cell(final)
                    self.update_frontiers(final)
                    self.exploration_steps +=1
            else:
                self.frontiers.discard(target)
            if self.exploration_steps > 50:
                break
        self.log_status_brief()

    # reward = w_explore * belief(frontier)+ w_travel * (–distance) + w_time   * (–elapsed steps)
    # Strart the belief state at 0.5 for all,
    # if visited make it 1, not a complete POMDP, but like an 
    # inspired lightweight
    def pomdp_exploration(self) -> None:
        self.algorithm_used = "POMDP + Travel Cost"
        self.initialize_belief_state()
        self.explored_cells.add(self.current_cell)
        self.scan_walls_in_cell(self.current_cell)
        self.update_frontiers(self.current_cell)
        while self.frontiers:
            cur= self.real_position_to_cell()
            best = self.select_best_frontier_pomdp(cur)
            if best is None:
                break
            path = self.find_path_to_cell(cur, best)
            if path and len(path)> 1:
                for i in range(1, len(path)):
                    step_cell = path[i]
                    if not self.move_to_adjacent_cell(step_cell):
                        self.handle_collision()
                        break
                    read_data()
                    if position.get("stop_flag", False):
                        self.handle_collision()
                        break
                    time.sleep(0.3)
                final =self.real_position_to_cell()
                if final not in self.explored_cells:
                    self.explored_cells.add(final)
                    self.scan_walls_in_cell(final)
                    self.update_frontiers(final)
                    self.update_belief_state(final)
                    self.exploration_steps+= 1
            else:
                self.frontiers.discard(best)
            if self.exploration_steps > 50:
                break
        self.log_status_brief()

    def initialize_belief_state(self) -> None:
        # if the maze is known, just use its sizes
        if self.maze_known:
            H, W = self.maze_size
        else:
            H, W = self.known_maze.shape[:2]
        for i in range(H):
            for j in range(W):
                self.belief_state[(i,j)] = 0.5 #mean unknown but potentiall interesting

    def update_belief_state(self, cell: Cell) -> None:
        # after visiting a cell make it 1, make neightbour min 1
        self.belief_state[cell] = 1.0
        for nb in self.get_accessible_neighbors(cell):
            if nb in self.belief_state:
                self.belief_state[nb] = min(1.0,self.belief_state[nb] + 0.2)

        
        # if exp is larger more aggresive approach
        # if travel cost is more -, go shorter path
        # if time pen is more neg, finish sooner, like a limit

    def select_best_frontier_pomdp(self, current_pos: Cell) -> Cell | None:
        if not self.frontiers:
            return None
        best_f = None
        best_r = float('-inf')
        for f in self.frontiers:
            dist = abs(f[0] - current_pos[0]) +abs(f[1] - current_pos[1])
            expl = self.reward_weights['exploration']
            cost = self.reward_weights['travel_cost'] *dist
            time_pen =self.reward_weights['time_penalty'] * self.exploration_steps
            belief = self.belief_state.get(f, 0.5)
            reward = expl * belief + cost + time_pen
            if reward> best_r:
                best_r = reward
                best_f = f
        return best_f
        
    #wall booleans per cell (N,E,S,W)
    # a timestamp and a short description stating which algorithm I used
    def save_explored_maze(self) -> str:
        timestamp= datetime.now().isoformat()
        if self.maze_known:
            H, W =self.maze_size
            section = self.known_maze
            min_y = 0
            min_x = 0
        else:
            ys = [c[0] for c in self.explored_cells] or [0]
            xs = [c[1] for c in self.explored_cells] or [0]
            min_y, max_y = min(ys),max(ys)
            min_x, max_x = min(xs),max(xs)
            H = max_y-min_y + 1
            W = max_x-min_x + 1
            off_y = min_y+self.maze_offset_y
            off_x = min_x+self.maze_offset_x
            section = self.known_maze[off_y:off_y+H, off_x:off_x+W]
        maze_list = []

        for y in range(H):
            row= []
            for x in range(W):
                if self.maze_known:
                    cw = [bool(self.known_maze[y, x, k]) for k in range(4)]
                else:
                    cw = [bool(section[y,x, k]) for k in range(4)]
                row.append(cw)
            maze_list.append(row)
        maze_data = {
            "maze": maze_list,
            "description": f"Hardware exploration - {self.algorithm_used}",
            "dimensions": {"height": H, "width": W},
            "boundaries": {"min_x": 0, "max_x": W - 1, "min_y": 0, "max_y": H - 1},
            "exploration_info": {
                "algorithm_used": self.algorithm_used,
                "cells_explored": len(self.explored_cells),
                "total_distance_traveled": self.total_distance_traveled,
                "exploration_steps": self.exploration_steps,
                "original_maze_size": f"{H}x{W}",
                "exploration_percentage": (len(self.explored_cells) / (H * W)) * 100 if H * W else 0,
            },
            "robot_specs": {"cell_size": self.CELL_SIZE, "robot_length": self.ROBOT_LENGTH, "robot_width": self.ROBOT_WIDTH},
            "source": "Hardware Maze Explorer",
            "timestamp": timestamp,
        }
        fn = f"explored_maze_{self.algorithm_used.lower().replace(' ', '_')}_{timestamp.replace(':', '-').replace('.', '-')}.json"
        with open(fn,'w') as f:
            json.dump(maze_data, f,indent=2)
        return fn



    def run_exploration(self) -> None:
        self.initialize_hardware()
        while True:
            mode = input("\nKnown or Unknown 1,2").strip()
            if mode in ('1','2'):
                self.maze_known = (mode == '1')
                break
        if self.maze_known:
            while True:
                try:
                    dims = input("Enter height,width: ").strip()
                    h, w = map(int, dims.split(','))
                    if h > 0 and w > 0:
                        self.initialize_maze(w,h)
                        break
                except Exception:
                    pass
                print("Error")
        else:
            self.initialize_maze()
        print("Modified BFS + TSP or POMDP + Travel Cost")
        while True:
            algo = input("Choose 1.2:").strip()
            if algo in ('1', '2'):
                break
        start_t = time.time()
        ok = False
        try:
            if algo =='1':
                self.modified_bfs_with_tsp()
            else:
                self.pomdp_exploration()
            ok = True
        except KeyboardInterrupt:
            pass
        except Exception:
            pass
        elapsed = time.time() - start_t
        print(f"\nAlgorithm: {self.algorithm_used}")
        print(f"Cells explored: {len(self.explored_cells)}")
        print(f"Steps: {self.exploration_steps}")
        print(f"Time: {elapsed:.1f}s")
        if self.maze_known and self.maze_size:
            cov = (len(self.explored_cells) /(self.maze_size[0] * self.maze_size[1])) * 100
            print(f"Coverage: {cov:.1f}%")
        if ok and len(self.explored_cells) > 1:
            if input("Save explored maze? (y/n): ").strip().lower() in ("y", "yes"):
                self.save_explored_maze()
        send_command(MOVE_STOP)
def main() -> None:
  
    explorer = HardwareMazeExplorer()
    explorer.run_exploration()


if __name__ == "__main__":
    main()
