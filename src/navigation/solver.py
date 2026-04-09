from __future__ import annotations

from collections.abc import Callable
import time
import math
import numpy as np
import numpy.typing as npt
import heapq
import json
import os
from collections import deque

from src.config.config import *
from src.control.odometry_and_control import *
from src.sensors.imu_filtered import start_heading_tracker, get_heading
from src.control.controller import *
from src.fusion.sensor_fusion import *

Cell = tuple[int, int]
MazeGrid = npt.NDArray[np.int_]


class HardwareMazeSolver:
    def __init__(self, cell_size: int = 57) -> None:
        # Those stuff could be changed, and even though length of a cell is 55cm, becasue how some conencted there is additional acouple cm sometimes
        # hence I got better results with 57
        self.CELL_SIZE: int = cell_size
        self.POSITION_TOLERANCE: float = 5.0
        self.ANGLE_TOLERANCE: float = 8.0
        self.MAX_CORRECTION_ATTEMPTS: int = 1
        self.MAX_REPLANNING_ATTEMPTS: int = 2
        self.MOVEMENT_TIMEOUT: float = 2.0
        self.ROBOT_LENGTH: int = 27

        # Maze + navigation state
        self.maze: MazeGrid | None = None
        self.target_pos: list[int] = [0, 0]
        self.current_path: list[Cell] = []
        self.replanning_count: int = 0

        self.consecutive_failed_moves: int = 0
        self.MAX_FAILED_MOVES: int = 8

        # IMU heading -180 to180 
        self.expected_heading: float = 0.0
        self.imu_correction_threshold: float = 8.0

        # IMU correction right after a forward move
        self.post_movement_imu_check: bool = True
        self.post_movement_threshold: float = 12.0

        # This timing is found by testing, hence it approxiamtely goes like 55 cm in 1 go
        self.base_move_time: float = 3.2
        self.current_move_time: float = 3.2
        self.time_adjustment_factor: float = 0.6

        # Interactive mode is designed to debug stuff, make it true to have it, then it will ask
        self.interactive_mode: bool = False

    def normalize_angle_180(self, angle: float) -> float:
        while angle > 180:
            angle -= 360
        while angle <= -180:
            angle += 360
        return angle

    def get_imu_heading_normalized(self) -> float:
        raw = get_heading()
        return raw - 360 if raw > 180 else raw

    def real_position_to_cell(self, x: float | None = None, y: float | None = None) -> Cell:
        # If no position is provided, I read the live position first.
        if x is None or y is None:
            x, y, _ = self.get_actual_position_from_print()
        cell_x = int(round((-28.5 - x) / self.CELL_SIZE))
        cell_y = int(round((y - 28.5) / self.CELL_SIZE))
        if self.maze is not None:
            cell_x = max(0, min(self.maze.shape[1] - 1, cell_x))
            cell_y = max(0, min(self.maze.shape[0] - 1, cell_y))
        return (cell_y, cell_x)

    def cell_to_real_position(self, cell_y: int, cell_x: int) -> tuple[float, float]:
        # Map cell centers to my metric frame.
        real_x = -28.5 - (cell_x * self.CELL_SIZE)
        real_y = 28.5 + (cell_y * self.CELL_SIZE)
        return real_x, real_y

    def get_actual_position_from_print(self) -> tuple[float, float, float]:
        # This part is a bit weird, for some reason read_data() was not working
        # the reason was reset_encoders function, but for some reason it worked with this one
        # therefore i didnt change but only using read_data is enough, the name of the function is same
        
        #import io
        #from contextlib import redirect_stdout
        #f = io.StringIO()
        #with redirect_stdout(f):
        #    print_position()
        #line = f.getvalue().strip()
        #if "The location is" in line:
        #    parts = line.split("The location is")[1].split(",")
        #    x = float(parts[0].strip())
        #    y = float(parts[1].strip())
        #    theta = float(parts[2].split("°")[0].strip())
        #    return x, y, theta
      
        read_data()
        return position["x"], position["y"], math.degrees(position["theta"])

    def get_distance_to_cell_center(self, target_cell: Cell) -> float:
        # calculating the distance to target centre
        cx, cy, _ = self.get_actual_position_from_print()
        tx, ty = self.cell_to_real_position(target_cell[0], target_cell[1])
        return math.hypot(tx - cx, ty - cy)

    def get_distance_travelled(self, start_x: float, start_y: float) -> float:
        cx, cy, _ = self.get_actual_position_from_print()
        return math.hypot(cx - start_x, cy - start_y)

    def is_at_target(self) -> bool:
        # checking if it is at the target as the name suggests
        return self.real_position_to_cell() == tuple(self.target_pos) or \
               self.get_distance_to_cell_center(tuple(self.target_pos)) <= self.POSITION_TOLERANCE

    def is_in_cell(self, target_cell: Cell) -> bool:
        return self.get_distance_to_cell_center(target_cell) <=self.POSITION_TOLERANCE

    def can_move(self, from_cell: Cell, to_cell: Cell) -> bool:
        # So this checks adjency since to make sure there is no wall between the 
        # current pos and the targeted pos
        fy, fx= from_cell
        ty, tx= to_cell
        if abs(ty-fy) + abs(tx-fx) != 1:
            return False
        if ty < 0 or ty >= self.maze.shape[0] or tx < 0 or tx >= self.maze.shape[1]:
            return False
        from_array_y = self.maze.shape[0] - 1 - fy
        from_array_x = fx
        # I read the wall bits directly; if there is a wall, I dont go.
        if tx > fx:  # East
            return not self.maze[from_array_y, from_array_x, 1]
        if tx < fx:   #West
            return not self.maze[from_array_y, from_array_x, 3]
        if ty > fy:   # north
            return not self.maze[from_array_y, from_array_x, 0]
        # south
        return not self.maze[from_array_y, from_array_x, 2]

    def get_neighbors(self, cell: Cell) -> list[Cell]:
        # FInd the neightbouts where we can go
        y, x = cell
        nbrs: list[Cell] = []
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nc =(y + dy, x + dx)
            if self.can_move(cell, nc):
                nbrs.append(nc)
        return nbrs
    
    # No heuristics, it is kinda like BFS
    # it uses FIFO to add or remove elelmetns(first in first out)
    # track parent pointer to reconstruct 
    # also all the algorithm beloves takes connectivty from can_move
    def flood_fill(self, start: Cell, goal: Cell) -> list[Cell] | None:
        if start == goal:
            return [start]
        distances = {start: 0}
        parent = {}
        q = deque([start])
        while q:
            cur = q.popleft()
            for n in self.get_neighbors(cur):  # neighbors allowed by walls
                if n not in distances:
                    distances[n] = distances[cur] + 1
                    parent[n] = cur
                    if n == goal:
                        # Reconstruct via parent cells
                        path = [n]
                        while path[-1] in parent:
                            path.append(parent[path[-1]])
                        return list(reversed(path))
                    q.append(n)
        return None

    # This is like a weighted BFS, since all the costs are 1, this could be changed
    def dijkstra(self, start: Cell, goal: Cell) -> list[Cell] | None:
        dist = {start: 0}
        prev = {}
        pq = [(0, start)]
        vis = set()
        while pq:
            d,cur =heapq.heappop(pq)
            if cur in vis:
                continue
            vis.add(cur)
            if cur == goal:
                path = [cur] # reconstrust from goal to start
                while path[-1] in prev:
                    path.append(prev[path[-1]])
                return list(reversed(path))
            for n in self.get_neighbors(cur):
                nd = d +1 # this could be changed, CHECK !!!
                if n not in dist or nd <dist[n]:
                    dist[n] = nd
                    prev[n] = cur
                    heapq.heappush(pq,(nd,n))
        return None
    # Only one with a heuristic, even though it is not likely to see
    # it supposed to be faster and less reliable
    def a_star(self, start: Cell, goal: Cell) -> list[Cell] | None:
        if start == goal:
            return [start]
        def h(a: Cell, b: Cell) -> int:
            return abs(a[0] -b[0]) +abs(a[1] - b[1]) # manhattan heursitic used because it is a 2D grid like maze
        openq = [(0, start)]
        came = {}
        g = {start: 0}
        f = {start: h(start, goal)}
        while openq:
            cur = heapq.heappop(openq)[1]
            if cur== goal: # reconstruct
                path = [cur]
                while path[-1] in came:
                    path.append(came[path[-1]])
                return list(reversed(path))
            for n in self.get_neighbors(cur):
                tg = g[cur] + 1 # might be changed if the weights are added
                if n not in g or tg < g[n]:
                    came[n] = cur
                    g[n] = tg
                    f[n] = tg + h(n, goal) # f = g+h
                    heapq.heappush(openq,(f[n], n))
        return None

    def get_target_heading_for_direction(self, from_cell: Cell, to_cell: Cell) -> float:
        fy, fx = from_cell
        ty, tx = to_cell
        # like mentioned before, order is east wwest north and south
        if tx > fx:  
            return -90
        if tx < fx: 
            return 90
        if ty > fy: 
            return 0
        return 180 

    def correct_heading_after_movement(self, expected_heading: float) -> None:
        # After robot moves forward, make it look straightr by using IMU and rotate function
        cur = self.get_imu_heading_normalized()
        err = self.normalize_angle_180(expected_heading-cur)
        if abs(err) >1.0:
            rotate(err)
            time.sleep(0.3)

    def check_and_correct_heading_after_movement(self, expected_heading: float) -> bool:
        if not self.post_movement_imu_check:
            return False
        cur = self.get_imu_heading_normalized()
        err = self.normalize_angle_180(expected_heading - cur)
        if abs(err) > self.post_movement_threshold:
            corr = max(-15.0, min(15.0,err * 0.6))
            rotate(corr)
            time.sleep(0.4)
            return True
        return False

    def log_status_brief(self) -> None:
        # During operation I only print the cell and IMU headings
        cell = self.real_position_to_cell()
        imu = self.get_imu_heading_normalized()
        print(f"Cell: {cell} | IMU: {imu:.1f}")

    def forward(
        self,
        target_cell: Cell,
        expected_heading: float,
        target_distance: float | None = None,
    ) -> bool:
        # So basically, go some distnce then check how much it is, is ther undershooting or overshooting
        if target_distance is None:
            target_distance = self.CELL_SIZE
        read_data()
        move_start_x = position["x"]
        move_start_y = position["y"]

        # First go
        move_for_time(MOVE_FORWARD, duration_seconds=self.current_move_time)
        actual = self.get_distance_travelled(move_start_x, move_start_y)

        # small arrangements application
        attempts = 0
        while actual < min(target_distance, self.get_distance_to_cell_center(target_cell)) and attempts < 5:
            remaining = target_distance - actual
            corr_t = max(0.2, min(1.0, (remaining / max(target_distance, 1e-6)) * self.current_move_time))
            read_data()
            cx, cy = position["x"], position["y"]
            move_for_time(MOVE_FORWARD, duration_seconds=corr_t)
            actual += self.get_distance_travelled(cx, cy)
            attempts += 1


        self.correct_heading_after_movement(expected_heading)
        self.log_status_brief()

        # For the nect movement, adjust the timing variable
        self.adjust_movement_timing_with_overshoot(actual, target_distance, 0)

        final_cell = self.real_position_to_cell()
        final_dist = self.get_distance_to_cell_center(target_cell)
        success = (final_cell == target_cell) or (final_dist <=self.POSITION_TOLERANCE) or(actual >= 0.8 * target_distance)
        self.consecutive_failed_moves = 0 if success else self.consecutive_failed_moves + 1
        return success

    def adjust_movement_timing_with_overshoot(
        self,
        actual_distance: float,
        target_distance: float,
        overshoot_adjustment: float,
    ) -> None:
        # if it is short, increase time if it overshoots, reduce a bit.
        ratio = (actual_distance / target_distance) if target_distance > 0 else 1.0
        if overshoot_adjustment >0:
            self.current_move_time = max(1.0,self.current_move_time - overshoot_adjustment)
        if ratio <0.8:
            self.current_move_time = min(5.0,self.current_move_time+self.time_adjustment_factor *(0.8 - ratio))
        elif ratio > 1.2:
            self.current_move_time = max(1.0,self.current_move_time-self.time_adjustment_factor * (ratio - 1.2))

    def move_to_adjacent_cell_with_imu(self, target_cell: Cell) -> bool:
        cur_cell = self.real_position_to_cell()
        if cur_cell == target_cell or self.is_in_cell(target_cell):
            self.log_status_brief()
            return True
        if not self.can_move(cur_cell, target_cell):
            return False

        tgt_heading = self.get_target_heading_for_direction(cur_cell, target_cell) # face the right way first
        imu_now = self.get_imu_heading_normalized()
        err = self.normalize_angle_180(tgt_heading -imu_now)
        if abs(err) > 15:
            rotate(err)
            time.sleep(0.6)
        self.expected_heading = tgt_heading

        # Then go step forward
        ok = self.forward(target_cell, self.expected_heading)
        return ok

    def move_to_adjacent_cell(self, target_cell: Cell) -> bool:
        # Well this is useless now, but changing everthing takes time, so yeah, initial idea was maybe another alternative to IMU
        return self.move_to_adjacent_cell_with_imu(target_cell)

    def execute_path_with_corrections(self, initial_path: list[Cell]) -> bool:
        if len(initial_path) <= 1:
            self.log_status_brief()
            return True
        path =initial_path.copy()
        step = 0
        start_time = time.time()
        while step<len(path) - 1:
            if self.is_at_target():
                self.log_status_brief()
                return True
            if self.consecutive_failed_moves>= self.MAX_FAILED_MOVES: # If there is a morre than failed moves allowed, robot can not find right orientation, hence that run is considered as a failure
                return False

            actual_cell = self.real_position_to_cell()
            expected_cell = path[step]
            next_cell = path[step+1]

            # Interactive mode
            if self.interactive_mode:
                ans = input("Next step (y/n): ").strip().lower()
                if ans not in ("y"):
                    return False

            #If robot drifts replan the the thing
            # I used replanning with A* however any of them could work, the thing is, I had never have the
            #need of replan because maze is small and it either stucked to a wall or found its place,
            #inital path part is more important for this task
            cell_dist = abs(actual_cell[0] -expected_cell[0]) +abs(actual_cell[1] - expected_cell[1])
            if cell_dist > 2 and not self.is_in_cell(expected_cell):
                self.replanning_count +=1
                if self.replanning_count > self.MAX_REPLANNING_ATTEMPTS:
                    return False
                new_path = self.a_star(actual_cell,tuple(self.target_pos))
                if not new_path or len(new_path) <= 1:
                    return False
                path = new_path
                step = 0
                continue

            if not self.can_move(actual_cell, next_cell):
                new_path = self.a_star(actual_cell, tuple(self.target_pos))
                if not new_path or len(new_path) <= 1:
                    return False
                path = new_path
                step = 0
                continue

            if self.move_to_adjacent_cell(next_cell):
                step += 1
                self.consecutive_failed_moves = 0
            else:
                if self.consecutive_failed_moves >= 5:
                    actual_cell = self.real_position_to_cell()
                    new_path = self.a_star(actual_cell,tuple(self.target_pos))
                    if not new_path or len(new_path)<= 1:
                        return False
                    path = new_path
                    step = 0
            time.sleep(0.3)
            if time.time() - start_time > 600:
                return False
        return self.is_at_target()

    def load_available_maps(self) -> dict[str, dict[str, object]]:
        #A side note here it shows the maps however this is for feasibility
        # instead of maps you can try ../maps if reaching the gitlab account
        maps_folder = "maps"
        available = {}
        if not os.path.exists(maps_folder):
            return available
        for fn in os.listdir(maps_folder):
            if fn.endswith('.json'):
                with open(os.path.join(maps_folder, fn),'r') as f:
                    data = json.load(f)
                maze_array = np.array(data['maze'])
                available[fn[:-5]] = {
                    'maze': maze_array,
                    'description': data.get('description','No description'),
                    'size': f"{maze_array.shape[0]}x{maze_array.shape[1]}"
                }
        return available

    def select_map(self) -> bool:
        # basically choose one of the saved maps, if not chosen give warning to uver
        available = self.load_available_maps()
        if not available:
            print("No maps")
            return False
        print("\nMAPS:")
        names = list(available.keys())
        for i, name in enumerate(names, 1):
            info = available[name]
            print(f"{i}. {name}")
        while True:
            try:
                idx = int(input(f"Select map (1-{len(names)}): ")) - 1
                if 0 <= idx< len(names):
                    sel = names[idx]
                    self.maze = available[sel]['maze']
                    return True
            except ValueError:
                pass
            print("Invalid")

    def run_hardware(self) -> None:
        # In the begiinning IMU angle is 0, techically fluctuates between -2 and 2
        start_heading_tracker()
        update_speed(new_speed=110,new_turn=110) # Speeds 110-130 works, any more pi controller goes a bit off, max limit of speed = 130
        # So, it starts from here because reference lcoation is taken as the camera point of the rover, and -x means 28.5 to the right of the left wall
        position["x"], position["y"], position["theta"] = -28.5, 28.5, 0
        start_pos =(0, 0)
        self.expected_heading = 0

        # Map selection
        if not self.select_map():
            return
        h,w = self.maze.shape[:2]

        # Target selection
        while True:
            try:
                target_input = input(f"Enter target (row,col) [0-{h-1},0-{w-1}]: ")
                ty,tx = map(int, target_input.split(','))
                if 0 <= ty < h and 0 <= tx < w and (ty, tx) != start_pos:
                    self.target_pos =[ty, tx]
                    break
            except ValueError:
                pass
            print("Error")

        # Algorithm selection
        print("\nALGORITHM: 1) Flood Fill  2) Dijkstra  3) A*")
        while True:
            try:
                choice = int(input("Choose (1-3): "))
                if 1 <= choice <= 3:
                    break
            except ValueError:
                pass
            print("Invalid")

        # Interactive mode toggle
        ans = input("Interactive mode?").strip().lower()
        self.interactive_mode = ans in ("y", "yes")

        # Early exit if already there
        if self.is_at_target():
            self.log_status_brief()
            return

        # Planning and choosing the algorithm 
        def run_and_exec(
            plan_name: str,
            fn: Callable[[Cell, Cell], list[Cell] | None],
        ) -> tuple[bool, list[Cell] | None]:
            p = fn(start_pos, tuple(self.target_pos))
            if not p:
                return False, None
            # by this below, all the path planning results tested 
            print("Planned moves:", ", ".join(["N" if b[0] > a[0] else "S" if b[0] < a[0] else "E" if b[1] > a[1] else "W" for a, b in zip(p, p[1:])]))
            # Countdown before execution, to go from desk to maze, 3 sec
            go = input("Execute path now? ").strip().lower() in ("y", "yes")
            if not go:
                return False, p
            return self.execute_path_with_corrections(p), p

        success = False
        if choice == 1:
            success, _ =run_and_exec("Flood Fill", self.flood_fill)
        elif choice ==2:
            success, _ =run_and_exec("Dijkstra", self.dijkstra)
        elif choice== 3:
            success, _ =run_and_exec("A*", self.a_star)
     

        # Final quiet status
        self.log_status_brief()
        if not success:
            print("Mission Fail")


def emergency_stop() -> None:
    # Stop the robot
    send_command(MOVE_STOP)


def main() -> None:
    solver = HardwareMazeSolver()
    try:
        # Wait a bit
        t0 = time.time()
        while time.time() - t0<3:
            solver.log_status_brief()
            time.sleep(0.5)
        solver.run_hardware()
    except KeyboardInterrupt:
        emergency_stop()
    finally:
        emergency_stop()


if __name__ == "__main__":
    main()
