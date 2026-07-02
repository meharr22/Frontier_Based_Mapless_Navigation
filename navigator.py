from collections import deque
import numpy as np
from world import MOVES, FREE, WALL, scan_to_cells
class Navigator:
    def __init__(self, shape, start, goal):
        """
        Initialize the robot's navigation state and planning data structures.
        """
        self.shape = shape
        self.start = start
        self.goal = goal
        self.known = {}
        self.visited = set()
        self.current_path = []
        self.current_target = None
        self.steps_since_plan = 0
        self.REPLAN_INTERVAL = 5
    def neighbors(self, cell):
        """
        Generate all valid neighbouring cells within map boundaries.
        """
        r, c = cell
        for dr, dc in MOVES.values():
            nr = r + dr
            nc = c + dc
            if 0 <= nr < self.shape[0] and 0 <= nc < self.shape[1]:
                yield (nr, nc)
    def bfs(self, start, goal):
        """
        Compute the shortest path between two cells using
        Breadth-First Search over the discovered free space.
        Returns the path as a list of cells, or None if unreachable.
        """
        queue = deque([start])
        visited = {start}
        parent = {}
        while queue:
            current = queue.popleft()
            if current == goal:
                path = []
                while current != start:
                    path.append(current)
                    current = parent[current]
                path.reverse()
                return path
            for nxt_cell in self.neighbors(current):
                if nxt_cell in visited:
                    continue
                if self.known.get(nxt_cell) != FREE:
                    continue
                visited.add(nxt_cell)
                parent[nxt_cell] = current
                queue.append(nxt_cell)
        return None
    def find_frontiers(self):
        """
        Identify frontier cells.
        A frontier is a discovered free cell that borders at least
        one unexplored cell.
        """
        frontiers = []
        for cell, state in self.known.items():
            if state != FREE:
                continue
            for nxt_cell in self.neighbors(cell):
                if nxt_cell not in self.known:
                    frontiers.append(cell)
                    break
        return frontiers
    def cluster_frontiers(self):
        """
        Group neighbouring frontier cells into connected frontier
        clusters using BFS.
        """
        frontiers = set(self.find_frontiers())
        clusters = []
        while frontiers:
            start = frontiers.pop()
            cluster = [start]
            queue = deque([start])
            while queue:
                cell = queue.popleft()
                for nxt_cell in self.neighbors(cell):
                    if nxt_cell in frontiers:
                        frontiers.remove(nxt_cell)
                        cluster.append(nxt_cell)
                        queue.append(nxt_cell)
            clusters.append(cluster)
        return clusters
    def select_cluster(self, pose):
        """
        Select the most promising frontier cluster based on a
        utility function that balances exploration potential
        against travel cost.
        """
        clusters = self.cluster_frontiers()
        best = None
        best_score = -float("inf")
        for cluster in clusters:
            representative = min(
                cluster,
                key=lambda cell: abs(cell[0] - pose[0]) + abs(cell[1] - pose[1])
            )
            path = self.bfs(pose, representative)
            if path is None:
                continue
            cluster_size = len(cluster)
            path_cost = len(path)
            goal_distance = (
                abs(representative[0] - self.goal[0]) +
                abs(representative[1] - self.goal[1])
            )
            score = (
                2.5 * cluster_size
                - 1.0 * path_cost
                - 0.2 * goal_distance
            )
            if score > best_score:
                best_score = score
                best = (representative, path)
        return best
    def target_is_valid(self):
        """
        Check whether the current target is still a valid frontier.
        """
        if self.current_target is None:
            return False
        return self.current_target in self.find_frontiers()
    def plan(self, pose):
        """
        Plan a new path either towards the goal
        or towards the best frontier cluster.
        """
        if self.known.get(self.goal) == FREE:
            goal_path = self.bfs(pose, self.goal)
            if goal_path is not None:
                self.current_target = self.goal
                self.current_path = goal_path
                return
        choice = self.select_cluster(pose)
        if choice is None:
            self.current_target = None
            self.current_path = []
            return
        target, path = choice
        self.current_target = target
        self.current_path = path
    def act(self, obs):
        """
         Process the latest LiDAR observation and decide the robot's next action.
        """
        pose = obs["pose"]
        free_cells, occupied_cells = scan_to_cells(pose, obs["scan"])
        for cell in free_cells:
            self.known[cell] = FREE
        for cell in occupied_cells:
            self.known[cell] = WALL
        self.visited.add(pose)
        if pose == self.current_target:
            self.current_path = []
        need_plan = (
            not self.current_path
            or self.steps_since_plan >= self.REPLAN_INTERVAL
        )
        if need_plan:
            self.plan(pose)
            self.steps_since_plan = 0
        if (
        not self.current_path
        or not self.target_is_valid()
    ):
            self.plan(pose)
        nxt_cell = self.current_path.pop(0)
        dr = nxt_cell[0] - pose[0]
        dc = nxt_cell[1] - pose[1]
        for action, (r, c) in MOVES.items():
            if (dr, dc) == (r, c):
                self.steps_since_plan += 1
                return action
        return np.random.choice(list(MOVES))
