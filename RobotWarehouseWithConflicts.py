import random
import networkx as nx
import matplotlib.pyplot as plt
from collections import defaultdict
from queue import PriorityQueue


class RobotFleetSimulation:
    def __init__(self, grid_size, num_robots):
        self.grid_size = grid_size
        self.graph = self._create_graph()
        self.num_robots = num_robots
        self.robots = {}  # Robot positions
        self.goals = {}  # Robot goals
        self.priorities = {}  # Robot priorities
        self.paths = {}  # Current paths of robots
        self.path_history = defaultdict(list)  # Path history
        self.conflicts = []

    def _create_graph(self):
        graph = nx.grid_2d_graph(self.grid_size, self.grid_size)
        for edge in graph.edges:
            graph.edges[edge]["weight"] = 1
        nx.set_node_attributes(graph, False, "occupied")
        return graph

    def spawn_robots(self):
        for i in range(self.num_robots):
            while True:
                start_node = (random.randint(0, self.grid_size - 1),
                              random.randint(0, self.grid_size - 1))
                if not self.graph.nodes[start_node]["occupied"]:
                    self.robots[f"Robot_{i}"] = start_node
                    self.graph.nodes[start_node]["occupied"] = True
                    self.priorities[f"Robot_{i}"] = random.randint(1, 10)
                    break

    def generate_goals(self):
        for robot in self.robots:
            while True:
                goal_node = (random.randint(0, self.grid_size - 1),
                             random.randint(0, self.grid_size - 1))
                if not self.graph.nodes[goal_node]["occupied"] and goal_node != \
                        self.robots[robot]:
                    self.goals[robot] = goal_node
                    break

    def compute_paths(self):
        for robot, position in self.robots.items():
            goal = self.goals[robot]
            try:
                path = nx.shortest_path(self.graph, position, goal,
                                        weight="weight")
                self.paths[robot] = path
            except nx.NetworkXNoPath:
                print(f"No path found for {robot} from {position} to {goal}")

    def bfs_alternative_path(self, start, goal):
        queue = [(start, [start])]
        visited = set()
        while queue:
            current_node, path = queue.pop(0)
            if current_node == goal:
                return path
            for neighbor in self.graph.neighbors(current_node):
                if neighbor not in visited and not self.graph.nodes[neighbor][
                    "occupied"]:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return None

    def move_robot(self, robot):
        if robot in self.paths and len(self.paths[robot]) > 1:
            current_position = self.robots[robot]
            next_position = self.paths[robot][1]
            if not self.graph.nodes[next_position]["occupied"]:
                self.graph.nodes[current_position]["occupied"] = False
                self.graph.nodes[next_position]["occupied"] = True
                self.robots[robot] = next_position
                self.path_history[robot].append(next_position)

    # Conflict Resolution: Move robots based on priorities and resolve conflicts
    def resolve_conflicts(self):
        pq = PriorityQueue()
        for robot in self.robots:
            pq.put((self.priorities[robot], robot))

        # While conflicts exist, try to move robots based on priority
        while not pq.empty():
            priority, robot = pq.get()
            print(f"Moving {robot} with priority {priority}")

            # If there is a conflict, try to find an alternative path
            current_position = self.robots[robot]
            if self.graph.nodes[current_position]["occupied"]:
                alternative_path = self.bfs_alternative_path(current_position,
                                                             self.goals[robot])
                if alternative_path:
                    self.paths[robot] = alternative_path
                else:
                    print(f"{robot} is stuck, no alternative path found!")

            self.move_robot(robot)

    # DFS to detect deadlocks: Identify robots that are stuck in cycles
    def detect_deadlocks(self):
        def dfs(robot_position, visited, stack):
            # Add current position to visited and stack
            visited.add(robot_position)
            stack.add(robot_position)

            # Get the neighbors of the current position
            for neighbor in self.graph.neighbors(robot_position):
                # If the neighbor is in the stack, it's a cycle (deadlock)
                if neighbor in stack:
                    print(
                        f"Deadlock detected: Robot at position {robot_position} is part of a cycle.")
                    return True
                if neighbor not in visited:
                    if dfs(neighbor, visited, stack):
                        return True

            # Remove from the stack when done exploring
            stack.remove(robot_position)
            return False

        visited = set()
        stack = set()
        for robot, position in self.robots.items():
            if position not in visited:
                if dfs(position, visited, stack):
                    return True
        return False

    def detect_conflicts(self):
        self.conflicts = []
        node_conflicts = defaultdict(list)
        for robot, position in self.robots.items():
            node_conflicts[position].append(robot)

        for node, robots in node_conflicts.items():
            if len(robots) > 1:
                self.conflicts.append(
                    {"type": "Node Conflict", "node": node, "robots": robots})

        future_positions = defaultdict(list)
        for robot, path in self.paths.items():
            if len(path) > 1:
                future_positions[path[1]].append(robot)
        for node, robots in future_positions.items():
            if len(robots) > 1:
                self.conflicts.append(
                    {"type": "Path Conflict", "node": node, "robots": robots})

    def assign_new_goals(self):
        for robot in list(self.robots.keys()):
            if self.robots[robot] == self.goals[robot]:
                print(
                    f"{robot} reached its goal at {self.goals[robot]}. Assigning a new goal.")
                self.generate_goals()
                self.compute_paths()

    def log_path_history(self):
        for robot, history in self.path_history.items():
            print(f"Path history for {robot}: {history}")

    def simulate(self, steps):
        self.spawn_robots()
        self.generate_goals()  # Ensure goals are generated for all robots
        self.compute_paths()

        # Forcing specific positions to increase chances of conflict and deadlock
        self.robots = {
            "Robot_0": (2, 2),
            "Robot_1": (2, 3),
            "Robot_2": (3, 2),
            "Robot_3": (3, 3),
            "Robot_4": (2, 4)
        }
        self.goals = {
            "Robot_0": (3, 3),
            "Robot_1": (3, 2),
            "Robot_2": (2, 3),
            "Robot_3": (2, 2),
            "Robot_4": (3, 4)
        }
        self.compute_paths()

        for step in range(steps):
            print(f"\nStep {step + 1}:\n")

            # Force a type of conflict in each step if not naturally occurring
            if step == 0:
                self.force_conflict("node")
            elif step == 1:
                self.force_conflict("path")
            elif step == 2:
                self.force_conflict("deadlock")

            for robot in self.robots:
                self.move_robot(robot)

            self.detect_conflicts()
            self.resolve_conflicts()
            self.assign_new_goals()

            print(f"Robot positions: {self.robots}")
            print(f"Robot goals: {self.goals}")
            print(f"Robot paths: {self.paths}")
            print(f"Conflicts: {self.conflicts}")

            self.log_path_history()

            # Check if robots have reached their goals or are stuck
            if all(self.robots[robot] == self.goals[robot] for robot in
                   self.robots):
                print("All robots have reached their goals!")
                break

            # Detect deadlocks
            deadlocks = self.detect_deadlocks()
            if deadlocks:
                print(f"Deadlocks detected: {deadlocks}")
            else:
                print("No deadlocks detected.")

    def force_conflict(self, conflict_type):
        if conflict_type == "node":
            self.robots['Robot_0'] = (2, 2)
            self.robots['Robot_1'] = (2, 3)
            self.robots['Robot_2'] = (2, 4)
            self.robots['Robot_3'] = (2, 5)
            self.robots['Robot_4'] = (2, 6)
            self.goals['Robot_0'] = (2, 3)
            self.goals['Robot_1'] = (2, 4)
            self.goals['Robot_2'] = (2, 5)
            self.goals['Robot_3'] = (2, 6)
            self.goals['Robot_4'] = (2, 7)
        elif conflict_type == "path":
            self.robots['Robot_0'] = (2, 2)
            self.goals['Robot_0'] = (4, 4)
            self.robots['Robot_1'] = (4, 2)
            self.goals['Robot_1'] = (2, 4)
            self.robots['Robot_2'] = (5, 5)
            self.goals['Robot_2'] = (3, 3)
            self.robots['Robot_3'] = (3, 5)
            self.goals['Robot_3'] = (5, 3)
        elif conflict_type == "deadlock":
            self.robots['Robot_0'] = (3, 3)
            self.goals['Robot_0'] = (3, 4)
            self.robots['Robot_1'] = (3, 4)
            self.goals['Robot_1'] = (4, 4)
            self.robots['Robot_2'] = (4, 4)
            self.goals['Robot_2'] = (4, 3)
            self.robots['Robot_3'] = (4, 3)
            self.goals['Robot_3'] = (3, 3)

    def visualize(self):
        pos = {node: (node[1], -node[0]) for node in self.graph.nodes}
        nx.draw(self.graph, pos, with_labels=True, node_size=500,
                node_color="lightblue")
        for robot, position in self.robots.items():
            plt.text(pos[position][0], pos[position][1], robot, fontsize=8,
                     ha="center", va="center", color="red")
        plt.show()

    def interactive_visualization(self):
        plt.ion()
        pos = {node: (node[1], -node[0]) for node in self.graph.nodes}
        while True:
            plt.clf()
            nx.draw(self.graph, pos, with_labels=True, node_size=500,
                    node_color="lightblue")
            for robot, position in self.robots.items():
                plt.text(pos[position][0], pos[position][1], robot, fontsize=8,
                         ha="center", va="center", color="red")
            plt.pause(0.1)


# Run the simulation
simulator = RobotFleetSimulation(grid_size=10, num_robots=5)
simulator.simulate(steps=20)
simulator.visualize()
# Optionally, run interactive visualization
# simulator.interactive_visualization()
