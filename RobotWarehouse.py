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
        # Create a grid graph with edge weights
        graph = nx.grid_2d_graph(self.grid_size, self.grid_size)
        for edge in graph.edges:
            graph.edges[edge]["weight"] = 1  # Set all edge weights to 1
        nx.set_node_attributes(graph, False, "occupied")
        return graph

    def spawn_robots(self):
        # Assign initial positions and priorities to robots
        for i in range(self.num_robots):
            while True:
                start_node = (random.randint(0, self.grid_size - 1),
                              random.randint(0, self.grid_size - 1))
                if not self.graph.nodes[start_node]["occupied"]:
                    self.robots[f"Robot_{i}"] = start_node
                    self.graph.nodes[start_node]["occupied"] = True
                    self.priorities[f"Robot_{i}"] = random.randint(1,
                                                                   10)  # Assign random priority
                    break

    def generate_goals(self):
        # Assign random goals to each robot
        for robot in self.robots:
            while True:
                goal_node = (random.randint(0, self.grid_size - 1),
                             random.randint(0, self.grid_size - 1))
                if not self.graph.nodes[goal_node]["occupied"] and goal_node != \
                        self.robots[robot]:
                    self.goals[robot] = goal_node
                    break

    def compute_paths(self):
        # Compute shortest paths for each robot using Dijkstra's algorithm
        for robot, position in self.robots.items():
            goal = self.goals[robot]
            try:
                path = nx.shortest_path(self.graph, position, goal,
                                        weight="weight")
                self.paths[robot] = path
            except nx.NetworkXNoPath:
                print(f"No path found for {robot} from {position} to {goal}")

    def bfs_alternative_path(self, start, goal):
        # Perform a simple breadth-first search (BFS) for an alternative path
        queue = [(start, [start])]  # Start from initial position
        visited = set()
        while queue:
            current_node, path = queue.pop(0)
            if current_node == goal:
                return path  # Return path when goal is reached
            for neighbor in self.graph.neighbors(current_node):
                if neighbor not in visited and not self.graph.nodes[neighbor][
                    "occupied"]:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return None  # No path found

    def compute_paths_with_alternatives(self):
        for robot, position in self.robots.items():
            goal = self.goals[robot]
            try:
                path = nx.shortest_path(self.graph, position, goal,
                                        weight="weight")
                self.paths[robot] = path
            except nx.NetworkXNoPath:
                print(
                    f"No path found for {robot} from {position} to {goal}, recalculating...")
                # Try alternative pathfinding methods, e.g., Breadth-First Search (BFS)
                path = self.bfs_alternative_path(position, goal)
                if path:
                    self.paths[robot] = path
                else:
                    print(
                        f"Still no path found for {robot} after recalculation.")

    def move_robot(self, robot):
        # Move the robot one step along its path
        if robot in self.paths and len(self.paths[robot]) > 1:
            current_position = self.robots[robot]
            next_position = self.paths[robot][1]
            if not self.graph.nodes[next_position]["occupied"]:
                # Update positions
                self.graph.nodes[current_position]["occupied"] = False
                self.graph.nodes[next_position]["occupied"] = True
                self.robots[robot] = next_position
                self.path_history[robot].append(
                    next_position)  # Track path history

    def detect_conflicts(self):
        # Detect conflicts (node and path conflicts)
        self.conflicts = []
        node_conflicts = defaultdict(list)
        for robot, position in self.robots.items():
            node_conflicts[position].append(robot)

        # Node conflicts
        for node, robots in node_conflicts.items():
            if len(robots) > 1:
                self.conflicts.append(
                    {"type": "Node Conflict", "node": node, "robots": robots})

        # Path conflicts
        future_positions = defaultdict(list)
        for robot, path in self.paths.items():
            if len(path) > 1:
                future_positions[path[1]].append(robot)
        for node, robots in future_positions.items():
            if len(robots) > 1:
                self.conflicts.append(
                    {"type": "Path Conflict", "node": node, "robots": robots})

    def detect_deadlocks(self):
        # Detect deadlocks using DFS
        positions = list(
            self.robots.values())  # List of current positions of robots
        visited = set()  # Set to track visited nodes
        deadlocks = []  # Store deadlock cycles

        def dfs(node, path):
            if node in visited:
                return
            visited.add(node)
            path.append(node)
            # Look for the next robot blocking the path
            for next_robot, next_position in self.robots.items():
                if next_position == node and next_robot != robot:
                    # If cycle is formed, store it as a deadlock
                    if next_position in path:
                        deadlocks.append(path)
                    dfs(next_position, path.copy())

        # Perform DFS for every robot
        for robot in self.robots:
            dfs(self.robots[robot], [])
        return deadlocks

    def resolve_conflicts(self):
        # Resolve conflicts with priority queue
        pq = PriorityQueue()  # Create a priority queue

        # Put robots into the queue based on their priority
        for robot in self.robots:
            pq.put((self.priorities[robot], robot))

        # Resolve conflicts by moving robots based on priority
        while not pq.empty():
            priority, robot = pq.get()
            print(f"Moving {robot} with priority {priority}")
            self.move_robot(robot)  # Move robot based on priority

    def assign_new_goals(self):
        # Assign new goals to robots that have reached their destination
        for robot in list(self.robots.keys()):
            if self.robots[robot] == self.goals[robot]:
                print(
                    f"{robot} reached its goal at {self.goals[robot]}. Assigning a new goal.")
                self.generate_goals()
                self.compute_paths()

    def log_path_history(self):
        # Log the path history of robots
        for robot, history in self.path_history.items():
            print(f"Path history for {robot}: {history}")

    def simulate(self, steps):
        # Run the simulation
        self.spawn_robots()
        self.generate_goals()
        self.compute_paths()

        for step in range(steps):
            print(f"\nStep {step + 1}:\n")

            # Move robots and detect conflicts
            for robot in self.robots:
                self.move_robot(robot)

            self.detect_conflicts()
            self.resolve_conflicts()
            self.assign_new_goals()

            # Log current positions and paths
            print(f"Robot positions: {self.robots}")
            print(f"Robot goals: {self.goals}")
            print(f"Robot paths: {self.paths}")
            print(f"Conflicts: {self.conflicts}")

            # Log path history
            self.log_path_history()

            # Detect deadlocks
            deadlocks = self.detect_deadlocks()
            if deadlocks:
                print(f"Deadlocks detected: {deadlocks}")
            else:
                print("No deadlocks detected.")

            # Break if all robots are idle
            if not any(self.paths[robot] for robot in self.robots):
                print("All robots are idle.")
                break

    def visualize(self):
        # Visualize the graph with robot positions and paths
        pos = {node: (node[1], -node[0]) for node in self.graph.nodes}
        nx.draw(self.graph, pos, with_labels=True, node_size=500,
                node_color="lightblue")
        for robot, position in self.robots.items():
            plt.text(pos[position][0], pos[position][1], robot, fontsize=8,
                     ha="center", va="center", color="red")
        plt.show()

    def interactive_visualization(self):
        plt.ion()  # Interactive mode
        pos = {node: (node[1], -node[0]) for node in self.graph.nodes}
        while True:
            plt.clf()  # Clear the figure
            nx.draw(self.graph, pos, with_labels=True, node_size=500,
                    node_color="lightblue")
            for robot, position in self.robots.items():
                plt.text(pos[position][0], pos[position][1], robot, fontsize=8,
                         ha="center", va="center", color="red")
            plt.pause(0.1)  # Pause for real-time update


# Run the simulation
simulator = RobotFleetSimulation(grid_size=10, num_robots=5)
simulator.simulate(steps=20)
simulator.visualize()
# Optionally, run interactive visualization
# simulator.interactive_visualization()
