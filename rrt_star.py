import numpy as np
import matplotlib.pyplot as plt
from grid_map import create_grid_map

class RRTStar:
    def __init__(self, start, goal, occupancy_grid, boundaries, max_iter=500, step_size=0.05, neighbor_radius=0.1):
        self.start = start
        self.goal = goal
        self.occupancy_grid = occupancy_grid
        self.boundaries = boundaries
        self.max_iter = max_iter
        self.step_size = step_size
        self.neighbor_radius = neighbor_radius
        self.nodes = [start]
        self.edges = []

    def plan(self):
        for _ in range(self.max_iter):
            rand_point = self.get_random_point()
            nearest_node = self.nearest_neighbor(rand_point)
            new_node = self.steer(nearest_node, rand_point)

            if self.is_free(new_node):
                self.nodes.append(new_node)
                self.edges.append((nearest_node, new_node))

                # Rewire the tree
                self.rewire(new_node)

                # Print the new node for debugging
                print(f"New node created: {new_node}")

            if np.linalg.norm(np.array(new_node) - np.array(self.goal)) < self.step_size:
                print("Goal reached!")
                break

        return self.nodes, self.edges

    def get_random_point(self):
        min_x, max_x, min_y, max_y = self.boundaries
        rand_x = np.random.uniform(min_x, max_x)
        rand_y = np.random.uniform(min_y, max_y)
        return (rand_x, rand_y)

    def nearest_neighbor(self, point):
        return min(self.nodes, key=lambda node: np.linalg.norm(np.array(node) - np.array(point)))

    def steer(self, from_node, to_point):
        direction = np.array(to_point) - np.array(from_node)
        distance = np.linalg.norm(direction)
        if distance > self.step_size:
            direction = direction / distance * self.step_size
        new_node = np.array(from_node) + direction
        return (new_node[0], new_node[1])

    def is_free(self, node):
        x_idx, y_idx = self.to_grid_index(node)
        return 0 <= x_idx < self.occupancy_grid.shape[0] and 0 <= y_idx < self.occupancy_grid.shape[1] and not self.occupancy_grid[x_idx, y_idx]

    def to_grid_index(self, node):
        min_x, min_y = self.boundaries[0], self.boundaries[2]
        x_idx = int((node[0] - min_x) / self.step_size)
        y_idx = int((node[1] - min_y) / self.step_size)
        return (x_idx, y_idx)

    def rewire(self, new_node):
        for node in self.nodes:
            if node != new_node:
                if np.linalg.norm(np.array(node) - np.array(new_node)) < self.neighbor_radius:
                    if self.is_free(node):
                        self.edges.append((node, new_node))

def visualize_path(nodes, edges, filename='path.png'):
    plt.figure(figsize=(10, 10))
    import matplotlib.pyplot as plt

def visualize_path(nodes, edges, occupancy_grid, filename='path_visualization.png'):
    plt.figure(figsize=(10, 10))
    for edge in edges:
        plt.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], color='blue')
    plt.scatter(*zip(*nodes), color='red')
    plt.title("RRT* Path Planning")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.savefig(filename)  # Save the figure as a PNG file
    plt.close()  # Close the figure

