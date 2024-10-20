import numpy as np
from rrt_star import RRTStar, visualize_path  # Import the RRTStar class and visualize_path function
from grid_map import create_grid_map

def main():
    start = (-59.55459976196289, -253.5229949951172)  # Choose one of the valid points
    goal = (-57.65459976196289, -253.5229949951172)  # Choose another valid point
    ply_file = '3dmap.ply'  # Update this path to your actual file
    grid_resolution = 0.05  # Adjust based on your requirements

    # Create the occupancy grid and boundaries
    occupancy_grid, boundaries = create_grid_map(ply_file, grid_resolution)

    # Initialize RRT* planner
    rrt_star = RRTStar(start, goal, occupancy_grid, boundaries)

    # Plan the path
    nodes, edges = rrt_star.plan()

    # Visualize the path and save as a PNG
    visualize_path(nodes, edges, filename='rrt_star_path.png')  # Save as 'rrt_star_path.png'

if __name__ == "__main__":
    main()

