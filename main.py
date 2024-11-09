from queue import PriorityQueue
import time

# Define the grid, start, and goal position
grid = [
    [0, 0, 1, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 0, 1],
    [0, 1, 1, 0, 0],
    [0, 0, 0, 1, 0],
]

start = (0, 0)
goal = (4, 4)

# Define the heuristic function using Manhattan distance
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Node representation with coordinates, cost, and heuristic
class Node:
    def _init_(self, position, cost = 0, heuristic = 0):
        self.position = position
        self.cost = cost
        self.heuristic = heuristic

    def _lt_(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)
    
# Greedy Best-First Search algorithm
def greedy_best_first_search(grid, start, goal):
    open_set = PriorityQueue()
    open_set.put((heuristic(start, goal), start))
    came_from = {}
    visited = set()
    visited.add(start)

    while not open_set.empty():
        _, current = open_set.get()

        if current == goal:
            return reconstruct_path(came_from, current)
        
        for neighbor in get_neighbors(grid, current):
            if neighbor in visited:
                continue
            visited.add(neighbor)
            came_from[neighbor] = current
            open_set.put((heuristic(neighbor, goal), neighbor))

    return None # No path found

# A* Search algorithm
def a_star_search(grid, start, goal):
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from ={}
    cost_so_far = {start: 0}

    while not open_set.empty():
        _, current = open_set.get()

        if current == goal:
            return reconstruct_path(came_from, current)
        
        for neighbor in get_neighbors(grid, current):
            new_cost = cost_so_far[current] + 1 # Each move costs 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal)
                open_set.put((priority, neighbor))
                came_from[neighbor] = current

    return None # No path found

# Helper function to get neighbors (up, down, left, right) and check for obstacles
def get_neighbors(grid, position):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    neighbors = []
    for d in directions:
        neighbor = (position[0] + d[0], position[1] + d[1])
        if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]):
            if grid[neighbor[0]][neighbor[1]] == 0:
                neighbors.append(neighbor)
    return neighbors

# Helper function to reconstruct the path from start to goal
def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.append(current)
    path.reverse()
    return path

# Testing both algorithms
if __name__ == "__main__":
    # Measure Greedy Best-First Search time
    start_time = time.perf_counter()
    path_greedy = greedy_best_first_search(grid, start, goal)
    time_greedy = time.perf_counter() - start_time
    print("Greedy Best-First Search: ")
    print("Path found", path_greedy)
    print("Time taken: {:.6f} seconds".format(time_greedy))

    # Measure A* Search time
    start_time = time.perf_counter()
    path_a_star = a_star_search(grid, start, goal)
    time_a_star = time.perf_counter() - start_time
    print("\nA* Search:")
    print("Path found: ", path_a_star)
    print("Time taken: {:.6f} seconds".format(time_a_star))

    # Efficiency Comparison
    print("\nEfficiency Comparison:")
    print("Greedy Best-First Search took {:.6f} seconds".format(time_greedy))
    print("A* Search took {:.6f} seconds".format(time_a_star))

    #Path Quality Comparison
    print("\nPath Quality Comparison: ")
    print("Length of path found by Greedy:", len(path_greedy) if path_greedy else "No path found")
    print("Length of path found by A*:", len(path_a_star) if path_a_star else "No path found")

