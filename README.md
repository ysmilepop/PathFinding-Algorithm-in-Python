# CSEG LAB EXAM
## 1. Overview of the Algorithms Implemented
 This project implements two popular pathfinding algorithms to find the shortest path between a start and goal point on a grid:
  ### **Greedy Best-First Search (GBFS)**:
   * This algorithm uses a heuristic to select the next node based only on the estimated distance to the goal, without considering the path cost.
  ### **A Search**:
   * A* is a refined algorithm that combines the actual cost to reach a node and the heuristic estimate to the goal, unlike GBFS, which only uses the heuristic.
 Both algorithms aim to find a path through a grid where obstacles are represented by 1 and free spaces by 0. The search proceeds by considering valid neighbors (up, down, left, right) and avoiding obstacles.

## 2. Instructions on How to Run the Code
  To run the pathfinding algorithms:
   ### 1. Make sure you have Python installed on your system.
   ### 2. Steps to Run:
   * Clone or download the repository.
   * Navigate to the folder where the script is located using a terminal or command prompt
   #### Run the script using
     python main.py
   ### The program will:
   *Execute both the Greedy Best-First Search and A Search* algorithms.
   * Print the paths found by each algorithm (if a path exists).
   * Display the time taken for each algorithm to run.
   * Compare the performance and path quality of both algorithms.
       
  ## 3. Description of the Approach
  ### Grid Representation:
  * A 2D list represents the grid, with 0 for open space and 1 for obstacles. Start and goal positions are given as (x, y) tuples.
  ### Algorithms:
  * Greedy Best-First Search: Uses only the heuristic to select the next node closest to the goal.
  * A Search*: Uses both the actual cost and the heuristic to find the shortest path.
  ### Helper Functions:
  * get_neighbors: Returns valid neighboring nodes (up, down, left, right).
  * reconstruct_path: Reconstructs the path from start to goal by tracing node predecessors.
     
  ### Challenges Faced
  * Handling Obstacles:
     * One of the challenges was ensuring the algorithms correctly handled obstacles in the grid. We had to implement checker to ensure that
       a node isn't revisited if it has already been encountered and if it is blocked by an obstacle.
   * Efficient Path Reconstruction:
      * Reconstructing the path once the goal is reached required careful tracking of each node's predecessor (came_from). Ensuring that the path was
        reconstructed correctly without errors took some time and attention to detail.
   * Algorithm Comparison: 
      * Benchmarked the runtime and quality of paths for both algorithms. We used time.perf_counter() instead of time.time() because 
        it provides more precise timing, as the actual time wasn't showing correctly with time.time().
