# Search
BFS and A* implementation

## Introduction

Implemented general-purpose search algorithms and use them to solve pacman-like maze puzzles. This projects has four parts.

- Breadth-first search, with one waypoint.
- A* search, with one waypoint.
- A* search, with many waypoints.
- Heuristic search, with many waypoints.

![image](https://github.com/amithachari/Search/assets/64373075/d504326c-9a44-4364-ae3e-39732e753b0d)
  
The blue dot represents the agent. You can move the agent, using the arrow keys, to trace out a path, shown in color.

### Breadth First Search
Implementation as a function bfs(_:) in search.py
python3 main.py data/part-1/tiny --search bfs
python3 main.py data/part-1/small --search bfs
python3 main.py data/part-1/no_obs --search bfs
python3 main.py data/part-1/open --search bfs

![BFS](https://github.com/amithachari/Search/assets/64373075/d3bc01ba-28a2-4a14-9b8a-9214e21a8edf)

### A*
Implementation as a function astar_single(_:) in search.py with the following signature:

def astar_single(maze):
Used heapq module for better data structure

Since all the test mazes contain only a single waypoint, used the Manhattan Distance from the agent's current position to the singular waypoint as the A* heuristic function. For two grid coordinates a and b, the manhattan distance is given by:

abs(a[0] - b[0]) + abs(a[1] - b[1])
Test implementation by running main.py as follows, replacing data/part-2/small as needed:

python3 main.py data/part-2/tiny --search astar_single

![Astar](https://github.com/amithachari/Search/assets/64373075/0a7d3735-9ee7-4e5e-93af-8407f945a540)

### A* for Multiple Waypoints
Now, consider the more general and harder problem of finding the shortest path through a maze while hitting multiple waypoints. Problem is to solve different mazes using A* search with an appropriate admissible heuristic that takes into account the multiple waypoints.

There is a beautiful heuristic that will not only find the optimal path through all the waypoints, but is admissable as well. The Minimum Spanning Tree (MST). Instead of computing the distances to each waypoint from the current position, it would be more helpful to obtain an estimate of the cost of reaching the rest of the unreached waypoints once we have reached one. Obtaining this estimate can be done with an MST: by constructing a graph where the vertices are the waypoints and each edge connecting w_i to w_j has weight manhattan_distance(w_i, w_j) for all pairs of vertices (w_i, w_j), the MST represents the approximate lowest cost path that connects all the waypoints. Since it strictly underestimates the cost of going through all the waypoints, this is an admissable heuristic.

def astar_multiple(maze):
Test implementation by running main.py as follows, replacing data/part-3/small as needed:

python3 main.py data/part-3/tiny --search astar_multiple

![Astar_multiple](https://github.com/amithachari/Search/assets/64373075/2e5834dc-6aa1-4b0d-a3c6-d9731da9dc9d)
