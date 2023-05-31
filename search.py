# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# sols, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Kelvin Ma (kelvinm2@illinois.edu) on 01/24/2021
from collections import deque
from queue import PriorityQueue
import heapq

"""
This is the main entry point for MP3. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# Search should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,fast)


# Feel free to use the code below as you wish
# Initialize it with a list/tuple of objectives
# Call compute_mst_weight to get the weight of the MST with those objectives
# TODO: hint, you probably want to cache the MST value for sets of objectives you've already computed...
# Note that if you want to test one of your search methods, please make sure to return a blank list
#  for the other search methods otherwise the grader will not crash.


class MST:
    def __init__(self, objectives):
        self.elements = {key: None for key in objectives}

        # TODO: implement some distance between two objectives
        # ... either compute the shortest path between them, or just use the manhattan distance between the objectives
        self.distances   = {
                (i, j): self.distance(i, j)
                for i, j in self.cross(objectives)
            }

    def distance(self, i, j):
        cost = abs(i[0] - j[0]) + abs(i[1] - j[1])
        return cost

    # Prim's algorithm adds edges to the MST in sorted order as long as they don't create a cycle
    def compute_mst_weight(self):
        weight      = 0
        for distance, i, j in sorted((self.distances[(i, j)], i, j) for (i, j) in self.distances):
            if self.unify(i, j):
                weight += distance
        return weight

    # helper checks the root of a node, in the process flatten the path to the root
    def resolve(self, key):
        path = []
        root = key
        while self.elements[root] is not None:
            path.append(root)
            root = self.elements[root]
        for key in path:
            self.elements[key] = root
        return root

    # helper checks if the two elements have the same root they are part of the same tree
    # otherwise set the root of one to the other, connecting the trees
    def unify(self, a, b):
        ra = self.resolve(a)
        rb = self.resolve(b)
        if ra == rb:
            return False
        else:
            self.elements[rb] = ra
            return True

    # helper that gets all pairs i,j for a list of keys
    def cross(self, keys):
        return (x for y in (((i, j) for j in keys if i < j) for i in keys) for x in y)

def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    frontier = deque([])
    sol = {}
    explored = set()
    frontier.append(maze.start)
    explored.add(maze.start)
    sol[maze.start] = (0, 0)

    while len(frontier) > 0:
        curr = frontier.popleft()
        if curr == maze.waypoints[0]:
            break
        for neighbor in maze.neighbors(curr[0], curr[1]):
            if neighbor not in explored:
                frontier.append(neighbor)
                sol[neighbor] = curr
                explored.add(neighbor)

    curr = (maze.waypoints)[0]
    path = []
    while curr != (0, 0):
        path.append(curr)
        curr = sol[curr]

    path.reverse()
    return path

def astar_single(maze):
    """
    Runs A star for part 2 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    def manhattan_cost(end, neighbor):
        cost = abs(end[0] - neighbor[0]) + abs(end[1] - neighbor[1])
        return cost

    frontier = []
    visited = {}
    cost = {}

    start = maze.start
    end = maze.waypoints[0]
    heapq.heappush(frontier,(0.0 + manhattan_cost(end,start), maze.start))
    visited[start] = None
    cost[start] = 0
    path = deque()
    print(frontier[0][0])
    while len(frontier) > 0 or frontier[0][0] != end:
        current = heapq.heappop(frontier)[1]
        if current == end:
            break
        for neighbor in maze.neighbors(current[0], current[1]):
            new_cost = cost[current] + 1 + manhattan_cost(end, neighbor)
            if neighbor not in cost or new_cost < cost[neighbor]:
                heapq.heappush(frontier,(new_cost,neighbor))
                visited[neighbor] = current
                cost[neighbor] = cost[current] + 1

    current = maze.waypoints[0]
    while current != None:
        path.append(current)
        current = visited[current]
    path.reverse()
    return path

def find_closest_goal(neighbor, new_waypoints):
    closest = new_waypoints[0]
    length0 = abs(neighbor[0] - closest[0]) + abs(neighbor[1] - closest[1])
    for waypoint in new_waypoints:
        length = abs(neighbor[0] - waypoint[0]) + abs(neighbor[1] - waypoint[1])
        if length < length0:
            length0 = length
            closest = waypoint
    return closest, length0


def astar_multiple(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    frontier = []
    start = maze.start
    rem_waypoints = list(maze.waypoints)
    path = deque()

    visited = {start: start}
    count = {start: 0}

    flag = 0
    finalpath = deque()

    _, length = find_closest_goal(start, rem_waypoints)
    cost = 0.0 + length + MST(rem_waypoints).compute_mst_weight()
    heapq.heappush(frontier, (cost, start))

    while len(rem_waypoints) > 0:
        current = heapq.heappop(frontier)[1]

        if flag == 0:
            flag = 1
        if current in rem_waypoints:
            current2 = current

            while current2 != start:
                path.appendleft(current2)
                current2 = visited[current2]

            finalpath = finalpath + path

            path = deque()
            start = current
            rem_waypoints.remove(current)
            flag = 0
            visited = {current: current}
            frontier = []

        for neighbor in maze.neighbors(current[0], current[1]):

            # print(cost_MST)
            if len(rem_waypoints) == 0:
                finalpath.appendleft(maze.start)
                print(finalpath)
                return finalpath
            _, length = find_closest_goal(neighbor, rem_waypoints)

            priority = length + MST(rem_waypoints).compute_mst_weight() + count[current] + 1
            if neighbor not in visited or count[neighbor] > count[current] + 1:
                visited[neighbor] = current
                count[neighbor] = count[current]
                heapq.heappush(frontier, (priority, neighbor))
    return []


#
# def astar_multiple(maze):
#     """
#     Runs A star for part 3 of the assignment in the case where there are
#     multiple objectives.
#
#     @param maze: The maze to execute the search on.
#
#     @return path: a list of tuples containing the coordinates of each state in the computed path
#     """
#
#     start = maze.start
#     rem_waypoints = list(maze.waypoints)
#     frontier = []
#     cost = {}
#     visited = {}
#     mst_map = {}
#     cost[start] = 0
#     order_waypoints = []
#     # print(type(waypoints))
#     start_state = start, rem_waypoints
#     visited[start] = start, None    #pos, parent
#     start_cost = cost[start] + MST(start_state[1]).compute_mst_weight()
#     heapq.heappush(frontier,(start_cost,start_state))
#     last_waypoint = None
#     mst_map[tuple(start_state[1])] = MST(start_state[1]).compute_mst_weight()
#     print(frontier)
#     path = deque()
#
#     while frontier:
#         current = heapq.heappop(frontier)
#         current_state = current[1][0], rem_waypoints
#         print("Current",current[1][0])
#         if not current[1][1]: #current[1][1] is rem_waypoints
#             last_waypoint = current_state
#             break
#
#         for neighbor in maze.neighbors(current[1][0][0], current[1][0][1]):
#             new_waypoints = []
#             new_waypoints += current[1][1]
#             if neighbor in new_waypoints:
#                 order_waypoints.append(neighbor)
#                 new_waypoints.remove(neighbor)
#
#             if len(new_waypoints) == 0:
#                 visited[neighbor] = current[1][0]
#                 break
#             if tuple(new_waypoints) not in mst_map:
#                 mst_map[tuple(new_waypoints)] = MST(tuple(new_waypoints)).compute_mst_weight()
#
#             heuristic_cost = find_closest_goal(neighbor,new_waypoints) + mst_map[tuple(new_waypoints)] + cost[current[1][0]] + 1
#             node_state = neighbor, new_waypoints
#
#             # node = neighbor, cost[current[1][0]] + 1 + heuristic_cost, cost[current[1][0]] + 1, current[0], rem_waypoints
#             if neighbor not in cost or heuristic_cost < cost[neighbor]:
#             # if neighbor not in cost or cost[current[1][0]] < cost[neighbor]:
#                 cost[neighbor] = cost[current[1][0]] + 1
#                 heapq.heappush(frontier, (heuristic_cost,node_state))
#                 visited[neighbor] = current[1][0]
#
#     print("Solution:", visited)
#     print("ordered",order_waypoints)
#
#     curr_cell = visited[order_waypoints[-1]]
#     print(curr_cell)
#
#     while curr_cell != order_waypoints[0]:
#         path.appendleft(curr_cell)
#         curr_cell = visited[curr_cell]
#
#     # path.append(start)
#
#     print(path)
#     return path




#
# def astar_multiple(maze):
#     """
#     Runs A star for part 3 of the assignment in the case where there are
#     multiple objectives.
#
#     @param maze: The maze to execute the search on.
#
#     @return path: a list of tuples containing the coordinates of each state in the computed path
#     """
#     rem_waypoints = list(maze.waypoints)
#     order_waypoints = []
#     solutiondict = {}
#     # print("waypoint",type(rem_waypoints))
#     def manhattan_cost(end, neighbor):
#         cost = abs(end[0] - neighbor[0]) + abs(end[1] - neighbor[1])
#         return cost
#
#     def to_cost(current, neighbor):
#         cost = abs(current[0] - neighbor[0]) + abs(current[1] - neighbor[1])
#         return cost
#
#
#     start = maze.start
#
#     # for end in maze.waypoints:
#     # print("ENDDDD",end)
#     frontier = PriorityQueue()
#     solution = {}
#     cost = {}
#     explored = {}
#     explored[start] = 0
#     solution[start] = None
#     cost[start] = 0
#     frontier.put(start, manhattan_cost(end, start))
#     while not frontier.empty():
#         current = frontier.get()
#         # print("Current",current)
#         # print("Frontier",frontier.queue)
#         if current == end:
#             break
#
#         for neighbor in maze.neighbors(current[0], current[1]):
#             new_cost = cost[current] + to_cost(current, neighbor) + MST(rem_waypoints).compute_mst_weight()
#
#             # print(maze.neighbors(current[0], current[1]))
#             if neighbor not in explored or new_cost < explored[neighbor]:
#                 cost[neighbor] = new_cost
#                 priority = new_cost + manhattan_cost(neighbor, end)
#                 frontier.put(neighbor, priority)
#                 solution[neighbor] = current
#                 explored[neighbor] = new_cost
#
#             # print("Frontier_pop",frontier.queue)
#
#             if neighbor in rem_waypoints:
#                 order_waypoints.append(neighbor)
#                 rem_waypoints.remove(neighbor)
#
#     solutiondict[end] = solution
#
#     lendict = {}
#
#     for key in solutiondict:
#         lendict[key] = len(solutiondict[key])
#     print(lendict)
#     print(min(lendict, key=lendict.get))
#     key = min(lendict, key=lendict.get)
#     solution = solutiondict[key]
#
#     current = key
#     path = []
#
#     while current != None:
#         path.append(current)
#         current = solution[current]
#
#     path.reverse()
#
#     return path


def fast(maze):
    """
    Runs suboptimal search algorithm for extra credit/part 4.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    frontier = []
    start = maze.start
    rem_waypoints = list(maze.waypoints)

    visited = {start: start}
    cost_dict = {start: 0}
    count = {start: 0}
    path = deque()

    cost_MST = MST.compute_mst_weight(MST(rem_waypoints))
    flag = 0
    finalpath = deque()

    _, length = find_closest_goal(start, rem_waypoints)
    cost = 0.0 + length + cost_MST
    heapq.heappush(frontier, (cost, start))

    while len(rem_waypoints) > 0:
        current = heapq.heappop(frontier)[1]

        if flag == 0:
            flag = 1

        if current in rem_waypoints:
            current2 = current
            while current2 != start:
                path.appendleft(current2)
                current2 = visited[current2]

            finalpath = finalpath + path

            path = deque()
            start = current
            rem_waypoints.remove(current)
            cost_MST = MST.compute_mst_weight(MST(rem_waypoints))

            flag = 0
            visited = {current: current}
            cost_dict[current] = 0
            frontier = []

        for neighbor in maze.neighbors(current[0], current[1]):

            if len(rem_waypoints) == 0:
                return finalpath
            _, length = find_closest_goal(neighbor, rem_waypoints)
            cost = length + cost_MST + count[current] + 1

            if neighbor not in visited or cost < cost_dict[neighbor]:
                heapq.heappush(frontier, (cost, neighbor))
                visited[neighbor] = current
                cost_dict[neighbor] = cost
                count[neighbor] = count[current] + 1

    return []


