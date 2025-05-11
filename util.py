from queue import PriorityQueue

import pygame

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)


def h(p1, p2):
    """Heuristic function for A* algorithm."""
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)


def reconstruct_path(came_from, current, draw):
    """Reconstructs the path by backtracking from the goal node to the start node."""
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()


def astar(draw, grid, start, end):
    """
    Implements the A* (A-star) pathfinding algorithm.

    Parameters:
    - draw: A callback function used to update the visualization at each step.
    - grid: 2D list representing the grid of nodes.
    - start: The starting node.
    - end: The goal node.

    Returns:
    - True if a path is found, False otherwise.
    """

    count = 0  # Counter to break ties in the priority queue
    open_set = PriorityQueue()
    open_set.put((0, count, start))  # Priority queue stores (f_score, count, node)

    came_from = {}  # Dictionary to reconstruct the path
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0  # Distance from start to start is 0

    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = h(
        start.get_pos(), end.get_pos()
    )  # Estimated cost from start to end

    open_set_hash = {start}  # A set to quickly check if a node is in the open set

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2]  # Get the node with the lowest f_score
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1  # Assumes all edges have a weight of 1

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())

                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        draw()

        if current != start:
            current.make_closed()

    return False


def dijkstra(draw, grid, start, end):
    """
    Implements Dijkstra's pathfinding algorithm.

    Parameters:
    - draw: A callback function used to update the visualization at each step.
    - grid: 2D list representing the grid of nodes.
    - start: The starting node.
    - end: The goal node.

    Returns:
    - True if a path is found, False otherwise.
    """

    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))

    came_from = {}
    distance = {spot: float("inf") for row in grid for spot in row}
    distance[start] = 0  # Distance from start to itself is 0

    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True

        for neighbor in current.neighbors:
            temp_distance = distance[current] + 1  # Uniform edge weights

            if temp_distance < distance[neighbor]:
                came_from[neighbor] = current
                distance[neighbor] = temp_distance
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((distance[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        draw()

        if current != start:
            current.make_closed()

    return False


def bfsearch(draw, grid, start, end):
    """Implements the Breadth-First Search (BFS) algorithm.
    Parameters:
    - draw: A callback function used to update the visualization at each step.
    - grid: 2D list representing the grid of nodes.
    - start: The starting node.
    - end: The goal node.
    Returns:
    - True if a path is found, False otherwise.
    """
    queue = []
    queue.append(start)
    visited = set()
    visited.add(start)
    came_from = {}
    found = False
    while queue:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = queue.pop(0)

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True
            break

        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                queue.append(neighbor)
                came_from[neighbor] = current
                neighbor.make_open()

        draw()

        if current != start:
            current.make_closed()
    return False
