import pygame

from models import Pointer
from util import GREY, WHITE, dijkstra, astar, bfsearch


def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Pointer(i, j, gap, rows)
            grid[i].append(spot)

    return grid


def draw_grid(win, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))


def draw(win, grid, rows, width):
    win.fill(WHITE)
    for row in grid:
        for spot in row:
            spot.draw(win)
    draw_grid(win, rows, width)
    pygame.display.update()


def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y, x = pos
    row = y // gap
    col = x // gap
    return row, col


def program(win, width, algo="astar"):
    """Visualization program which updates based on obstacles and path finding algorithm outcome."""
    ROWS = 50
    grid = make_grid(ROWS, width)
    start = None
    end = None
    run = True
    while run:
        draw(win, grid, ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if pygame.mouse.get_pressed()[0]:  # LEFT
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                try:
                    spot = grid[row][col]
                    if not start and spot != end:
                        start = spot
                        start.make_start()
                    elif not end and spot != start:
                        end = spot
                        end.make_end()
                    elif spot != end and spot != start:
                        spot.make_barrier()
                except IndexError:
                    pass
            elif pygame.mouse.get_pressed()[2]:  # RIGHT
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                spot.reset()
                if spot == start:
                    start = None
                elif spot == end:
                    end = None
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)
                    if algo == "a*":
                        astar(lambda: draw(win, grid, ROWS, width), grid, start, end)
                    elif algo == "bfs":
                        bfsearch(lambda: draw(win, grid, ROWS, width), grid, start, end)
                    else:
                        dijkstra(lambda: draw(win, grid, ROWS, width), grid, start, end)
                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)
    pygame.quit()


if __name__ == "__main__":
    print("Path finding by Nicholas Stewart, Dennis Grigoryev, and Alisha Karim.")
    print("----------------------------------------------------------------------")
    print("Welcome to our pathfinding visualization project. ")
    print(
        "First, click two nodes to set the start and end points. Then draw the maze and press Enter to run the "
        "pathfinding algorithm."
    )
    selection = input("Please input a* , bfs or dijkstra: ")
    if selection == "a*":
        print(
            """
        A* (A-star) is a pathfinding algorithm that efficiently finds the shortest path between two points by 
        combining the actual distance from the start and an estimated distance to the goal. It uses a heuristic to
        guide its search, making it faster and more optimal than simpler algorithms like Dijkstra's in many cases.
        
        - What is it's input size: Number of nodes in a graph, can be referenced as barriers in a maze.
        - What is the basic operation: The basic operation is removing the node with the lowest f(n) = g(n) + h(n) from the priority queue.
        - Does the basic operation depend only on n: It depends on the structure of the graph (e.g. obstructions in maze) and the heuristic function used.
        - Summation to count number of operations: sum(n, i=1) log i.
        - Find the closed-form formula or order of growth.: Time complexity in worst case is O(n log n).
        """
        )
    elif selection == "bfs":
        print(
            """ 
        Breadth-First Search (BFS) is a graph traversal algorithm that explores all neighbors at the present depth before moving on to nodes at the next depth level.
        It is optimal for unweighted graphs, as it guarantees the shortest path in terms of the number of edges.
        - What is it's input size: Number of nodes in a graph, can be referenced as barriers in a maze.
        - What is the basic operation: The basic operation is adding nodes to the queue and removing them for exploration.
        - Does the basic operation depend only on n: It depends on the structure of the graph (e.g. obstructions in maze)? : Yes as the number of nodes and edges directly affect the traversal.
        - Summation to count number of operations: sum(n, i=1) + sum(m, j=1).
        - Find the closed-form formula or order of growth.: Time complexity is O(n + m) where n is the number of nodes and m is the number of edges.
        """
        )
    else:
        print(
            """
        Dijkstra's algorithm finds the shortest path from a starting node to all other nodes in a weighted graph 
        with non-negative edge weights. It uses a priority queue to always expand the node with the smallest known 
        distance, updating paths efficiently as it goes.

        - What is it's input size: Number of nodes as well as the number of edges.
        - What is the basic operation: Extracting the minimum-distance node from the priority queue.
        - Does the basic operation depend only on n: It depends on n and m, but these two values directly effect outcome unlike with a* where heuristic function matters.
        - Summation to count number of operations: sum(n, i = 1) log i + sum(m , j=1) log n
        - Find the closed-form formula or order of growth.: Time complexity is O((n + m) log n)
        """
        )
    input("Press enter to open the visualizer.")
    pygame.display.set_caption(f"{selection} Path Finding Visualizer")
    program(pygame.display.set_mode((700, 700)), 700, selection)
