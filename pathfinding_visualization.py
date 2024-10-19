import pygame
import math
import time  # For timing the algorithms
from queue import PriorityQueue

# Initialize Pygame
pygame.init()

# Window dimensions
WIDTH = 600
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Pathfinding Algorithm Visualization")

# Color definitions
RED = (255, 0, 0)          # Closed nodes
GREEN = (0, 255, 0)        # Open nodes
BLUE = (0, 0, 255)         # Unused
YELLOW = (255, 255, 0)     # Unused
WHITE = (255, 255, 255)    # Background
BLACK = (0, 0, 0)          # Obstacles
PURPLE = (128, 0, 128)     # Final path
ORANGE = (255, 165, 0)     # Start node
TURQUOISE = (64, 224, 208) # Goal node
GREY = (220, 220, 220)     # Grid lines (unused)

class Node:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width  # x-coordinate in pixels
        self.y = col * width  # y-coordinate in pixels
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows
        self.distance = float("inf")  # For Dijkstra's algorithm
        self.cost = float("inf")      # For custom algorithm

    def get_pos(self):
        return self.row, self.col

    def is_obstacle(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE

    def is_goal(self):
        return self.color == TURQUOISE

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = ORANGE

    def make_goal(self):
        self.color = TURQUOISE

    def make_obstacle(self):
        self.color = BLACK

    def draw(self, win):
        # Draw obstacles, start, and goal nodes as circles
        if self.is_obstacle():
            pygame.draw.circle(win, BLACK, (self.x + self.width // 2, self.y + self.width // 2), self.width // 2)
        elif self.is_start():
            pygame.draw.circle(win, ORANGE, (self.x + self.width // 2, self.y + self.width // 2), self.width // 2)
        elif self.is_goal():
            pygame.draw.circle(win, TURQUOISE, (self.x + self.width // 2, self.y + self.width // 2), self.width // 2)

    def update_neighbors(self, grid):
        # Update the list of accessible neighbors
        self.neighbors = []
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),  # Up, Down, Left, Right
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]  # Diagonals
        for dx, dy in directions:
            row = self.row + dx
            col = self.col + dy
            if 0 <= row < self.total_rows and 0 <= col < self.total_rows:
                neighbor = grid[row][col]
                if not neighbor.is_obstacle():
                    self.neighbors.append(neighbor)

    def __lt__(self, other):
        # Less than operator for PriorityQueue
        return False

def h(p1, p2):
    # Heuristic function (Euclidean distance)
    x1, y1 = p1
    x2, y2 = p2
    return math.hypot(x2 - x1, y2 - y1)

def reconstruct_path(came_from, current, win):
    # Reconstruct the path from start to goal
    total_cost = 0
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.append(current)
    path.reverse()

    # Draw the final path as a thick line
    for i in range(len(path) - 1):
        pygame.draw.line(win, PURPLE,
                         (path[i].x + path[i].width // 2, path[i].y + path[i].width // 2),
                         (path[i + 1].x + path[i + 1].width // 2, path[i + 1].y + path[i + 1].width // 2), 5)
        pygame.display.update()
        pygame.time.delay(20)  # Delay to visualize drawing

    # Calculate total cost
    for i in range(len(path) - 1):
        total_cost += h(path[i].get_pos(), path[i + 1].get_pos())

    return total_cost

def draw_search_path(win, came_from, current_node):
    # Draw lines to represent the search path
    parent = came_from.get(current_node)
    if parent:
        pygame.draw.line(win, RED,
                         (current_node.x + current_node.width // 2, current_node.y + current_node.width // 2),
                         (parent.x + parent.width // 2, parent.y + parent.width // 2), 1)
        pygame.display.update()

def dijkstra(draw, grid, start, goal):
    # Standard Dijkstra's algorithm
    start_time = time.time()  # Start timing
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    start.distance = 0
    came_from = {}

    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return False

        current_node = open_set.get()[2]
        open_set_hash.remove(current_node)

        if current_node == goal:
            total_cost = reconstruct_path(came_from, goal, WIN)
            end_time = time.time()  # End timing
            print("Dijkstra's Algorithm:")
            print(f"Total Cost: {total_cost}")
            print(f"Time Taken: {end_time - start_time:.6f} seconds")
            goal.make_goal()
            start.make_start()
            return True

        for neighbor in current_node.neighbors:
            temp_distance = current_node.distance + h(current_node.get_pos(), neighbor.get_pos())

            if temp_distance < neighbor.distance:
                came_from[neighbor] = current_node
                neighbor.distance = temp_distance
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((neighbor.distance, count, neighbor))
                    open_set_hash.add(neighbor)
                    # Draw the search path
                    draw_search_path(WIN, came_from, neighbor)

        # Draw the updated window
        draw()

    return False

def custom_algorithm(draw, grid, start, goal):
    # Custom algorithm with refined obstacle handling and heuristic
    start_time = time.time()  # Start timing
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    start.cost = 0
    came_from = {}

    open_set_hash = {start}
    distance_weight = 1.0  # Weight for distance cost
    obstacle_weight = 0.5   # Further reduced obstacle penalty

    # Memoization to store minimum costs for nodes
    min_cost = {start.get_pos(): 0}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return False

        current_node = open_set.get()[2]
        open_set_hash.remove(current_node)

        if current_node == goal:
            total_cost = reconstruct_path(came_from, goal, WIN)
            end_time = time.time()  # End timing
            print("Custom Algorithm (Further Optimized):")
            print(f"Total Cost: {total_cost}")
            print(f"Time Taken: {end_time - start_time:.6f} seconds")
            goal.make_goal()
            start.make_start()
            return True

        for neighbor in current_node.neighbors:
            # Basic distance cost
            distance = h(current_node.get_pos(), neighbor.get_pos()) * distance_weight

            # Obstacle proximity penalty (only immediate obstacles)
            obstacle_penalty = 0
            immediate_neighbors = [
                (neighbor.row + dx, neighbor.col + dy)
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
                if 0 <= neighbor.row + dx < neighbor.total_rows and 0 <= neighbor.col + dy < neighbor.total_rows
            ]

            for nx, ny in immediate_neighbors:
                if grid[nx][ny].is_obstacle():
                    obstacle_penalty += obstacle_weight

            # Total cost for the neighbor
            total_neighbor_cost = current_node.cost + distance  # Exclude obstacle penalty from cost

            # Heuristic includes obstacle penalty to influence node selection
            heuristic = h(neighbor.get_pos(), goal.get_pos()) + obstacle_penalty

            # Check memoized cost to avoid redundant exploration
            if neighbor.get_pos() not in min_cost or total_neighbor_cost < min_cost[neighbor.get_pos()]:
                came_from[neighbor] = current_node
                neighbor.cost = total_neighbor_cost
                min_cost[neighbor.get_pos()] = total_neighbor_cost  # Memoize the cost

                if neighbor not in open_set_hash:
                    count += 1
                    priority = total_neighbor_cost + heuristic  # Priority includes heuristic
                    open_set.put((priority, count, neighbor))
                    open_set_hash.add(neighbor)
                    # Draw the search path as it progresses
                    draw_search_path(WIN, came_from, neighbor)

        # Draw the updated window
        draw()

    return False



def make_grid(rows, width):
    # Create a grid of nodes
    grid = []
    gap = width // rows  # Size of each node
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            node = Node(i, j, gap, rows)
            grid[i].append(node)
    return grid

def draw(win, grid, rows, width):
    # Draw all elements on the window
    win.fill(WHITE)
    for row in grid:
        for node in row:
            node.draw(win)

    pygame.display.update()

def get_clicked_pos(pos, rows, width):
    # Get the position in the grid based on mouse position
    gap = width // rows
    y, x = pos
    row = y // gap
    col = x // gap
    return row, col

def main(win, width):
    # Main function to run the visualization
    ROWS = 50  # Number of rows in the grid
    grid = make_grid(ROWS, width)

    start = None
    goal = None

    run = True
    algorithm_started = False
    use_custom = False  # Toggle between Dijkstra and custom algorithm

    while run:
        draw(win, grid, ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if algorithm_started:
                continue

            if pygame.mouse.get_pressed()[0]:  # Left-click
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                node = grid[row][col]
                if not start and node != goal:
                    start = node
                    start.make_start()
                elif not goal and node != start:
                    goal = node
                    goal.make_goal()
                elif node != start and node != goal:
                    node.make_obstacle()
            elif pygame.mouse.get_pressed()[2]:  # Right-click
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                node = grid[row][col]
                node.reset()
                if node == start:
                    start = None
                elif node == goal:
                    goal = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and not algorithm_started and start and goal:
                    for row in grid:
                        for node in row:
                            node.update_neighbors(grid)
                    algorithm_started = True
                    if use_custom:
                        custom_algorithm(lambda: draw(win, grid, ROWS, width), grid, start, goal)
                    else:
                        dijkstra(lambda: draw(win, grid, ROWS, width), grid, start, goal)
                    algorithm_started = False
                if event.key == pygame.K_c:
                    start = None
                    goal = None
                    grid = make_grid(ROWS, width)
                if event.key == pygame.K_d:
                    use_custom = False
                    print("Switched to Dijkstra's algorithm.")
                if event.key == pygame.K_a:
                    use_custom = True
                    print("Switched to custom algorithm with obstacle cost.")

    pygame.quit()

if __name__ == "__main__":
    main(WIN, WIDTH)
