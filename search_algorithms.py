import sys
import random
import heapq
import math
import time
from collections import deque

#we'll start by defining the eight possible moves mapped to their corresponding cost using a Python dictionary
move_set = {"L": 2,
            "UL": 1,
            "U": 2,
            "UR": 1,
            "R": 2,
            "DR": 1,
            "D": 2,
            "DL": 1}


#the following function reads input from file
with open("input.txt", "r") as f:
    algorithm = int(f.readline().strip())
    n = int(f.readline().strip())
    board = [f.readline().strip() for _ in range(n)]


#now, we'll find the start and goal positions on the board
for i in range(n):
    for j in range(n):
         if board[i][j] == "S":
             start_pos = (i, j)
         elif board[i][j] == "G":
             goal_pos = (i, j)


#the following function checks to see if moves are valid.
def move_validator(i, j):
    if i < 0 or i >= n or j < 0 or j >= n or board[i][j] == "X":
        return False
    return True

#now we'll create a function to get all possible moves from the robot
def get_possible_moves(pos):
    i, j = pos
    moves = []
    for move, cost in move_set.items():
        if move == "L":
            new_pos = (i, j-1)
        elif move == "UL":
            new_pos = (i-1, j-1)
            if j > 0 and i > 0 and (board[i][j-1] == "X" or board[i-1][j] == "X"):
                continue
        elif move == "U":
            new_pos = (i-1, j)
        elif move == "UR":
            new_pos = (i-1, j+1)
            if j < n-1 and i > 0 and (board[i][j+1] == "X" or board[i-1][j] == "X"):
                continue
        elif move == "R":
            new_pos = (i, j+1)
        elif move == "DR":
            new_pos = (i+1, j+1)
            if j < n-1 and i < n-1 and (board[i][j+1] == "X" or board[i+1][j] == "X"):
                continue
        elif move == "D":
            new_pos = (i+1, j)
        elif move == "DL":
            new_pos = (i+1, j-1)
            if j > 0 and i < n-1 and (board[i][j-1] == "X" or board[i+1][j] == "X"):
                continue
        else:
            continue
        if move_validator(*new_pos):
            moves.append((new_pos, cost))
    return moves

#the following functions involve the required search algorithms
def bfs():
    # We'll start by initializing the sets for visited and explored positions
    visited = set()
    explored = set()

    # We then initialize the queue with the starting position, empty path, and zero cost
    queue = deque([(start_pos, [], 0)])

    # While there are positions in the queue
    while queue:
        # Pop the position, path, and cost from the front of the queue
        pos, path, cost = queue.popleft()

        # If the current position is the goal position
        if pos == goal_pos:
            # Add the goal position to the explored set
            explored.add(pos)
            # Return the path, cost, and explored positions
            return path, cost, explored

        # If the current position has been visited, skip to the next iteration
        if pos in visited:
            continue

        # Mark the current position as visited and add it to the explored set
        visited.add(pos)
        explored.add(pos)

        # Iterate through the possible moves from the current position
        for move, move_cost in get_possible_moves(pos):
            # Add the new position, updated path, and updated cost to the queue
            queue.append((move, path + [move], cost + move_cost))

    # If no path is found, return None for path, cost, and the explored set
    return None, None, explored


def ucs():
    visited = set()
    explored = set()

    # Initialize the priority queue with the starting position, empty path, and zero cost
    queue = [(0, start_pos, [])]

    while queue:
        # Pop the position, path, and cost with the lowest total cost from the priority queue
        cost, pos, path = heapq.heappop(queue)

        # If the current position is the goal position, return the path, cost, and number of explored states
        if pos == goal_pos:
            return path, cost, len(explored) + 1

        # If the current position has not been visited yet
        if pos not in visited:
            visited.add(pos)
            explored.add(pos)

            # Iterate through the possible moves from the current position
            for new_pos, move_cost in get_possible_moves(pos):
                if new_pos not in visited:
                    new_path = path + [new_pos]
                    new_cost = cost + move_cost
                    heapq.heappush(queue, (new_cost, new_pos, new_path))

    return path, new_cost, explored


def iterative_deepening_search():
    depth = 0
    explored = set()

    # Incrementally increase the depth limit
    while True:
        result = depth_limited_search(start_pos, [], depth)
        if result[0] is not None:
            path, cost, explored = result
            return path, cost, len(explored)
        depth += 1

def depth_limited_search(pos, path, depth):
    # If the depth limit is reached
    if depth == 0:
        if pos == goal_pos:
            return path + [pos], 0, {pos}
        else:
            return None, None, set()

    if depth > 0:
        for new_pos, move_cost in get_possible_moves(pos):
            result = depth_limited_search(new_pos, path + [pos], depth - 1)
            if result[0] is not None:
                new_path, cost, new_explored = result
                return new_path, cost + move_cost, {pos} | new_explored

    return None, None, set()


def a_star():
    visited = set()
    explored = set()

    # Initialize the priority queue with the starting position, empty path, and zero cost
    queue = [(0, start_pos, [], 0)]

    while queue:
        # Pop the position, path, and cost with the lowest f-score (cost + heuristic) from the priority queue
        f, pos, path, cost = heapq.heappop(queue)

        if pos == goal_pos:
            return path, cost, len(explored) + 1

        if pos not in visited:
            visited.add(pos)
            explored.add(pos)

            for new_pos, move_cost in get_possible_moves(pos):
                if new_pos not in visited:
                    new_path = path + [new_pos]
                    new_cost = cost + move_cost
                    heuristic = manhattan_distance(new_pos, goal_pos)
                    heapq.heappush(queue, (new_cost + heuristic, new_pos, new_path, new_cost))

    return path, new_cost, explored


def manhattan_distance(pos1, pos2):
    # Calculate Manhattan distance between two positions
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

def euclidean_distance(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def chebyshev_distance(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))

#I've included two hill climbling algorithms, the one's that commented out  performs a basic greedy local search based on the heuristic function (Manhattan distance in my case).
# def hill_climbing():
#     # Find start and goal positions
#     start_pos, goal_pos = None, None
#     for i in range(n):
#         for j in range(n):
#             if board[i][j] == "S":
#                 start_pos = (i, j)
#             elif board[i][j] == "G":
#                 goal_pos = (i, j)

#     visited = set()
#     current_pos = start_pos
#     current_cost = 0
#     path = [start_pos]

#     while current_pos != goal_pos:
#         visited.add(current_pos)
#         possible_moves = [(move, move_cost) for move, move_cost in get_possible_moves(current_pos) if move not in visited]
#         if not possible_moves:
#             break  # No moves available, exit the loop

#         # Find the next position with the lowest heuristic value
#         next_pos, next_cost = min(possible_moves, key=lambda x: manhattan_distance(x[0], goal_pos) + x[1])
#         current_pos = next_pos
#         current_cost += next_cost
#         path.append(current_pos)

#     if current_pos == goal_pos:
#         return path, current_cost, visited
#     else:
#         return None, None, visited


def hill_climbing(restarts=10, random_choice_prob=0.3):
    def random_move(pos, cost):
        # Get valid moves and return a random move
        valid_moves = get_possible_moves(pos)
        if not valid_moves:
            return None, None
        return random.choice(valid_moves)

    # Perform hill climbing with random restarts
    for _ in range(restarts):
        # Initialize the starting position, cost, and path
        current_pos = start_pos
        current_cost = 0
        path = [current_pos]
        visited = set([current_pos])

        # Continue until the goal is reached
        while current_pos != goal_pos:
            # Get unvisited possible moves
            possible_moves = [move for move in get_possible_moves(current_pos) if move[0] not in visited]
            if not possible_moves:
                break

            # Choose the move with the minimum heuristic value
            next_pos, next_cost = min(possible_moves, key=lambda x: manhattan_distance(x[0], goal_pos) + x[1])

            # Randomly decide whether to make a random move
            if random.random() < random_choice_prob:
                random_move_pos, random_move_cost = random_move(current_pos, current_cost)
                if random_move_pos is not None:
                    next_pos, next_cost = random_move_pos, random_move_cost

            # Update current position, cost, path, and visited states
            current_pos = next_pos
            current_cost += next_cost
            path.append(current_pos)
            visited.add(current_pos)

        # If the goal is reached, return the path, cost, and visited states
        if current_pos == goal_pos:
            return path, current_cost, visited

    # If no solution is found after all restarts, return None for path and cost, and visited states
    return None, None, visited



#the next function helps determine the direction of movement
def get_move_direction(curr_pos, next_pos):
    i, j = curr_pos
    new_i, new_j = next_pos
    if new_i == i-1 and new_j == j-1:
        return "UL"
    elif new_i == i-1 and new_j == j:
        return "U"
    elif new_i == i - 1 and new_j == j + 1:
        return "UR"
    elif new_i == i and new_j == j + 1:
        return "R"
    elif new_i == i + 1 and new_j == j + 1:
        return "DR"
    elif new_i == i + 1 and new_j == j:
        return "D"
    elif new_i == i + 1 and new_j == j - 1:
        return "DL"
    elif new_i == i and new_j == j - 1:
        return "L"


#this is a helper function that replaces path with "o" and the goal with "G" to help visualize
def replace_path_on_board(board, path):
    board_with_path = [list(row) for row in board]
    for i, pos in enumerate(path):
        if i == len(path) - 1:
            board_with_path[pos[0]][pos[1]] = "G"
        else:
            board_with_path[pos[0]][pos[1]] = "o"
    return board_with_path


#the output from the search algorithm are written in to an output file by the function below
def write_results(algorithm_name, path, cost, explored, board, start_time):
    end_time = time.time()
    execution_time = round((end_time - start_time) * 1000, 3) # convert to ms and round to 3 decimal places
    with open("output.txt", "w") as f:
        f.write("Algorithm: {}\n".format(algorithm_name))
        if path:
            path_str = "-".join([get_move_direction(path[i], path[i + 1]) for i in range(len(path) - 1)])
            f.write("Here's the solution I found: {}\n".format(path_str))
            f.write("The total path cost: {}\n".format(cost))
            f.write("The path length: {}\n".format(len(path)))
            if isinstance(explored, set):
                f.write("The total number of explored states: {}\n".format(len(explored)))
            else:
                f.write("The total number of explored states: {}\n".format(explored))
            board_with_path = replace_path_on_board(board, path)
            f.write("Visualization of path:\n")
            for row in board_with_path:
                f.write("".join(row) + "\n")
        else:
            f.write("Sorry, no path found.\n")
        f.write("Execution time: {} ms\n".format(execution_time))


algorithm_mapping = {
    1: ("Breadth First Search", bfs),
    2: ("Uniform Cost Search", ucs),
    3: ("Iterative Deepening", iterative_deepening_search),
    4: ("A*", a_star),
    5: ("Hill Climbing", hill_climbing)
}


start_time = time.time()
execution_time = time.time() - start_time
execution_time_str = "{:.3f}".format(execution_time)

# run the algorithm and get the results
algorithm_name, algorithm_func = algorithm_mapping[algorithm]
path, cost, explored = algorithm_func()
# write the results and include the execution time
write_results(algorithm_name, path, cost, explored, board, start_time)i