import heapq
import random
import math

class state:
    def __init__(self, board, parent=None, move=None, depth=0, cost=0, heuristic=0):
        self.board = board
        self.parent = parent
        self.move = move
        self.depth = depth  # g(n) number of moves taken
        self.cost = cost  # f(n) = g(n) + h(n)
        self.heuristic = heuristic  # h(n)

    def __lt__(self, other):
        return self.cost < other.cost  # for priority queue

# cound_misplaced_tiles: for counting the tiles that are not in the correct place
def count_misplaced_tiles(board, final, size):
    "Number of misplaced tiles: "
    return sum(1 for i in range(len(board)) if board[i] != 0 and board[i] != final[i])

# manhattan_distance_calc: calculate the sum of row and column
# h(n) = sum|x_current - x_final| + |y_current - y_final|
def manhattan_distance_calc(board, final, size):
    "Manhattan distance heuristic"
    distance = 0
    for i in range(len(board)):
        if board[i] == 0:
            continue
        final_pos = final.index(board[i])
        distance += abs((i // size) - (final_pos // size)) + abs((i % size) - (final_pos % size))
    return distance

# euclidean_distance_calc: calculate the straight-line distance
# h(n) = sum(sqrt((x_current - x_final)^2 + (y_current - y_final)^2))
def euclidean_distance_calc(board, final, size):
    "Euclidean distance heuristic"
    distance = 0
    for i in range(len(board)):
        if board[i] == 0:
            continue
        final_pos = final.index(board[i])
        dx = (i // size - final_pos // size)
        dy = (i % size - final_pos % size)
        distance += math.sqrt(dx * dx + dy * dy)
    return distance

# valid_move: find all the valid moves
def valid_move(state, size):
    "All valid moves from the current state"
    board = state.board
    zero_index = board.index(0)
    row, col = zero_index // size, zero_index % size
    moves = []
    directions = {'Up': (-1, 0), 'Down': (1, 0), 'Left': (0, -1), 'Right': (0, 1)}

    for move, (dr, dc) in directions.items():
        new_row, new_col = row + dr, col + dc
        if (0 <= new_row < size and 0 <= new_col < size):
            new_index = new_row * size + new_col
            new_board = board[:]
            new_board[zero_index], new_board[new_index] = new_board[new_index], new_board[zero_index]
            moves.append((new_board, move))
    return moves

# optimal_solver: A* algorithm
def optimal_solver(start, final, size, heuristic_function, max_time=60):
    priority_q = []
    initial_heuristic = heuristic_function(start, final, size)  # this is for calculating h(n)
    start_state = state(start, None, None, 0, initial_heuristic, initial_heuristic)
    heapq.heappush(priority_q, start_state)

    visited = set()
    nodes_expanded = 0

    while priority_q:
        current = heapq.heappop(priority_q)
        nodes_expanded += 1

        # find the best state
        if current.board == final:
            path = []
            while current.parent:
                path.append(current.move)
                current = current.parent
            return len(path), nodes_expanded

        visited.add(tuple(current.board))

        for new_board, move in valid_move(current, size):
            if tuple(new_board) in visited:
                continue
            new_heuristic = heuristic_function(new_board, final, size)
            new_state = state(new_board, current, move, current.depth + 1, current.depth + 1 + new_heuristic, new_heuristic)
            heapq.heappush(priority_q, new_state)

    return -1, nodes_expanded

def generate_random_puzzle(size):
    "Generates a random solvable puzzle."
    board = list(range(size * size))
    random.shuffle(board)
    return board if is_solvable(board, size) else generate_random_puzzle(size)

def is_solvable(board, size):
    """Checks if a puzzle is solvable."""
    inv_count = sum(1 for i in range(len(board)) for j in range(i + 1, len(board)) if board[i] > board[j] and board[j] != 0)
    if size % 2 == 1:
        return inv_count % 2 == 0
    else:
        blank_row = board.index(0) // size
        return (inv_count + blank_row) % 2 == 1

def run_experiment(size):
    "Runs A* on a limited number of random puzzles and calculates average steps/nodes expanded."
    heuristics = {
        "h1 (Misplaced Tiles)": count_misplaced_tiles,
        "h2 (Manhattan Distance)": manhattan_distance_calc,
        "h3 (Euclidean Distance)": euclidean_distance_calc,
    }

    final = list(range(1, size * size)) + [0]
    results = {h: {"steps": 0, "nodes": 0} for h in heuristics}

    for _ in range(10):  # Limiting to 10 puzzles to speed up the experiment
        start = generate_random_puzzle(size)

        for h_name, heuristic in heuristics.items():
            steps, nodes = optimal_solver(start, final, size, heuristic)
            if steps != -1:  # Only count valid solutions
                results[h_name]["steps"] += steps
                results[h_name]["nodes"] += nodes

    print(f"\n8-puzzle...\n")
    print(f"{'Puzzle Type':<12}{'Heuristic':<25}{'Average Steps to Solution':<30}{'Average Nodes Expanded':<20}")
    print("-" * 84)

    for h_name in heuristics:
        avg_steps = results[h_name]["steps"] / 10 if results[h_name]["steps"] > 0 else 0
        avg_nodes = results[h_name]["nodes"] / 10 if results[h_name]["nodes"] > 0 else 0
        print(f"{size * size - 1}-puzzle   {h_name:<25} {avg_steps:<30.2f} {avg_nodes:<20.2f}")

if __name__ == "__main__":
    run_experiment(3)  # 8puzzle

