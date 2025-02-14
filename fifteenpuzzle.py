from collections import deque
import random
import copy
import time

class Node:
    def __init__(self, state, totalCost, position):
        self.state = state
        self.totalCost = totalCost
        self.position = position
        self.parent = None

def swap(state, pos, n, direct):
    x, y = pos % n, pos // n
    new_state = copy.deepcopy(state)
    
    if direct == 1 and y > 0:  # UP
        new_state[pos], new_state[pos - n] = new_state[pos - n], new_state[pos]
        return new_state, pos - n
    if direct == 2 and y < n - 1:  # DOWN
        new_state[pos], new_state[pos + n] = new_state[pos + n], new_state[pos]
        return new_state, pos + n
    if direct == 3 and x > 0:  # LEFT
        new_state[pos], new_state[pos - 1] = new_state[pos - 1], new_state[pos]
        return new_state, pos - 1
    if direct == 4 and x < n - 1:  # RIGHT
        new_state[pos], new_state[pos + 1] = new_state[pos + 1], new_state[pos]
        return new_state, pos + 1
    return None, None

# cound_misplaced_tiles: for counting the tiles that are not in the correct place
def misplaced_tiles(state, goal, n):
    return sum(1 for i in range(n * n) if state[i] != 0 and state[i] != goal[i])

# manhattan_distance_calc: calculate the sum of row and column
# h(n) = sum|x_current - x_final| + |y_current - y_final|
def manhattan(state, goal, n):
    total = 0
    for i in range(n * n):
        # ignore the blank tile
        if state[i] != 0:
            # find where it should be
            goal_pos = goal.index(state[i])
            total += abs((i // n) - (goal_pos // n)) + abs((i % n) - (goal_pos % n))
    return total

# improves manhattan_distance by adding 2 extra moves for conflicting tiles.
def linear_conflict(state, goal, n):
    """Improved heuristic: Manhattan Distance + Linear Conflict"""
    manhattan_dist = manhattan(state, goal, n)
    conflict_count = 0

    # Check rows for conflicts
    for row in range(n):
        tiles = [state[row * n + col] for col in range(n) if state[row * n + col] != 0]
        goal_positions = [goal.index(tile) for tile in tiles]
        for i in range(len(tiles)):
            for j in range(i + 1, len(tiles)):
                if goal_positions[i] > goal_positions[j] and goal_positions[i] // n == goal_positions[j] // n:
                    conflict_count += 1

    # Check columns for conflicts
    for col in range(n):
        tiles = [state[row * n + col] for row in range(n) if state[row * n + col] != 0]
        goal_positions = [goal.index(tile) for tile in tiles]
        for i in range(len(tiles)):
            for j in range(i + 1, len(tiles)):
                if goal_positions[i] > goal_positions[j] and goal_positions[i] % n == goal_positions[j] % n:
                    conflict_count += 1

    return manhattan_dist + 2 * conflict_count

def Astar(start, goal, heuristic, n):
    bound = heuristic(start.state, goal, n)
    stack = [start]
    while True:
        result = search(stack, bound, goal, heuristic, n)
        if isinstance(result, int):
            bound = result
        else:
            return result

def search(stack, bound, goal, heuristic, n):
    global nodes_expanded
    node = stack[-1]
    nodes_expanded += 1

    cost = node.totalCost + heuristic(node.state, goal, n)
    if cost > bound:
        return cost
    if node.state == goal:
        return node.totalCost

    min_cost = float('inf')
    for direction in range(1, 5):
        new_state, new_pos = swap(node.state, node.position, n, direction)
        if new_state and not any(n.state == new_state for n in stack):
            new_node = Node(new_state, node.totalCost + 1, new_pos)
            stack.append(new_node)
            result = search(stack, bound, goal, heuristic, n)
            if isinstance(result, int):
                min_cost = min(min_cost, result)
            else:
                return result
            stack.pop()
    return min_cost

def generate_solvable_puzzle(n):
    while True:
        state = random.sample(range(n * n), n * n)
        pos = state.index(0)
        if is_solvable(state, n):
            return state, pos

def is_solvable(state, n):
    inversions = sum(
        1 for i in range(n * n) for j in range(i + 1, n * n)
        if state[i] and state[j] and state[i] > state[j]
    )
    blank_row = (state.index(0) // n)
    return (n % 2 == 1 and inversions % 2 == 0) or (n % 2 == 0 and (blank_row % 2 == 0) == (inversions % 2 == 1))

def run_experiment(num_trials=5):
    heuristics = {
        "Misplaced Tiles": misplaced_tiles,
        "Manhattan Distance": manhattan,
        "Linear Conflict": linear_conflict
    }

    results = {h: {"steps": [], "nodes": []} for h in heuristics}

    for i in range(num_trials):
        initial_state, position = generate_solvable_puzzle(4)
        print(f"\nTrial {i+1}: now solving...")
        
        for h_name, h_func in heuristics.items():
            global nodes_expanded
            nodes_expanded = 0
            start_node = Node(initial_state, 0, position)
            print(f"-> Using heuristic: {h_name}")

            start_time = time.time()
            steps = Astar(start_node, goal_state, h_func, 4)
            end_time = time.time()

            print(f"Solved in {steps} steps, {nodes_expanded} nodes expanded. Time: {end_time - start_time:.2f}s")

            results[h_name]["steps"].append(steps)
            results[h_name]["nodes"].append(nodes_expanded)

    print_results(results)

def print_results(results):
    print("\nPuzzle Type  Heuristic              Avg Steps to Solution  Avg Nodes Expanded")
    print("--------------------------------------------------------------------------------")
    for h_name, data in results.items():
        avg_steps = sum(data["steps"]) / len(data["steps"])
        avg_nodes = sum(data["nodes"]) / len(data["nodes"])
        print(f"15-puzzle  {h_name: <23} {avg_steps: <24.2f} {avg_nodes: <20.2f}")

SIZE = 4
goal_state = list(range(1, SIZE * SIZE)) + [0]

run_experiment(100)
