# I chose the Rapidly-exploring Random Tree (RRT) algorithm for this path planning Lab. And why i do that is because:

# 1. RRT is sort of efficient, especially when dealing with obstacle-filled environments like ours. That means, 
# It quickly explores possible paths without getting bogged down in unnecessary calculations.

# 2. Also, since this assignment focuses on finding a single path and the environment itself is kind of static (likely won't change), 
# RRT's lower upfront cost compared to other algorithms is ideal. The thing is, We don't need to spend time building a complex roadmap beforehand, 
# for RRT can efficiently search for the path as we go.

import numpy as np
import matplotlib.pyplot as plt

class Node:
    def __init__(self, point):
        self.point = np.array(point)
        self.parent = None

def generate_random_point(x_range, y_range):
    return [np.random.uniform(x_range[0], x_range[1]), np.random.uniform(y_range[0], y_range[1])]

def steer(start, end, max_step):
    dist = np.linalg.norm(np.array(start) - np.array(end))
    if dist > max_step:
        ratio = max_step / dist
        new_point = (1 - ratio) * np.array(start) + ratio * np.array(end)
        return new_point.tolist()
    else:
        return end

def is_collision_free(start, end, obstacles):
    for obstacle in obstacles:
        if obstacle[0] <= start[0] <= obstacle[2] and obstacle[1] <= start[1] <= obstacle[3]:
            return False
        if obstacle[0] <= end[0] <= obstacle[2] and obstacle[1] <= end[1] <= obstacle[3]:
            return False
        if start[0] != end[0]:
            m = (end[1] - start[1]) / (end[0] - start[0])
            c = start[1] - m * start[0]
            y_left = m * obstacle[0] + c
            y_right = m * obstacle[2] + c
            if (obstacle[1] <= y_left <= obstacle[3]) or (obstacle[1] <= y_right <= obstacle[3]):
                return False
        if start[1] != end[1]:
            m = (end[0] - start[0]) / (end[1] - start[1])
            c = start[0] - m * start[1]
            x_bottom = m * obstacle[1] + c
            x_top = m * obstacle[3] + c
            if (obstacle[0] <= x_bottom <= obstacle[2]) or (obstacle[0] <= x_top <= obstacle[2]):
                return False
    return True

def RRT(start, goal, obstacles, max_iterations, max_step):
    tree = [Node(start)]
    for _ in range(max_iterations):
        random_point = generate_random_point([0, 100], [0, 100])
        nearest_node = min(tree, key=lambda node: np.linalg.norm(node.point - np.array(random_point)))
        new_point = steer(nearest_node.point, random_point, max_step)
        if is_collision_free(nearest_node.point, new_point, obstacles):
            new_node = Node(new_point)
            new_node.parent = nearest_node
            tree.append(new_node)
            if np.linalg.norm(new_point - np.array(goal)) < max_step:
                path = [goal]
                current_node = new_node
                while current_node.parent is not None:
                    path.append(current_node.point)
                    current_node = current_node.parent
                path.append(start)
                return True, path[::-1]
    return False, None

def main():
    try:
       
        points_str = input("Enter the starting point and target point (start_x,start_y;goal_x,goal_y):\n")
        start_str, goal_str = points_str.split(';')
        start = [int(coord) for coord in start_str.split(',')]
        goal = [int(coord) for coord in goal_str.split(',')]
        
        obstacle_coords = []
        print("Enter an obstacle's coordinates (x1,y1;x2,y2) or -1 to finish:")
        while True:
            obstacle_str = input()
            if obstacle_str.strip() == '-1':
                break
            coords1, coords2 = obstacle_str.split(';')
            coords1 = [int(coord) for coord in coords1.split(',')]
            coords2 = [int(coord) for coord in coords2.split(',')]
            obstacle_coords.append(coords1 + coords2)
        
        print("Output:")
        
        success, path = RRT(start, goal, obstacle_coords, 1000, 10)

        if success:
            for point in path:
                print(f"{int(point[0])},{int(point[1])}")
        else:
            print("No feasible path found.")

        plt.figure()
        for obstacle in obstacle_coords:
            plt.plot([obstacle[0], obstacle[2]], [obstacle[1], obstacle[1]], 'k')
            plt.plot([obstacle[0], obstacle[2]], [obstacle[3], obstacle[3]], 'k')
            plt.plot([obstacle[0], obstacle[0]], [obstacle[1], obstacle[3]], 'k')
            plt.plot([obstacle[2], obstacle[2]], [obstacle[1], obstacle[3]], 'k')

        plt.plot(start[0], start[1], 'go', label='Start')
        plt.plot(goal[0], goal[1], 'ro', label='Goal')

        if success and path:
            path_x, path_y = zip(*path)
            plt.plot(path_x, path_y, 'b-', label='Path')
            plt.title("RRT Path Found")
        else:
            plt.title("RRT: No Path Found")

        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.grid(True)
        plt.show()
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
