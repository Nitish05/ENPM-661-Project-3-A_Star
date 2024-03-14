

# Import necessary libraries
import cv2
import numpy as np
from queue import PriorityQueue
import time
import random

# Canvas dimensions
canvas_height = 501
canvas_width = 1201

# Define the colors
clearance_color = (127, 127, 127)
obstacle_color = (0, 0, 0)
free_space_color = (255, 255, 255)
theta_list = [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330]
step_size = 3
clearance_distance = 5
threshold = 1.5
robo_radius = 5

# Initialize a white canvas
canvas = np.ones((canvas_height, canvas_width, 3), dtype="uint8") * 255

# Define obstacles using half plane model
def obstacles(node):
    x, y = node
    Hex_center = (650, 250)
    Xc, Yc = Hex_center
    y = abs(y - canvas_height)
    side_length = 150
    R = np.cos(np.pi / 6) * side_length
    obstacles = [
        (x >= 100 and x <= 175 and y >= 100 and y <= 500), 
        (x >= 275 and x <= 350 and y >= 0 and y <= 400),
        (x >= 900 and x <= 1100 and y >= 50 and y <= 125),
        (x >= 900 and x <= 1100 and y >= 375 and y <= 450),
        (x >= 1020 and x <= 1100 and y >= 50 and y <= 450),
        (x >= Xc - R and x <= Xc + R and y <= ((np.pi/6)*(x-(Xc-R)))+325 and y <= -((np.pi/6)*(x-(Xc+R)))+325 and y >= -((np.pi/6)*(x-(Xc-R)))+175 and y >= ((np.pi/6)*(x-(Xc+R)))+175),
        
    ]
    return any(obstacles)

# Define clearance zones
def clearance(x, y, clearance):
    clearance = clearance + robo_radius
    Hex_center = (650, 250)
    Xc, Yc = Hex_center
    y = abs(y - canvas_height)
    side_length = 150 
    R = (np.cos(np.pi / 6) * side_length)  + clearance
    clearance_zones = [
        (x >= 100 - clearance and x <= 175 + clearance and y >= 100 - clearance and y <= 500 + clearance),
        (x >= 275 - clearance and x <= 350 + clearance and y >= 0 - clearance and y <= 400 + clearance),
        (x >= 900 - clearance and x <= 1100 + clearance and y >= 50 - clearance and y <= 125 + clearance),
        (x >= 900 - clearance and x <= 1100 + clearance and y >= 375 - clearance and y <= 450 + clearance),
        (x >= 1020 - clearance and x <= 1100 + clearance and y >= 50 - clearance and y <= 450 + clearance),
        (x >= Xc - R and x <= Xc + R and y <= ((np.pi/6)*(x-(Xc-R)))+325 + clearance and y <= -((np.pi/6)*(x-(Xc+R)))+325 + clearance and y >= -((np.pi/6)*(x-(Xc-R)))+175 - clearance and y >= ((np.pi/6)*(x-(Xc+R)))+175 - clearance),
        (x <= clearance or x >= canvas_width - clearance or y <= clearance or y >= canvas_height - clearance), # Add clearance to the edges of the canvas
    ]
    return any(clearance_zones)

# Draw the obstacles and clearance zones on the canvas
for x in range(canvas_width):
    for y in range(canvas_height):
        if clearance(x, y, clearance_distance):
            canvas[y, x] = clearance_color
        if obstacles((x, y)):
            canvas[y, x] = obstacle_color

canvas_array = np.zeros((canvas_width, canvas_height, 12))
for x in range(canvas_width):
    for y in range(canvas_height):
        if all(canvas[y, x] != free_space_color):
            canvas_array[x, y] = np.inf

out = cv2.VideoWriter('A_star.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (canvas_width, canvas_height))

def random_node_gen():
    x = random.randint(0, canvas_width - 1)
    y = random.randint(0, canvas_height - 1)
    T = random.choice(theta_list)
    # y = abs(500 - y)
    while not is_free(x, abs(500 - y), T):
        x = random.randint(0, canvas_width - 1)
        y = random.randint(0, canvas_height - 1)
        T = random.choice(theta_list)
        # y = abs(500 - y)
    return (x, y, T)



# def is_free(x, y):
#     return all(canvas[y, x] == free_space_color) or all(canvas[y, x] == (0, 255, 0)) or all(canvas[y, x] == (0, 0, 255))

def is_free(x, y, theta):
    # Normalize theta to be in the range of 0-359 degrees
    theta_normalized = theta % 360
    # Convert theta into one of 12 discrete orientations (0-11)
    theta_index = theta_normalized // 30
    if x >= 0 and x < canvas_width and y >= 0 and y < canvas_height:
        if canvas_array[x, y, theta_index] == 0:
            return True
    else:
        return False
# def get_neighbors(node):
#     x, y = node
#     neighbors = []
#     directions = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]
#     for dx, dy in directions:
#         nx, ny = x + dx, y + dy
#         if 0 <= nx < canvas_width and 0 <= ny < canvas_height and is_free(nx, ny):
#             if dx != 0 and dy != 0:
#                 cost = 1.4 
#             else:
#                 cost = 1.0
#             neighbors.append(((nx, ny), cost))
#     return neighbors


# def get_neighbors(node):
#     x, y = node
#     neighbors = []
#     directions = [theta+60, theta+30, theta+0 , theta-30, theta-60]
#     for th in directions:
#         rad = np.deg2rad(th)
#         nx = x + (step_size * np.cos(rad))
#         ny = y + (step_size * np.sin(rad))
#         nx = int(np.round(nx, 0))
#         ny = int(np.round(ny, 0))
#         # ny = abs(500 - ny)
#         if 0 <= nx < canvas_width and 0 <= ny < canvas_height and is_free(nx,ny):
#             cost = 1
#             neighbors.append(((nx, ny), cost))
#     return neighbors

def get_neighbors(node):
    x, y , theta = node
    neighbours = []
    action_set = [theta, theta +30, theta -30, theta +60, theta -60]
    for action in action_set:
        x_new = x + step_size*np.sin(np.deg2rad(action))
        y_new = y + step_size*np.cos(np.deg2rad(action))
        x_new = int(round(x_new,0))
        y_new = int(round(y_new,0))
        # theta_index = int(action)%360
        if is_free(x_new, y_new, action):
            cost = step_size
            neighbours.append(((x_new, y_new, action), cost))

    return neighbours

D = 1.0  
D2 = 1.4 

def octile_distance(x1, y1, x2, y2):
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

def check_goal_reached(current_node, goal):
    distance = ((current_node[0] - goal[0]) ** 2 + (current_node[1] - goal[1]) ** 2) ** 0.5
    return distance < threshold and current_node[2] == goal[2]


def a_star(start, goal):
    pq = PriorityQueue()
    # cost_to_goal = octile_distance(start[0], start[1], goal[0], goal[1])
    cost_to_goal = ((goal[0] - start[0])**2 + (goal[1] - start[1])**2)**0.5
    pq.put((cost_to_goal, (start, 0)))
    came_from = {start: None}
    # cost_to_goal = abs(goal[0] - start[0]) + abs(goal[1] - start[1])
    cost_so_far = {start: cost_to_goal}
    count =0

    while not pq.empty():
        current_cost, current_node = pq.get()
        if check_goal_reached(current_node[0], goal):
            print("Cost to Goal: " , cost_so_far[current_node[0]])
            goal = current_node[0]
            print("Goal Reached")
            cv2.destroyAllWindows()
            break
        for next_node, cost in get_neighbors(current_node[0]):
            # cost_to_go = octile_distance(next_node[0], next_node[1], goal[0], goal[1])
            cost_to_go = ((goal[0] - next_node[0])**2 + (goal[1] - next_node[1])**2)**0.5
            # cost_to_go = abs(goal[0] - next_node[0]) + abs(goal[1] - next_node[1])
            theta_normalized = next_node[2] % 360
            theta_index = theta_normalized // 30
            new_cost = current_node[1] + cost + cost_to_go
            nc = current_node[1] + cost
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = nc
                priority = new_cost
                pq.put((priority, (next_node, nc)))
                # canvas[next_node[1], next_node[0]] = (255, 0, 0)
                cv2.arrowedLine(canvas, (current_node[0][0], current_node[0][1]), (next_node[0], next_node[1]), (255, 0, 0), 1)
                canvas_array[next_node[0], next_node[1], int(theta_index)] = np.inf
                came_from[next_node] = current_node[0]
                count += 1
                if count%1200 == 0:
                    # cv2.imshow('A*', canvas)
                    # cv2.waitKey(1)                    
                    out.write(canvas)
    return came_from, cost_so_far, goal

# def reconstruct_path(came_from, start, goal):
#     current = (goal[0], goal[1])
#     start = (start[0], start[1])
#     path = []
#     count = 0
#     while current != start:
#         path.append(current)
#         # cv2.circle(canvas, (current[0], current[1]), 2, (255, 255, 255), -1)
#         # cv2.arrowedLine(canvas, (came_from[current][0], came_from[current][1]), (current[0], current[1]), (0, 0, 255), 1)
#         if count%30 == 0:
#             # cv2.imshow('A*', canvas)
#             # cv2.waitKey(1)
            
#             out.write(canvas)
#         count += 1
#         current = came_from[current]
#     cv2.destroyAllWindows()
#     path.append(start)
#     path.reverse()
#     return path

def reconstruct_path(came_from, start, goal):
    # Start with the goal node and work backwards to the start
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]  # Move to the previous node in the path
        path.append(current)

    path.reverse()  # Reverse the path to go from start to goal
    return path


def visualize_path(path):
    count = 0
    for node in path:
        x, y, t = node
        cv2.circle(canvas, (x, y), 2, (0, 0, 255), -1) 
        count += 1
        if count%15 == 0:
            # cv2.imshow('A*', canvas)
            # cv2.waitKey(1)
            out.write(canvas)
    cv2.destroyAllWindows()       
    for i in range(30):
        out.write(canvas)

    cv2.imshow('Path', canvas)
    out.release()

Xi = input("Enter the start node X: ")
Yi = input("Enter the start node Y: ")
Ti = input("Enter the start node Angle: ")



Xg = input("Enter the goal node X: ")
Yg = input("Enter the goal node Y: ")
To = input("Enter the goal node Angle: ")

if not Xi.isdigit() or not Yi.isdigit() or not Xg.isdigit() or not Yg.isdigit() or not Ti.isdigit() or not To.isdigit():
    print("Picking a Random Start and Goal Node")
    start_node_er = random_node_gen()  
    goal_node_er = random_node_gen()
    
    print("Start Node: ", start_node_er)
    print("Goal Node: ", goal_node_er)
    start_node = (start_node_er[0], abs(500 - start_node_er[1]), start_node_er[2])
    goal_node = (goal_node_er[0], abs(500 - goal_node_er[1]), goal_node_er[2])
else:
    print("Start Node: ", (int(Xi), int(Yi), int(Ti)))
    print("Goal Node: ", (int(Xg), int(Yg), int(To)))
    Yi = abs(500 - int(Yi))
    start_node = (int(Xi), int(Yi), int(Ti))

    Yg = abs(500 - int(Yg))
    goal_node = (int(Xg), int(Yg), int(To))

start_time = time.time()

for j in  range(25):
    out.write(canvas)



if start_node[0] < 0 or start_node[0] >= canvas_width or start_node[1] < 0 or start_node[1] >= canvas_height:
    print("Start node is out of bounds.")
elif goal_node[0] < 0 or goal_node[0] >= canvas_width or goal_node[1] < 0 or goal_node[1] >= canvas_height:
    print("Goal node is out of bounds.")
elif not is_free(*start_node):
    print("Start node is inside an obstacle.")
elif not is_free(*goal_node):
    print("Goal node is inside an obstacle.")
else:
    # cv2.circle(canvas, start_node, 5, (0, 0, 255), -1)
    # cv2.circle(canvas, goal_node, 5, (0, 255, 0), -1)
    
    came_from, cost_so_far, goal = a_star(start_node, goal_node)
    path = reconstruct_path(came_from, start_node, goal)
    visualize_path(path)

end_time = time.time()
execution_time = end_time - start_time

print("Execution time: %.4f seconds" % execution_time)
cv2.waitKey(0)
out.release()
cv2.destroyAllWindows()