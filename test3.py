# Import necessary libraries
import cv2
import numpy as np
from queue import PriorityQueue
import time

# Canvas dimensions
canvas_height = 501
canvas_width = 1201

# Define the colors
clearance_color = (127, 127, 127)
obstacle_color = (0, 0, 0)
free_space_color = (255, 255, 255)

clearance_distance = 5

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
# print(canvas_array)

def is_free(x, y, theta):
    # Normalize theta to be in the range of 0-359 degrees
    theta_normalized = theta % 360
    # Convert theta into one of 12 discrete orientations (0-11)
    theta_index = theta_normalized // 30
    return canvas_array[x, y, theta_index] == 0

def get_neighbors(node):
    x, y , theta = node
    neighbours = []
    action_set = [theta, theta +30, theta -30, theta +60, theta -60]
    for action in action_set:
        x_new = x + 1*np.cos(np.deg2rad(action))
        y_new = y + 1*np.sin(np.deg2rad(action))
        x_new = int(round(x_new,0))
        y_new = int(round(y_new,0))
        theta_index = int(action)%360
        if is_free(x_new, y_new, theta_index):
            cost = 1
            neighbours.append(((x_new, y_new, action), cost))

    return neighbours

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
        if current_node[0] == goal:
            print("Cost to Goal: " , cost_so_far[goal])
            print("Goal Reached")
            cv2.destroyAllWindows()
            break
        for next_node, cost in get_neighbors(current_node[0]):
            # cost_to_go = octile_distance(next_node[0], next_node[1], goal[0], goal[1])
            cost_to_go = ((goal[0] - next_node[0])**2 + (goal[1] - next_node[1])**2)**0.5
            # cost_to_go = abs(goal[0] - next_node[0]) + abs(goal[1] - next_node[1])
            new_cost = current_node[1] + cost + cost_to_go
            nc = current_node[1] + cost
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = nc
                priority = new_cost
                pq.put((priority, (next_node, nc)))
                canvas[next_node[1], next_node[0]] = (255, 0, 0)
                came_from[next_node] = current_node[0]
                count += 1
                if count%120000 == 0:
                    cv2.imshow('A*', canvas)
                    cv2.waitKey(1)
                    
                    
    return came_from, cost_so_far

while True:
    print("\nThe start node and goal node should be within the canvas dimensions (6-1195, 6-495) and not inside an obstacle.\n")
    Xi = input("Enter the start node X: ")
    Yi = input("Enter the start node Y: ")
    Ti = input("Enter the start node Angle: ")
    Xi = int(Xi)
    Yi = int(Yi)
    Ti = int(Ti)

    if not (Xi < 0 or Xi >= canvas_width or Yi < 0 or Yi >= canvas_height):
        # Ti_i = int(Ti)%360
        # if Ti_i == 12:
        #     Ti_i = 0
        if is_free(Xi, Yi, Ti):
            break
        else:
            print("Start node is inside an obstacle")
    else:
        print("Start node is out of bounds.")

# Ask the user for the goal node
while True:
    Xg = input("Enter the goal node X: ")
    Yg = input("Enter the goal node Y: ")
    To = input("Enter the goal node Angle: ")
    Xg = int(Xg)
    Yg = int(Yg)
    To = int(To)
    if not (Xg < 0 or Xg >= canvas_width or Yg < 0 or Yg >= canvas_height):
        # To_i = int(To)%360
        # if To_i == 12:
        #     To_i = 0
        if is_free(Xg, Yg, To):
            break
        else:
            print("Goal node is inside an obstacle")
    else:
        print("Goal node is inside an obstacle or out of bounds.")

start_node = (int(Xi), int(Yi), int(Ti))
goal_node = (int(Xg), int(Yg), int(To))
came_from, cost_so_far = a_star(start_node, goal_node)


