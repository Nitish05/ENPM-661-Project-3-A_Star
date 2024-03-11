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

# Define the video writer
out = cv2.VideoWriter('dijkstra_nitish_ravisankar_raveendran.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (canvas_width, canvas_height))

# Function to check if a node is free
def is_free(x, y):
    return all(canvas[y, x] == free_space_color) or all(canvas[y, x] == (0, 255, 0)) or all(canvas[y, x] == (0, 0, 255))

# Function to get the neighbors of a node
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
step_size = 1
theta = 30

def get_neighbors(node):
    x, y = node
    neighbors = []
    directions = [-2, -1, 0, 1, 2]
    for th in directions:
        rad = np.deg2rad(th * theta)
        nx = x + step_size * np.sin(rad)
        ny = y + step_size * np.cos(rad)
        # ny = abs(500 - ny)
        if 0 <= nx < canvas_width and 0 <= ny < canvas_height and is_free(int(nx), int(ny)):
            cost = 1
            neighbors.append(((int(nx), int(ny)), cost))
    return neighbors

# Function to implement Dijkstra's algorithm
def dijkstra(start, goal):
    pq = PriorityQueue() # Initialize the priority queue
    pq.put((0, start))   # Put the start node in the priority queueW
    came_from = {start: None}  
    cost_so_far = {start: 0}
    count =0

    while not pq.empty():  # Loop until the priority queue is empty
        current_cost, current_node = pq.get() 
        if current_node == goal:
            cv2.circle(canvas, start_node, 1, (0, 0, 255), -1)  # Draw the start node
            cv2.circle(canvas, goal_node, 1, (0, 255, 0), -1)   # Draw the goal node
            print("Cost to Goal: %.4f" %cost_so_far[goal])
            print("Goal Reached")
            break
        for next_node, cost in get_neighbors(current_node):  # Loops through the neighbors of the current node
            new_cost = current_cost + cost  # Calculate the new cost
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost
                pq.put((priority, next_node))
                canvas[next_node[1], next_node[0]] = (255, 0, 0)
                came_from[next_node] = current_node
                count += 1
                if count%3000 == 0:
                    out.write(canvas)
    return came_from, cost_so_far

# Function to reconstruct the path
def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    count = 0
    while current != start:
        path.append(current)
        cv2.circle(canvas, current, 1, (255, 255, 255), -1)
        if count%60 == 0:
            out.write(canvas)
        count += 1
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

# Function to visualize the path
def visualize_path(path):
    count = 0
    for node in path:
        x, y = node
        cv2.circle(canvas, (x, y), 1, (0, 0, 255), -1) 
        count += 1
        if count%30 == 0:
            out.write(canvas)
    for i in range(30):
        out.write(canvas)

    cv2.imshow('Path', canvas)
    # out.release()


# Ask the user for the start node
while True:
    print(''' 
          
        ▓█████▄  ██▓ ▄▄▄██▀▀▀██ ▄█▀  ██████ ▄▄▄█████▓ ██▀███   ▄▄▄      
        ▒██▀ ██▌▓██▒   ▒██   ██▄█▒ ▒██    ▒ ▓  ██▒ ▓▒▓██ ▒ ██▒▒████▄    
        ░██   █▌▒██▒   ░██  ▓███▄░ ░ ▓██▄   ▒ ▓██░ ▒░▓██ ░▄█ ▒▒██  ▀█▄  
        ░▓█▄   ▌░██░▓██▄██▓ ▓██ █▄   ▒   ██▒░ ▓██▓ ░ ▒██▀▀█▄  ░██▄▄▄▄██ 
        ░▒████▓ ░██░ ▓███▒  ▒██▒ █▄▒██████▒▒  ▒██▒ ░ ░██▓ ▒██▒ ▓█   ▓██▒
        ▒▒▓  ▒ ░▓   ▒▓▒▒░  ▒ ▒▒ ▓▒▒ ▒▓▒ ▒ ░  ▒ ░░   ░ ▒▓ ░▒▓░ ▒▒   ▓▒█░
        ░ ▒  ▒  ▒ ░ ▒ ░▒░  ░ ░▒ ▒░░ ░▒  ░ ░    ░      ░▒ ░ ▒░  ▒   ▒▒ ░
        ░ ░  ░  ▒ ░ ░ ░ ░  ░ ░░ ░ ░  ░  ░    ░        ░░   ░   ░   ▒   
        ░     ░   ░   ░  ░  ░         ░              ░           ░  ░
 ░                                                              
          ''')
    print("\nThe start node and goal node should be within the canvas dimensions (6-1195, 6-495) and not inside an obstacle.\n")
    Xi = input("Enter the start node X: ")
    Yi = input("Enter the start node Y: ")
    Xi = int(Xi)
    Yi = int(Yi)
    if not (Xi < 0 or Xi >= canvas_width or Yi < 0 or Yi >= canvas_height):
        if is_free(Xi, Yi):
            break
        else:
            print("Start node is inside an obstacle")
    else:
        print("Start node is out of bounds.")

# Ask the user for the goal node
while True:
    Xg = input("Enter the goal node X: ")
    Yg = input("Enter the goal node Y: ")
    Xg = int(Xg)
    Yg = int(Yg)
    if not (Xg < 0 or Xg >= canvas_width or Yg < 0 or Yg >= canvas_height):
        if is_free(Xg, Yg):
            break
        else:
            print("Goal node is inside an obstacle")
    else:
        print("Goal node is inside an obstacle or out of bounds.")

# Print the start and goal nodes
print("Start Node: ", (int(Xi), int(Yi)))
print("Goal Node: ", (int(Xg), int(Yg)))
Yi = abs(500 - int(Yi))
start_node = (int(Xi), int(Yi))

Yg = abs(500 - int(Yg))
goal_node = (int(Xg), int(Yg))

start_time = time.time()    # Start the timer

canvas[Yi, Xi] = (0, 0, 255)  # Draw the start node
canvas[Yg, Xg] = (0, 255, 0)  # Draw the goal node

for j in  range(25):
    out.write(canvas)
    
# Run Dijkstra's algorithm
came_from, cost_so_far = dijkstra(start_node, goal_node)
# path = reconstruct_path(came_from, start_node, goal_node)
# visualize_path(path)

end_time = time.time()  # End the timer
execution_time = end_time - start_time

out.release()
print("Execution time: %.4f seconds" % execution_time)
cv2.waitKey(0)
cv2.destroyAllWindows()