import cv2
import numpy as np
from queue import PriorityQueue
import time
import random

canvas_height = 501
canvas_width = 1201
free_space_color = (255, 255, 255)
obstacle_color = (0, 0, 0)
canvas = np.ones((canvas_height, canvas_width, 3), dtype="uint8") * 255  

def draw_rectangle(center, width, height, color):
    top_left = (int(center[0] - width/2), int(center[1] - height/2))
    bottom_right = (int(center[0] + width/2), int(center[1] + height/2))
    cv2.rectangle(canvas, top_left, bottom_right, color, -1)

clearance = [
    ((137.5, 200), 85, 410),
    ((312.3, 300), 85, 410),
    ((1000, 87.5), 210, 85),
    ((1000, 412.5), 210, 85),
    ((1060, 250), 90, 240),
]

obstacle = [
    ((137.5, 200), 75, 400),
    ((312.3, 300), 75, 400),
    ((1000, 87.5), 200, 75),
    ((1000, 412.5), 200, 75),
    ((1060, 250),80, 250),
]

for center, width, height in clearance:
    draw_rectangle(center, width, height, (127, 127, 127)) 

for center, width, height in obstacle:
    draw_rectangle(center, width, height, (0, 0, 0))

center = (650, 250)
side_length = 150
radius_c = (side_length / (2 * np.sin(np.pi / 6))) + 5

hex_c = []
for i in range(6):
    x_c = int(center[0] + radius_c * np.cos(i * 2 * np.pi / 6))
    y_c = int(center[1] + radius_c * np.sin(i * 2 * np.pi / 6))
    hex_c.append((x_c, y_c))
    
radius = (side_length / (2 * np.sin(np.pi / 6)))
hex = []
for j in range(6):
    x = int(center[0] + radius * np.cos(j * 2 * np.pi / 6))
    y = int(center[1] + radius * np.sin(j * 2 * np.pi / 6))
    hex.append((x, y))

pts_c = np.array(hex_c, np.int32)
pts_c = pts_c.reshape((-1, 1, 2))
color_c = (127, 127, 127)
pts = np.array(hex, np.int32)
pts = pts.reshape((-1, 1, 2))
color = (0, 0, 0)

cv2.fillPoly(canvas, [pts_c], color_c)
cv2.fillPoly(canvas, [pts], color)

out = cv2.VideoWriter('A_star.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (canvas_width, canvas_height))

def random_node_gen():
    x = random.randint(0, canvas_width - 1)
    y = random.randint(0, canvas_height - 1)
    # y = abs(500 - y)
    while not is_free(x, abs(500 - y)):
        x = random.randint(0, canvas_width - 1)
        y = random.randint(0, canvas_height - 1)
        # y = abs(500 - y)
    return (x, y)

def is_free(x, y):
    return all(canvas[y, x] == free_space_color) or all(canvas[y, x] == (0, 255, 0)) or all(canvas[y, x] == (0, 0, 255))

def get_neighbors(node):
    x, y = node
    neighbors = []
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < canvas_width and 0 <= ny < canvas_height and is_free(nx, ny):
            if dx != 0 and dy != 0:
                cost = 1.4 
            else:
                cost = 1.0
            neighbors.append(((nx, ny), cost))
    return neighbors

D = 1.0  
D2 = 1.4 

def octile_distance(x1, y1, x2, y2):
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

def a_star(start, goal):
    pq = PriorityQueue()
    pq.put((0, start))
    came_from = {start: None}
    cost_to_goal = octile_distance(start[0], start[1], goal[0], goal[1])
    # cost_to_goal = ((goal[0] - start[0])**2 + (goal[1] - start[1])**2)**0.5
    # cost_to_goal = abs(goal[0] - start[0]) + abs(goal[1] - start[1])
    cost_so_far = {start: cost_to_goal}
    count =0

    while not pq.empty():
        current_cost, current_node = pq.get()
        if current_node == goal:
            print("Cost to Goal: " , cost_so_far[goal])
            print("Goal Reached")
            cv2.destroyAllWindows()
            break
        for next_node, cost in get_neighbors(current_node):
            cost_to_go = octile_distance(next_node[0], next_node[1], goal[0], goal[1])
            # cost_to_go = ((goal[0] - next_node[0])**2 + (goal[1] - next_node[1])**2)**0.5
            # cost_to_go = abs(goal[0] - next_node[0]) + abs(goal[1] - next_node[1])
            new_cost = current_cost + cost + cost_to_go
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost
                pq.put((priority, next_node))
                canvas[next_node[1], next_node[0]] = (255, 0, 0)
                came_from[next_node] = current_node
                count += 1
                if count%1200 == 0:
                    cv2.imshow('A*', canvas)
                    cv2.waitKey(1)
                    
                    out.write(canvas)
    return came_from, cost_so_far

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    count = 0
    while current != start:
        path.append(current)
        cv2.circle(canvas, current, 2, (255, 255, 255), -1)
        if count%30 == 0:
            cv2.imshow('A*', canvas)
            cv2.waitKey(1)
            
            out.write(canvas)
        count += 1
        current = came_from[current]
    cv2.destroyAllWindows()
    path.append(start)
    path.reverse()
    return path

def visualize_path(path):
    count = 0
    for node in path:
        x, y = node
        cv2.circle(canvas, (x, y), 2, (0, 0, 255), -1) 
        count += 1
        if count%15 == 0:
            cv2.imshow('A*', canvas)
            cv2.waitKey(1)
            out.write(canvas)
    cv2.destroyAllWindows()       
    for i in range(30):
        out.write(canvas)

    cv2.imshow('Path', canvas)
    out.release()

Xi = input("Enter the start node X: ")
Yi = input("Enter the start node Y: ")


Xg = input("Enter the goal node X: ")
Yg = input("Enter the goal node Y: ")

if not Xi.isdigit() or not Yi.isdigit() or not Xg.isdigit() or not Yg.isdigit():
    print("Picking a Random Start and Goal Node")
    start_node_er = random_node_gen()  
    goal_node_er = random_node_gen()
    
    print("Start Node: ", start_node_er)
    print("Goal Node: ", goal_node_er)
    start_node = (start_node_er[0], abs(500 - start_node_er[1]))
    goal_node = (goal_node_er[0], abs(500 - goal_node_er[1]))
else:
    print("Start Node: ", (int(Xi), int(Yi)))
    print("Goal Node: ", (int(Xg), int(Yg)))
    Yi = abs(500 - int(Yi))
    start_node = (int(Xi), int(Yi))

    Yg = abs(500 - int(Yg))
    goal_node = (int(Xg), int(Yg))

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
    cv2.circle(canvas, start_node, 5, (0, 0, 255), -1)
    cv2.circle(canvas, goal_node, 5, (0, 255, 0), -1)
    
    came_from, cost_so_far = a_star(start_node, goal_node)
    path = reconstruct_path(came_from, start_node, goal_node)
    visualize_path(path)

end_time = time.time()
execution_time = end_time - start_time

print("Execution time: %.4f seconds" % execution_time)
cv2.waitKey(0)
cv2.destroyAllWindows()