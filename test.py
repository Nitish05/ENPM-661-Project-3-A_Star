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

x =650
y = 250
y = abs(canvas_height - y)
w = 10
l = 10
theta = 30
theta = np.deg2rad(theta)
canvas[y,x] = (0,0,255)

direction = [ -2, -1, 0, 1, 2]

for i in  direction:
    X = x + l*np.sin(theta * i)
    Y = y + l*np.cos(theta * i)
    Y = abs(canvas_height - Y)
    cv2.line(canvas, (x, y), (int(X), int(Y)), (0, 0, 255), 2)

cv2.imshow("Canvas", canvas)
cv2.waitKey(0)