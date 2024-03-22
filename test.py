# This script was used to test the arrowedLine function in OpenCV

import cv2
import numpy as np

# Create a blank image, +1 in dimensions for boundary conditions
image = np.zeros((500, 500, 3), dtype="uint8")

# Define the starting and ending points of the vector
start_point = (100, 100)  # (x1, y1)
# end_point = (400, 400)    # (x2, y2)


color = (255, 0, 0)  # Blue color in BGR
thickness = 2

x = 100
y = 100
while True:
    angle = input("Enter the angle of the vector: ")
    angle = int(angle)
    angle = np.radians(angle)

    # Draw the vector as an arrowed line
    xe = x + 50 * np.cos(angle)
    ye = y - 50 * np.sin(angle)
    end_point = (int(xe), int(ye))
    cv2.arrowedLine(image, start_point, end_point, color, thickness)
    cv2.imshow('Vector', image)
    cv2.waitKey(0)
    image = np.zeros((500, 500, 3), dtype="uint8")
    cv2.destroyAllWindows()

