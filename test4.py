import cv2
import numpy as np

# Create a blank image, +1 in dimensions for boundary conditions
image = np.zeros((500, 500, 3), dtype="uint8")

# Define the starting and ending points of the vector
start_point = (100, 100)  # (x1, y1)
end_point = (400, 400)    # (x2, y2)

# Define the color of the arrow (BGR format) and the thickness
color = (255, 0, 0)  # Blue color in BGR
thickness = 2

# Draw the vector as an arrowed line
cv2.arrowedLine(image, start_point, end_point, color, thickness)

# Show the image
cv2.imshow('Vector', image)
cv2.waitKey(0)
cv2.destroyAllWindows()