# Constants
WIDTH = 600  # Width of the map
HEIGHT = 250  # Height of the map
THETA_STEPS = 30  # Degrees

# Matrix dimensions
x_dim = int(WIDTH / 0.5)  # 1200
y_dim = int(HEIGHT / 0.5)  # 500
theta_dim = int(360 / THETA_STEPS)  # 12

# Initialize the visited matrix with zeros
V = [[[0 for _ in range(theta_dim)] for _ in range(x_dim)] for _ in range(y_dim)]

def mark_visited(x, y, theta):
    # Convert to discrete indices
    x_index = int(x * 2)
    y_index = int(y * 2)
    theta_index = int(theta / THETA_STEPS)

    # Mark as visited
    V[y_index][x_index][theta_index] = 1

def is_visited(x, y, theta):
    # Convert to discrete indices
    x_index = int(x * 2)
    y_index = int(y * 2)
    theta_index = int(theta / THETA_STEPS)

    # Return visited status
    return V[y_index][x_index][theta_index] == 1

# Example usage
mark_visited(3.2, 4.7, 0)
mark_visited(10.2, 8.8, 30)
if not is_visited(10.1, 8.9, 30):  # This will return True since it's considered visited
    print("Node is not a duplicate.")
else:
    print("Node is a duplicate.")
