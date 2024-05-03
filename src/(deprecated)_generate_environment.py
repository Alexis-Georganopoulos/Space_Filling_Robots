import numpy as np
import random
import matplotlib.pyplot as plt

def generateEnvironment(field_size, obstacle_coverage_ratio, min_obstacle_size, max_obstacle_size):
    # Create a square filled with zeros (0 represents an empty cell).
    navigation_field = np.zeros((FIELD_SIZE, FIELD_SIZE), dtype=int)

    # Calculate the total number of cells in the navigation field.
    total_cells = navigation_field.size

    # Calculate the number of cells needed to cover 40% of the area.
    cells_to_cover = int(MIN_OBSTACLE_COVERAGE_RATIO * total_cells)

    # Generate obstacles until 40% of the area is covered.
    while np.sum(navigation_field) < cells_to_cover:
        # Generate random coordinates and size for the obstacle.
        x_root = random.randint(0, FIELD_SIZE - 1)
        y_root = random.randint(0, FIELD_SIZE - 1)
        size_obst = random.randint(MIN_OBSTACLE_SIZE, MAX_OBSTACLE_SIZE)  # Adjust the size range as needed.

        # Check if the obstacle fits within the bounds.
        if x_root + size_obst < FIELD_SIZE and y_root + size_obst < FIELD_SIZE:
            # Add the obstacle to the navigation field.
            navigation_field[x_root:x_root + size_obst, y_root:y_root + size_obst] = 1

    return navigation_field


if __name__ == '__main__':
    MIN_OBSTACLE_COVERAGE_RATIO = 0.1
    FIELD_SIZE = 100
    MIN_OBSTACLE_SIZE = 5
    MAX_OBSTACLE_SIZE = 20

    # Can also drop if you'd like to have real randomness. But would keep to have better reproducibility
    random.seed(10)

    navigation_field = generateEnvironment(FIELD_SIZE, MIN_OBSTACLE_COVERAGE_RATIO, MIN_OBSTACLE_SIZE, MAX_OBSTACLE_SIZE)

    # Create a figure and axis for the plot.
    fig, ax = plt.subplots()

    # Plot the navigation field using imshow.
    # Use cmap='binary' to display 0s (empty cells) as white and 1s (covered cells) as black.
    cax = ax.matshow(navigation_field, cmap='binary')

    # Show the plot.
    plt.show()

#%%
plt.rcParams["figure.dpi"] = 100
MIN_OBSTACLE_COVERAGE_RATIO = 0.2
FIELD_SIZE = 16
MIN_OBSTACLE_SIZE = 1
MAX_OBSTACLE_SIZE = int(FIELD_SIZE**0.5)


def generateEnvironment(field_size, obstacle_coverage_ratio, min_obstacle_size, max_obstacle_size):
    # Create a square filled with zeros (0 represents an empty cell).
    navigation_field = np.zeros((FIELD_SIZE, FIELD_SIZE), dtype=int)

    # Calculate the total number of cells in the navigation field.
    total_cells = navigation_field.size
    print(total_cells)
    # Calculate the number of cells needed to cover 40% of the area.
    cells_to_cover = int(MIN_OBSTACLE_COVERAGE_RATIO * total_cells)
    
    obstacle_coordinates = []

    # Generate obstacles until 40% of the area is covered.
    while len(obstacle_coordinates) < cells_to_cover:
        # Generate random coordinates and size for the obstacle.
        x_root = random.randint(0, FIELD_SIZE - 1)
        y_root = random.randint(0, FIELD_SIZE - 1)
        size_obst = random.randint(MIN_OBSTACLE_SIZE, MAX_OBSTACLE_SIZE)  # Adjust the size range as needed.

        # Check if the obstacle fits within the bounds.
        if x_root + size_obst < FIELD_SIZE and y_root + size_obst < FIELD_SIZE:
            # Add the obstacle to the navigation field.
            navigation_field[x_root:x_root + size_obst, y_root:y_root + size_obst] = 1
            
            for i in range(x_root, x_root + size_obst):
                for j in range(y_root, y_root + size_obst):
                    if [j,i] not in obstacle_coordinates:
                        obstacle_coordinates.append([j,i])
            
            
    return navigation_field, obstacle_coordinates

random.seed()

navigation_field, obstacle_coordinates = generateEnvironment(FIELD_SIZE, MIN_OBSTACLE_COVERAGE_RATIO, MIN_OBSTACLE_SIZE, MAX_OBSTACLE_SIZE)

# Create a figure and axis for the plot.
fig, ax = plt.subplots()

# Plot the navigation field using imshow.
# Use cmap='binary' to display 0s (empty cells) as white and 1s (covered cells) as black.
cax = ax.matshow(navigation_field, cmap='binary')

# Show the plot.
plt.show()