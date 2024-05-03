#!/usr/bin/python3.8
from hilbertcurve.hilbertcurve import HilbertCurve
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import igraph as ig
from numba import njit
import random


########## Hyperparameters ##########

# DPI control for visualisation
plt.rcParams["figure.dpi"] = 1200

# Iteration of Hilbert's curve
iter = 3

# Percentage of the board all of the obstacles should cover
MIN_OBSTACLE_COVERAGE_RATIO = 0.15

# The smallest side size of an obstacle. 
MIN_OBSTACLE_SIZE = 1

#####################################


# Dimension of Hilber's curve
dim = 2
# Number of points in Hilbert's curve
size = 2 ** (iter * dim)
side_size = int(2 ** (iter * dim / 2))


##### Secondary Hyperparameters #####

MAX_OBSTACLE_SIZE = int(side_size**0.5) if int(side_size**0.5) >= 1 else 1 

#####################################


# Area covered by the Hibert's curve
xmin = 0
ymin = 0
xmax = 10 * side_size
ymax = 10 * side_size
# Grid size
xgrid = 1
ygrid = 1
# Bounding grid
xmin_grid = xmin - xgrid / 2
ymin_grid = ymin - ygrid / 2
xmax_grid = xmax + xgrid / 2
ymax_grid = ymax + ygrid / 2

# Creating Hilbert's curve
hilbert_curve = HilbertCurve(iter, dim)
distances = list(range(size))
points = hilbert_curve.points_from_distances(distances)

x = np.array([points[i][0] for i in range(size)])
y = np.array([points[i][1] for i in range(size)])

########## Function Definitions ##########
@njit
def get_point_index(xl, yl):
    """Calculates the index of the point on the hilbert's curve"""
    global x, y
    x_index = np.where(x == xl)[0]
    y_index = np.where(y == yl)[0]
    return list(np.intersect1d(x_index, y_index))


def get_adjacent_nodes(i):
    """Outputs the adjacent nodes of a given node"""
    x_i = x[i]
    y_i = y[i]

    adjacent_points = {
        "p1": [x_i + xgrid, y_i],
        "p2": [x_i, y_i + ygrid],
        "p3": [x_i - xgrid, y_i],
        "p4": [x_i, y_i - ygrid],
    }

    adjacent_nodes = [
        get_point_index(adjacent_points[point][0], adjacent_points[point][1])
        if adjacent_points[point][0] in x and adjacent_points[point][1] in y
        else [-1]
        for point in adjacent_points
    ]

    return adjacent_nodes


def get_adjacency_list(v_list, obs_list):
    """Gives the Adjacency list"""
    v_neigh = g.neighborhood(vertices=v_list)
    result_set = {v for neigh_list in v_neigh for v in neigh_list} - \
        set(v_list) - set(obs_list)
    return list(result_set)


def get_req_path(subgraph_list, l):
    """Gives the smallest path present within the subgraph"""
    k = np.array(l)
    if np.shape(k)[0] == 1:
        return k
    k = k[:, 0:-1]
    for i in range(size):
        if set(k[i]).intersection(subgraph_list) == set(k[i]):
            return k[i]


def get_subgraph(v):
    """Generates subgraph for vertex list v"""
    o = ig.Graph(n=size)
    for i in v:
        neigh = get_adjacent_nodes(i)
        for j in range(4):
            if (
                neigh[j][0] != -1
                and o.are_connected(i, neigh[j][0]) == False
                and neigh[j][0] in v
            ):
                o.add_edge(i, neigh[j][0])
    o.vs["name"] = [str(i) for i in range(size)]
    o.vs["label"] = o.vs["name"]
    return o


def generateObstacles(field_size, obstacle_coverage_ratio, min_obstacle_size, max_obstacle_size):

    # Calculate the total number of cells in the navigation field.
    total_cells = field_size**2

    # Calculate the number of cells needed to cover 40% of the area.
    cells_to_cover = int(obstacle_coverage_ratio * total_cells)
    
    obstacle_coordinates = []

    # Generate obstacles until 40% of the area is covered.
    while len(obstacle_coordinates) < cells_to_cover:
        # Generate random coordinates and size for the obstacle.
        x_root = random.randint(0, side_size - 1)
        y_root = random.randint(0, side_size - 1)
        size_obst = random.randint(min_obstacle_size, max_obstacle_size)

        # Check if the obstacle fits within the bounds.
        if x_root + size_obst < side_size and y_root + size_obst < side_size:
            
            for i in range(x_root, x_root + size_obst):
                for j in range(y_root, y_root + size_obst):
                    
                    if [j,i] not in obstacle_coordinates:
                        obstacle_coordinates.append([j,i])
            
            
    return obstacle_coordinates


def generateFieldXY(field_size):
    coordinates = []
    for i in range(field_size):
        for j in range(field_size):
            coordinates.append([j,i])
            
    return coordinates

#################################################

########## Main Driver Code ##############

g = ig.Graph(n=size)

# graph Generation
for i in range(size):
    neigh = get_adjacent_nodes(i)
    for j in range(4):
        if neigh[j][0] != -1 and g.are_connected(i, neigh[j][0]) == False:
            g.add_edge(i, neigh[j][0])


random.seed()

obstacle_coordinates = generateObstacles(side_size, MIN_OBSTACLE_COVERAGE_RATIO, MIN_OBSTACLE_SIZE, MAX_OBSTACLE_SIZE)

obstacle= hilbert_curve.distances_from_points(obstacle_coordinates)


# Initiaize visited_node list with the starting node
visited_nodes = np.array([0])
obstacle_detected = np.array([])

i = get_subgraph(visited_nodes)

while get_adjacency_list(visited_nodes, obstacle_detected) != []:
    min_adj = min(get_adjacency_list(visited_nodes, obstacle_detected))
    o = get_subgraph(np.append(visited_nodes, min_adj))
    if min_adj == visited_nodes[-1] + 1:
        if min_adj in obstacle:
            obstacle_detected = np.append(obstacle_detected, min_adj)
        else:
            visited_nodes = np.append(visited_nodes, min_adj)
    else:
        l = get_req_path(
            visited_nodes, o.get_all_shortest_paths(
                visited_nodes[-1], to=min_adj)
        )
        if min_adj in obstacle:
            try:
                obstacle_detected = np.append(obstacle_detected, l[0][-1])
                visited_nodes = np.append(visited_nodes, l[0][1:-1])
            except:
                obstacle_detected = np.append(obstacle_detected, l[-1])
                visited_nodes = np.append(visited_nodes, l[1:-1])
        else:
            try:
                visited_nodes = np.append(visited_nodes, l[0][1:])
            except:
                visited_nodes = np.append(visited_nodes, l[1:])
##################################################

########## Graphing and Visualisation ###########

x_visited = [x[i] for i in visited_nodes]
y_visited = [y[i] for i in visited_nodes]

x_bound = [min(x) - xgrid/2, min(x) - xgrid/2, max(x) +
           xgrid/2, max(x) + xgrid/2, min(x) - xgrid/2]
y_bound = [min(y) - ygrid/2, max(y) + ygrid/2, max(y) +
           ygrid/2, min(y) - ygrid/2, min(y) - ygrid/2]

fig, ax = plt.subplots()

for i in range(size):
    if i in obstacle_detected:
        rectangle = plt.Rectangle(
            (x[i] - xgrid / 2, y[i] - ygrid / 2),
            1,
            1,
            fc="red",
            alpha=0.2,
            ec="black",
        )
        plt.gca().add_patch(rectangle)
    else:
        rectangle = plt.Rectangle(
            (x[i] - xgrid / 2, y[i] - ygrid / 2),
            1,
            1,
            fc="grey",
            alpha=0.04,
            ec="black",
        )
        plt.gca().add_patch(rectangle)
        
       
        
hilbert_grid = hilbert_curve.distances_from_points(generateFieldXY(side_size))
hilbert_grid.sort()
hilbert_grid = hilbert_curve.points_from_distances(hilbert_grid)
x_hilbert = [x[0] for x in hilbert_grid]
y_hilbert = [x[1] for x in hilbert_grid]



node_coord = [[x[i], y[i]] for i in range(size)]
visual_style = {}
g.vs["name"] = [str(i) for i in range(size)]

visual_style["edge_width"] = [0.3]
visual_style["edge_color"] = "white"
layout_subgraph = ig.Layout(coords=node_coord)

ig.plot(g, target=ax, layout=layout_subgraph, **visual_style)
plt.plot(x_hilbert, y_hilbert, color = "orange", linewidth=0.2)
plt.plot(x_bound, y_bound, linestyle="solid", color="black", linewidth=0.2)
#plt.plot(x_visited, y_visited, linestyle="solid", color="green", linewidth=0.3)

line2 = ax.plot(x_visited[0], y_visited[0],linestyle="solid", color="green", linewidth=0.3)[0]
scat = ax.scatter(x_visited[0], y_visited[0], color="red", s=0.3)

def update(frame):
    scat.set_offsets([x_visited[frame-1], y_visited[frame-1]])
    # update the line plot:
    line2.set_xdata(x_visited[:frame])
    line2.set_ydata(y_visited[:frame])
    return (line2)

ani = animation.FuncAnimation(fig=fig, func=update, frames=len(x_visited), interval=200)
plt.show()

minimal_moves = int(side_size**2 - len(obstacle))
moves = len(x_visited)
overshoot = int((moves/minimal_moves-1)*100)
print(f"Minimal possible moves: {minimal_moves}")
print(f"Actual moves: {moves}")
print(f"Overshot moves by {overshoot}%")
