# Case Study 1: Robot Traversing a Grid
This readme discusses the navigation problem of a robot traversing a square grid with random obstacles. The documentation is also included here. <br>

[Run the code!](#running-the-code)

![Grid Progression](/gridgif.gif)

- [Case Study 1: Robot Traversing a Grid](#case-study-1-robot-traversing-a-grid)
  - [Summary \& Overview](#summary--overview)
  - [Primary Hyperparameters](#primary-hyperparameters)
    - [`plt.rcParams`](#pltrcparams)
    - [`iter`](#iter)
    - [`MIN_OBSTACLE_COVERAGE_RATIO`](#min_obstacle_coverage_ratio)
    - [`MIN_OBSTACLE_SIZE`](#min_obstacle_size)
  - [Secondary/Optional Hyperparameters](#secondaryoptional-hyperparameters)
    - [`MAX_OBSTACLE_SIZE`](#max_obstacle_size)
  - [Functions](#functions)
    - [`get_point_index(x,y)`](#get_point_indexxy)
    - [`get_adjacent_nodes(i)`](#get_adjacent_nodesi)
    - [`get_adjacency_list(v_list, obs_list)`](#get_adjacency_listv_list-obs_list)
    - [`get_req_path(subgraph_list,l)`](#get_req_pathsubgraph_listl)
    - [`get_subgraph(v)`](#get_subgraphv)
    - [`generateObstacles(field_size, ratio, min_obstacle, max_obstacle)`](#generateobstaclesfield_size-ratio-min_obstacle-max_obstacle)
    - [`generateFieldXY(field_size)`](#generatefieldxyfield_size)
  - [Driver Code](#driver-code)
  - [Running the code](#running-the-code)

## Summary & Overview
This algorithm is based on [this paper](https://arxiv.org/abs/2308.02200) for solving the grid traversal with obstacles problem: <br>
Given an $N\times N$ square grid with arbitrarily sized rectangular obstacles, minimize the number of steps(up/down/left/right) the robot must make to traverse the entire grid. <br>
In our implementation, the Hilbert curve is used (although any space filling curve is viable) based on [this source code](https://github.com/wakodeashay/skc).<br> 
The gist is to follow the space filling curve as far as we can, and where we cannot(because of an obstacle), use Dijkstra's algorithm to get us to the next viable position with the smallest Hilbert number.<br>
Since the Hilbert curve only works for grids with side length $2^n$, we could work on $N\times N$ grids by finding the smallest integer $n$ such that $2^n > N$, and filling the excess space with obstacles. In this script, we only consider the cases where $N = 2^n$ for simplicity. The underlying logic for navigation remains the same.<br>
We use the `igraph` library to set up a graph containing the visited gridpoints, augmented with the next possible set of positions. We then use Dijkstra's algorithm (`get_all_shortest_paths()` method) to find the shortest path to our desired position.<br>
It is recommended to first [run the code](#running-the-code) as-is, and quickly read through [`main.py`](/src/main.py), as well as the documentation on the [primary](#primary-hyperparameters) and [secondary](#secondaryoptional-hyperparameters) hyperparameters. It will be much simpler to understand once you'ce played with the algorithm a few times.<br>
Then feel free to read the [Functions](#functions) and [Driver Code](#driver-code) sections. <br>
If there are rendering issues, refer to the [pltrcparams](#pltrcparams) section, and make sure your IDE is using the appropriate backend. I used `TKikner` in the Spyder IDE, and made sure the animation was rendering as an independent window, outside of the terminal.<br> 
<br> 
The source code is divided into distinct sections, implying they should exist in separate files. Since `main.py` is only 300 lines of code, it's all grouped together.<br>
<br>
The script `generate_environment.py` is deprecated and unused. It was the original script in this repo. The functions it contains were adapted and incorporated into `main.py`
<br>
<br>
<br>

## Primary Hyperparameters
These parameters control the primary behaviors of the algorithm, the size of the board, the obstacles, and some visuals.


### `plt.rcParams`
This controls the resolution of the board when it is visualised. Currently set to `1200`. By default, `matplotlib` sets it to `100`. If your visualisation looks strange, try changing this number. If the issues persist, change the backend visualiser used by your IDE. In my case, I used `TKinker` backend rendered in the Spyder IDE.


### `iter`
This controls the iteration number of our Hilbert curve. It determines the size of the board. It grows in powers of 2, so if `iter = 3` then the side length is `16`, if `iter = 4` then the side length is `32` and so on.
$$side\ length = 2^{iter}$$
It is **NOT** recommended to exceed `iter = 5` as it becomes computationally demanding to run the algorithm. If you are confident in your hardware, feel free to increase it.


### `MIN_OBSTACLE_COVERAGE_RATIO`
This fraction is the minimum proportion of the grid that the obstacles should cover. So if `MIN_OBSTACLE_COVERAGE_RATIO = 0.2`, then roughly `20%` of the grid should have some obstacle on it. It's a minimum since these proportions cannot be exact for an arbitrary grid, so if at least some minimum percentage has obstacles, stop generating them. <br>
For example, if we have `19.5%` coverage, and the next obstacle will push it to `22%`, then stop generating new obstacles after `22%`, since we've crossed the minimum threshold.<br>
In essence,
$$proportion\ of\ obstacle\ coverage \approx MIN\\_ OBSTACLE\\_ COVERAGE\\_ RATIO$$

In general it should be bound between these values:
$$0 \le MIN \\_ OBSTACLE \\_ COVERAGE \\_ RATIO \ll 1$$


### `MIN_OBSTACLE_SIZE`
This determines the minimum side length of an obstacle. By default its fine to leave it at `1`, that way there is a mix of large and small obstacles.<br>
In general it should be bound between these values:
$$1 \le MIN\\_OBSTACLE\\_SIZE \le MAX\\_OBSTACLE\\_SIZE$$
If `MIN_OBSTACLE_SIZE = MAX_OBSTACLE_SIZE`, then all generated obstacles will be the same fixed size.
<br>
<br>
<br>

## Secondary/Optional Hyperparameters
These parameters do not need to be changed, other than for experimentation and curiosity. They still effect the underlying behaviour of the algorithm, but for presentation purposes, their default value is sufficient to understand what is happening.


### `MAX_OBSTACLE_SIZE`
This controls the maximum side length of a given obstacle. This should always be smaller than the side length of the grid. By default this is set to $\sqrt{side\ length}$ for aesthetic purposes. It "looks" right at this size, but of course you can freely change this. In case the grid is very small, the smallest value it can take is `1`.<br>
In general, it should be bound between these values:
$$MIN\\_OBSTACLE\\_SIZE \le MAX\\_OBSTACLE\\_SIZE \ll side\ length$$
<br>
<br>

## Functions
These are used in the driver code, or by eachother as helper functions. They mainly serve to set up the infrastructure necessary to run the main logic.
### `get_point_index(x,y)`
For a given Hilbert curve (in global scope), returns the index at which the point $(x,y)$ is located within it. It does **Not** return the Hilbert coordinate `H` of the $(x,y)$ pair, but the index `i`, such that  `Hilbert[i] = H`.<br>
Since `x,y` are part of the grid, they are type `int`. <br>
This function decorated with the `njit` function of the `numba` library, to make it run faster. <br>
Example of use:

```python
>> iter = 1 # A 2x2 grid
>> ...
>> get_point_index(0,0)
>> 0
>> get_point_index(1,0)
>> 3
>> get_point_index(1,1)
>> 2
```

### `get_adjacent_nodes(i)`
Returns the Hilbert indices of the points directly adjacent (up/down/left/right) to the point of consideration, $(x,y)$ with Hilbert index `i`. It uses `get_point_index` as a helper function.<br>
Suppose $(x,y)$ is on Hilbert point `H` of `Hilbert`. Then its' Hilbert index `i` is such that `Hilbert[i] = H`. This function returns the Hilbert indices of the points $(x+1,y), (x-1,y), (x,y+1),$ and $(x,y-1)$.<br>
In case we are on the edge of the grid, it only returns the indices within the grid. <br>
Since `i` is an index, it is of type `int`. <br>
It is used by itself, and also as a helper function in `get_subgraph`. <br>
Example of use:
```python
>> iter = 2 # A 4x4 grid
>> ...
>> get_adjacent_nodes(8) # grid coordinate (2,2)
>> [7,9,11,13]
```

### `get_adjacency_list(v_list, obs_list)`
Returns Hilbert indices of the non-visited gridpoints that neighbour the visited gridpoints, **and** aren't detected as obstacles.
- `v_list` is a list containing the Hilbert indices of the already visited gridpoints, each index of type `int`
- `obs_list` is a list containing the Hilbert indices of the detected obstacles, each index of type `int`

Example of use:
```python
>> iter = 2 # A 4x4 grid
>> ... # Obstacles at (0,1), (2,2)
>> ... # Currently we are on index Hilbert 2
>> ... # v_list = [0,1,2], obs_list = [3]
>> get_adjacency_list(v_list, obs_list)
>> [7,13,14]
```
### `get_req_path(subgraph_list,l)`
This function returns the first common path between `subgraph_list` and `l`. If `l` has only one path, return it instead.<br>
- `subgraph_list` is a list containing Hilbert indices, each index of type `int`.
- `l` is an `igraph` object of the `igraph` library.

The return is a list of Hilbert indices, each index representing a path segment on the grid.<br>
Typically we use this function to find the shortest viable path between two Hilbert indices on the grid. `subgraph_list` would contain the visited gridpoints and the next desired position. `l` contains the set of shortest past between the robot's current position, and the next desired position.<br>
Since `l` is an `igraph` object, it's typically created by calling the `get_all_shortest_paths()` method. From some larger graph, `o`, which is a subgraph of the current grid, we create `l` as follows: `l = o.get_all_shortest_paths(visited_nodes[-1], to = desired_node)`. `visited_nodes[-1]` refers to the robots' current position. Internally, the `get_all_shortest_paths` method uses Dijkstra's algorithm.<br>
Example of use:

```python
>> iter = 2 # A 4x4 grid
>> ... # Obstacles at (0,1), (0,2)
>> ... # We've visited Hilbert indices [0,1,2]
>> ...
>> get_req_path(subgraph_list,l)
>> [7,6] # The order matters(first step, second step, ...)
```

### `get_subgraph(v)`
This function generates a graph connecting a given set of Hilbert indices to their adjacent neighbours within those indices. <br>
This function is typically used to construct all viable next positions for the robot when an obstacle is present.<br>
The input `v` is a list of indices, each index of type `int`. <br>
The output is an `igraph` object from the `igraph` library, containing a set of connected nodes.<br>
Example of use:
```python
>> iter = 2 # A 4x4 grid
>> ... 
>> get_subgraph([2,7,11])
>> igraph Object at address 0x42683...
>> # igraph object where indices 2,7 are connected with an edge
```


### `generateObstacles(field_size, ratio, min_obstacle, max_obstacle)`
This function generates a list of lists, each containing the $(x,y)$ coordinates of the obstacles on the grid. For a given obstacle, it is decomposed into the coordinates that make it up, and is appended to the output.The positions of a given obstacle are randomly generated.<br>
- `field_size` is the side length of our board, it bounds the possible coordinates of the obstacles. It is an `int`. <br>
 - `ratio` is the proportion of the board that should be covered by obstacles. It is a `float`, bound within $[0,1[$
 - `min_obstacle` is the minimum side length an obstacle could have. It is an `int`.
 - `max_obstacle` is the maximum side length an obstacle could have. It is an `int`.

The two latter arguments must be bound according to: <br> 
$1 \le$ `min_obstacle` $\le$ `max_obstacle` $\lt$ `field_size`
<br>
Example of use:
```python
>> generateObstacles(4, 0.25, 1, 2)
>> [[1,1],[1,2],[2,1],[2,2]] # single 2x2 obstacle
>>
>> generateObstacles(4, 0.25, 1, 2)
>> [[1,1],[2,2],[0,3],[1,2]] # four 1x1 obstacles
``` 


### `generateFieldXY(field_size)`
This function produces a list of lists, each sub-list containing $(x,y)$ integer coordinate pairs of the entire grid. `field_size` specifies the side length of our grid, and should be and `int`.<br>
Example of use:
```python
>> generateFieldXY(2) # A 2x2 grid
>> [[0,0],[0,1],[1,0],[1,1]]
``` 
<br>
<br>

## Driver Code
Here we explain the main logic of the algorithm, along with the relevant variables.<br>
- The `obstacle` variable is a list containing the Hilbert indices of all the obstacles.
- The `visited_nodes` variable is an array containing the sequence of Hilbert indices we have already visited. Since the robot may go over the same path again, it may contain multiples of the same index. But the `unique` elements are the already visited gridpoints. It is initialised to `[0]` as this is the starting position.
- The `obstacles_detected` variable is an array containing the Hilbert indices of the obstacles the robot has encountered up until its' current position. It is initialised as an empty array since the robot hasn't encountered anything the moment it starts.
```python
obstacle= hilbert_curve.distances_from_points(obstacle_coordinates)

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
```
The condition `get_adjacency_list(visited_nodes, obstacle_detected) != []` continuously checks if there are any unvisited, and accessible, gridpoints. It's set up so that it **won't** include closed-off regions.<br>
This statement:

```python
    min_adj = min(get_adjacency_list(visited_nodes, obstacle_detected))
    o = get_subgraph(np.append(visited_nodes, min_adj))
```
Looks for the next smallest Hilbert index, `min_adj`, and creates a sub graph of the grid,`o`, among the `visited_nodes` that includes `min_adj`. By nature of the Hilbert curve, the next smallest index will either directly neighbour an already visited gridpoint, or an obstacle(ie: The next smallest index  will always neighbour the obstacle that neighbours the visited gridpoints). 

This statement:

```python
    if min_adj == visited_nodes[-1] + 1:
        if min_adj in obstacle:
            obstacle_detected = np.append(obstacle_detected, min_adj)
        else:
            visited_nodes = np.append(visited_nodes, min_adj)
```
Checks if the next smallest Hilbert gridpoint is immediately accessible. If there is no obstacle, we move to it(the `else` statement). If we cannot go there(so it's an obstacle), add the index to the list of detected obstacles, `obstacles_detected`.

This statement:

```python
    else:
        l = get_req_path(
            visited_nodes, o.get_all_shortest_paths(
                visited_nodes[-1], to=min_adj)
        )
```
Finds the shortest path (`l`), using Dijkstra, from the robots' current position (`visited_nodes[-1]`) to the desired position `min_adj`. This is run when `min_adj` is **not** the increment of the robots' current position (because its blocked by an obstacle, or is an obstacle neighbouring another obstacle).

This statement:

```python
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
```
Checks if `min_adj` is an obstacle(or not) in the case that `min_adj` **isn't** and increment of the robots' current position.<br>
If it's an obstacle, we follow Dijkstra until its' second-to-last gridpoint (so we don't crash into the obstacle), and append `min_adj`'s index into `obstacle_detected`.
<br>
If it isn't an obstacle, we follow Dijkstra's path directly, and we append it to the `visited_nodes`.<br>
The `try/except` clauses deals with the edge of the board, but their logic is equivalent.
<br>
<br>

## Running the code
The relevant code is in the [src](/src/) folder. First run [requirements.sh](/src/requirement.sh) to install the necessary libraries. Then run [main.py](/src/main.py) to view the results. Feel free to change the hyperparameters. <br>
It is not recommended to change the `iter` parameter above 5. It is computationally demanding and will take a long time to execute.<br>
When you close the animation window, something similar to the following should appear in your terminal:

![terminal_output](/terminal.PNG)

<br>
`minimal possible moves` refers to the theoretical minimal amount of moves the robot could do. This number isn't necessarily achievable, since it depends on how the obstacles are placed. It just counts the total number of gridpoints that aren't covered by an obstacle.<br>
As `actual moves` implies, its the total number of moves done by the robot to complete the grid.<br>
`overshot moves by` is the percentage difference between the `minimal possible moves` and the `actual moves.`