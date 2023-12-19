import numpy as np
import sympy as sp
from shapely.geometry import LineString
from matplotlib.path import Path
import matplotlib.pyplot as plt
from python_tsp.heuristics import solve_tsp_simulated_annealing
import math
import random

#function for checking if two circles overlap
def check_overlap_circle(O1,O2,r):
    """
    A function which inputs two circle center locations [x,y] and detects whether they intersect
    """
    
    if (min([0,(O1[0] - O2[0])**2 + (O1[1] - O2[1])**2 - 4*r**2]) == 0):
        
        intersect = 0
        return intersect

    intersect =1

    return intersect


def construct_q_list(vertex_mat, inside_point, r):
    """
    a function which takes in a list of vertices and finds a list of inside (q) points that create
    a contour in which a circle of size r can always be placed without intersecting the original polygon boundaries.
    """
    num_pts = len(vertex_mat) - 1
    q_mat = np.zeros((num_pts, 2))

    #we feed in a point that is already inside the contour to help with line intersect detection
    #later this point is replaced with the previous q point which is guaranteed to be inside the shape.
    last_q = inside_point
    last_vertex = vertex_mat[-2]
    
    for i in range(num_pts):
        next_q = find_q_point(last_vertex, vertex_mat[i], vertex_mat[i + 1], last_q, r)
        q_mat[i] = next_q
        last_q = next_q
        last_vertex = vertex_mat[i]

    return q_mat

def find_q_point(P0, P1, P2, Pq0, r):
    """
    Find the next inside vertex using a corner of the original polygon and a known inside point
    
    """
    #inside point will be on the bisector line of the corner
    offset_vector = find_bisector(P0, P1, P2, Pq0)
    
    #the following code checks all ways to place a q point on a vertex. There are 2 ways to place a point per
    #bisector vector at a corner and there are 2 bisector vectors per corner. The find_bisector function will 
    #always return the correct bisector because of internal checksIf a line is drawn from the last
    #q point to the new point and it intersects with one of the corner lines, then it must be outside the contour, similarly
    #the circle placed cannot intersect with one of the boundary vertices. The 2 extra cases checked are for if the vertex is convex.

    # Try concave options first
    candidate_pq = P1 + r * offset_vector
    if (check_line_intersect(Pq0[0], Pq0[1], candidate_pq[0], candidate_pq[1], P0[0], P0[1], P1[0], P1[1]) == 0 and
        circle_line_intersect(P0[0], P0[1], P1[0], P1[1], candidate_pq[0], candidate_pq[1], r) == 0 and
        check_line_intersect(Pq0[0], Pq0[1], candidate_pq[0], candidate_pq[1], P2[0], P2[1], P1[0], P1[1]) == 0):
        return candidate_pq

    candidate_pq = P1 - r * offset_vector
    if (check_line_intersect(Pq0[0], Pq0[1], candidate_pq[0], candidate_pq[1], P0[0], P0[1], P1[0], P1[1]) == 0 and
        circle_line_intersect(P0[0], P0[1], P1[0], P1[1], candidate_pq[0], candidate_pq[1], r) == 0 and
        check_line_intersect(Pq0[0], Pq0[1], candidate_pq[0], candidate_pq[1], P2[0], P2[1], P1[0], P1[1]) == 0):
        return candidate_pq

    # Handle convex vertex
    # Different geometry is required to figure out the correct offset for a convex vertex (it is not just the radius)
    l_a = np.linalg.norm(P1 - P2)
    l_b = np.linalg.norm(P1 - P0)
    l_c = np.linalg.norm(P0 - P2)

    angle = np.arccos((l_a**2 + l_b**2 - l_c**2) / (2 * l_a * l_b))
    convex_offset = r / np.sin(angle / 2)

    candidate_pq = P1 + convex_offset * offset_vector
    if (check_line_intersect(Pq0[0], Pq0[1], candidate_pq[0], candidate_pq[1], P0[0], P0[1], P1[0], P1[1]) == 0 and
        check_line_intersect(Pq0[0], Pq0[1], candidate_pq[0], candidate_pq[1], P2[0], P2[1], P1[0], P1[1]) == 0):
        return candidate_pq

    return P1 - convex_offset * offset_vector




def circle_line_intersect(lx1, ly1, lx2, ly2, cx, cy, r):
    """
    Check if a circle and a line segment intersect
    
    """
    # Handle the special case of a vertical line
    if lx2 == lx1:
        slope = float('inf')
        intercept = lx2
    else:
        slope = (ly2 - ly1) / (lx2 - lx1)
        intercept = ly1 - slope * lx1

    # Define symbolic variables
    x, y = sp.symbols('x y')

    # Define the line equation
    if slope == float('inf'):
        line_eq = sp.Eq(x, intercept)
    else:
        line_eq = sp.Eq(y, slope * x + intercept)

    # Define the circle equation
    circle_eq = sp.Eq((x - cx)**2 + (y - cy)**2, (0.99 * r)**2)

    # Find intersection points
    intersection_points = sp.solve((line_eq, circle_eq), (x, y))

    for point in intersection_points:
        x_val, y_val = point
        if isinstance(x_val, sp.core.numbers.Float) and isinstance(y_val, sp.core.numbers.Float):
            if min(lx1, lx2) <= x_val.evalf() <= max(lx1, lx2):
                if (x_val.evalf() == lx1 and y_val.evalf() == ly1) or (x_val.evalf() == lx2 and y_val.evalf() == ly2):
                    continue
                return 1

    return 0

def check_line_intersect(l1x1, l1y1, l1x2, l1y2, l2x1, l2y1, l2x2, l2y2):
    """
    Check if 2 lines intersect.
    """
    line1 = LineString([(l1x1, l1y1), (l1x2, l1y2)])
    line2 = LineString([(l2x1, l2y1), (l2x2, l2y2)])

    # Check if the two lines intersect
    if line1.intersects(line2):
        return 1
    else:
        return 0

def find_bisector(P0, P1, P2, Pq0):
    """
    This function finds the bisector of an angle. Two bisectors technically exist for every two line intersections.
    Using an internal point it always ensures that the correct bisector is found.
    
    """

    bisector_slope = np.tan(0.5 * (np.arctan2((P2[1] - P1[1]), (P2[0] - P1[0])) + np.arctan2((P0[1] - P1[1]), (P0[0] - P1[0]))))
    offset_vector = np.array([1, bisector_slope])
    offset_vector /= np.linalg.norm(offset_vector)
    offset_vector_2 = np.array([-offset_vector[1], offset_vector[0]])

    points = np.array([[P1[0] + offset_vector[0], P1[1] + offset_vector[1]],
                       [P1[0] - offset_vector[0], P1[1] - offset_vector[1]],
                       [P1[0] + offset_vector_2[0], P1[1] + offset_vector_2[1]],
                       [P1[0] - offset_vector_2[0], P1[1] - offset_vector_2[1]]])

    for i in range(4):
        chk = points[i]

        if (check_line_intersect(Pq0[0], Pq0[1], chk[0], chk[1], P0[0], P0[1], P1[0], P1[1]) == 0 and
            check_line_intersect(Pq0[0], Pq0[1], chk[0], chk[1], P2[0], P2[1], P1[0], P1[1]) == 0):
            direction = offset_vector_2 if i > 1 else offset_vector
            return direction

    # If no direction is found, return None or raise an error as appropriate
    return None

def place_circles(boundary_mat, radius, P_start, div_num):
    """
    Use Depth first search to place circles in the q boundary.
    """
    center_list = [P_start]
    queue = [0]
    angular_step = 2 * np.pi / div_num

    while queue:
        main_center = center_list[queue[-1]]
        for i in range(div_num):
            cand_pt = [
                main_center[0] + 2 * radius * np.cos(np.pi - angular_step * i),
                main_center[1] + 2 * radius * np.sin(np.pi - angular_step * i)
            ]

            #only place a circle if the point is within the correct region and does not overlap another circle.
            if check_point_inregion(cand_pt, boundary_mat) and not check_all_overlap(center_list, cand_pt, radius):
                #print(center_list)
                center_list.append(cand_pt)
                queue.append(len(center_list) - 1)
                break
            if i == div_num - 1:
                queue.pop()
        
    #print(center_list)
    
    return np.array(center_list)

def place_circles_bfs(boundary_mat, radius, P_start, div_num):
    """
    A BFS implementation of the circle placer.
    """
    center_list = [P_start]
    queue = [0]
    angular_step = 2 * np.pi / div_num

    while queue:
        main_center = center_list[queue[0]]
        for i in range(div_num):
            cand_pt = [
                main_center[0] + 2 * radius * np.cos(np.pi - angular_step * i),
                main_center[1] + 2 * radius * np.sin(np.pi - angular_step * i)
            ]
            #only place a circle if the point is within the correct region and does not overlap another circle.
            if check_point_inregion(cand_pt, boundary_mat) and not check_all_overlap(center_list, cand_pt, radius):
                #print(center_list)
                center_list.append(cand_pt)
                queue.append(len(center_list) - 1)
                break
            if i == div_num - 1:
                queue.pop(0)
        
    #print(center_list)
    
    return np.array(center_list)

def check_point_inregion(Point, vertex_mat):
    """
    Check if a point lies within a closed region.    
    """
    
    path = Path(vertex_mat)
    
    # Use the contains_point method to check if the point is inside the polygon
    return 1 if path.contains_point(Point) else 0
    
def check_all_overlap(center_list, circ_cand, radius):
    for center in center_list:
        if check_overlap_circle(center, circ_cand, radius):
            return 1
    return 0

def calculate_total_distance(points, order):
    """
    Calculate the total distance of the path described by the given order of points.
    """
    return sum(np.linalg.norm(np.array(points[order[i - 1]]) - np.array(points[order[i]])) for i in range(len(order)))

def swap_two_cities(order):
    """
    Swap two cities in the order, excluding the first city. Not swapping the first city ensures the robot startpoint is unchanged.
    """
    idx1, idx2 = random.sample(range(1, len(order)), 2)  # Exclude the first city from swapping
    order[idx1], order[idx2] = order[idx2], order[idx1]
    return order

def simulated_annealing(points, temperature=10000, cooling_rate=0.99, stopping_temperature=1e-8, stopping_iter=10000):
    """
    Perform the simulated annealing algorithm to solve the traveling salesman problem
    """
    current_order = list(range(len(points)))
    best_order = current_order[:]
    current_distance = calculate_total_distance(points, current_order)
    best_distance = current_distance
    iteration = 1

    while temperature > stopping_temperature and iteration < stopping_iter:
        candidate_order = swap_two_cities(current_order.copy())
        candidate_distance = calculate_total_distance(points, candidate_order)
        
        # Probability of accepting worse solution
        acceptance_probability = math.exp(-abs(candidate_distance - current_distance) / temperature)
        
        # Decide if we should accept the new order
        if candidate_distance < current_distance or random.random() < acceptance_probability:
            current_order = candidate_order
            current_distance = candidate_distance
        
        # Update the best order and distance found
        if candidate_distance < best_distance:
            best_order = candidate_order
            best_distance = candidate_distance
        
        # Cool down the temperature and increment iteration count
        temperature *= cooling_rate
        iteration += 1

    return best_order, best_distance

def plot_best_path(points, best_order):
    """
    A plotting helper function for the TSP solver. Deprecated.
    """
    # Reorder the points according to the best order
    ordered_points = [points[i] for i in best_order]

    # Separate the coordinates into two lists for plotting
    x_coords, y_coords = zip(*ordered_points)

    # Plot the points
    plt.figure(figsize=(10, 6))
    plt.plot(x_coords, y_coords, 'o-', label='Path')
    
    # Plot the start and end points
    plt.plot(x_coords[0], y_coords[0], 'go', label='Start')  # Start point
    plt.plot(x_coords[-1], y_coords[-1], 'ro', label='End')  # End point

    # Add labels and title
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.title('Traveling Salesman Path')

    # Add a legend
    plt.legend()

    # Show the plot


def create_distance_matrix(points):
    """
    Creates a distance matrix from a list of (x, y) coordinates.

    Args:
    - points: A list of tuples, where each tuple represents the (x, y) coordinates of a point.

    Returns:
    - A 2D numpy array representing the distance matrix.
    """
    num_points = len(points)
    distance_matrix = np.zeros((num_points, num_points))

    for i in range(num_points):
        for j in range(num_points):
            if i != j:
                distance_matrix[i, j] = np.linalg.norm(np.array(points[i]) - np.array(points[j]))

    return distance_matrix


def get_waypoint_list(boundary_vertices,r):
    """
    Path plan a route through a known closed area using circle packing. 
    The function that actually runs on the robot.

    r = radius to pack
    boundary_vertices = numpy array full of x,y pairs representing a closed boundary.
    """
    

    inside_pt = np.array([1, 1.2])
    #r = .4

    # Construct the q_list using the provided function
    q_mat = construct_q_list(boundary_vertices, inside_pt, r)

    #center_list = path_plan.place_circles(q_mat,r,q_mat[0,:],60)

    center_list = place_circles_bfs(q_mat,r,q_mat[0,:],60)


    dist_mat = create_distance_matrix(center_list)
    permutation, distance = solve_tsp_simulated_annealing(dist_mat)
    
    order = permutation
    items = center_list

    # Pair each item with its order, then sort by order
    paired = zip(order, items)
    sorted_pairs = sorted(paired)

    # Extract the items from the sorted pairs
    sorted_items = [item for _, item in sorted_pairs]
    return sorted_items

    #return center_list,permutation
