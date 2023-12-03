import numpy as np
import sympy as sp
from shapely.geometry import LineString
from matplotlib.path import Path
import matplotlib.pyplot as plt

#function for checking if two circles overlap
def check_overlap_circle(O1,O2,r):
    
    if (min([0,(O1[0] - O2[0])**2 + (O1[1] - O2[1])**2 - 4*r**2]) == 0):
        
        intersect = 0
        return intersect

    intersect =1

    return intersect


def construct_q_list(vertex_mat, inside_point, r):
    num_pts = len(vertex_mat) - 1
    q_mat = np.zeros((num_pts, 2))

    last_q = inside_point
    last_vertex = vertex_mat[-2]

    for i in range(num_pts):
        next_q = find_q_point(last_vertex, vertex_mat[i], vertex_mat[i + 1], last_q, r)
        q_mat[i] = next_q
        last_q = next_q
        last_vertex = vertex_mat[i]

    return q_mat

def find_q_point(P0, P1, P2, Pq0, r):
    # Assuming find_bisector, check_line_intersect, and circle_line_intersect are defined elsewhere
    offset_vector = find_bisector(P0, P1, P2, Pq0)

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
    line1 = LineString([(l1x1, l1y1), (l1x2, l1y2)])
    line2 = LineString([(l2x1, l2y1), (l2x2, l2y2)])

    # Check if the two lines intersect
    if line1.intersects(line2):
        return 1
    else:
        return 0

def find_bisector(P0, P1, P2, Pq0):
    # Function to check line intersection, assumed to be defined elsewhere
    # def check_line_intersect(x1, y1, x2, y2, x3, y3, x4, y4):
    #     ...

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
            #print(f"point is in region {check_point_inregion(cand_pt, boundary_mat)}")

            #print(f"point overlaps {check_all_overlap(center_list, cand_pt, radius)}")

            if check_point_inregion(cand_pt, boundary_mat) and not check_all_overlap(center_list, cand_pt, radius):
                #print(center_list)
                center_list.append(cand_pt)
                queue.append(len(center_list) - 1)
                break
            if i == div_num - 1:
                queue.pop()
        
    #print(center_list)
    
    return np.array(center_list)

def check_point_inregion(Point, vertex_mat):
    """M = len(vertex_mat) - 1
    x0, y0 = Point
    total_sum = 0
    
    for m in range(M):
        xqm, yqm = vertex_mat[m]
        xqm1, yqm1 = vertex_mat[m + 1]
        dot_product = (xqm - x0) * (xqm1 - x0) + (yqm - y0) * (yqm1 - y0)
        norm_product = np.sqrt((xqm - x0)**2 + (yqm - y0)**2) * np.sqrt((xqm1 - x0)**2 + (yqm1 - y0)**2)
        
        # Use arccos safely, prevent domain errors due to floating point arithmetic
        cosine_angle = np.clip(dot_product / norm_product, -1, 1)
        subres = np.arccos(cosine_angle)
        
        # Calculate the sign for the angle sum
        sign_factor = np.sign((xqm - x0) * (yqm1 - y0) - (xqm1 - x0) * (yqm - y0))
        
        total_sum += subres * sign_factor

    # Check if the total sum of angles is approximately 2*pi
    if np.isclose(total_sum, 2 * np.pi, atol=1e-7):
        return 1
    else:
        return 0"""
    
    path = Path(vertex_mat)
    
    # Use the contains_point method to check if the point is inside the polygon
    return 1 if path.contains_point(Point) else 0
    
def check_all_overlap(center_list, circ_cand, radius):
    for center in center_list:
        if check_overlap_circle(center, circ_cand, radius):
            return 1
    return 0


import numpy as np
import math
import random

def calculate_total_distance(points, order):
    total_distance = 0
    for i in range(len(order)):
        total_distance += np.linalg.norm(np.array(points[order[i - 1]]) - np.array(points[order[i]]))
    return total_distance

def swap_two(order):
    idx1, idx2 = random.sample(range(1, len(order)), 2)  # Avoid swapping the first city
    new_order = order[:]
    new_order[idx1], new_order[idx2] = new_order[idx2], new_order[idx1]
    return new_order

def simulated_annealing(points, temperature=10000, cooling_rate=0.003):
    current_order = list(range(len(points)))
    current_distance = calculate_total_distance(points, current_order)
    best_order = current_order[:]
    best_distance = current_distance

    while temperature > 1:
        new_order = swap_two(current_order)
        new_distance = calculate_total_distance(points, new_order)

        if new_distance < best_distance:
            best_order = new_order[:]
            best_distance = new_distance

        # Acceptance probability to decide if we should move to the solution or stay with the current one
        acceptance_probability = math.exp((current_distance - new_distance) / temperature)
        if new_distance < current_distance or random.random() < acceptance_probability:
            current_order = new_order[:]
            current_distance = new_distance

        # Cool down the temperature
        temperature *= 1 - cooling_rate

    return best_order, best_distance

def plot_best_path(points, best_order):
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
