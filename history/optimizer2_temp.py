from BRL import *
import cvxpy as cp
import numpy as np

def get_eT():
    return np.random.randint(low=4000, high=5550, size=10) # 랜덤 변수 실행

# e_T = get_eT()
e_T = np.random.randint(low=100, high=450, size=10)

node_index = {
    '0_0' : 0,
    '0_2' : 1,
    '1_3' : 2,
    '2_2' : 3,
    '3_2' : 4,
    '3_1' : 5,
    '4_1' : 6,
    '4_0' : 7
}

def get_I_matrix():
    I_matrix = [[0] * len(all_shortest_paths) for _ in range(8)]
    for j in range(len(all_shortest_paths)):
        first_node = edges[all_shortest_paths[j][0]][0]
        I_matrix[node_index[first_node]][j] = 1
    return np.array(I_matrix)

I_matix = get_I_matrix()

def get_O_matrix():
    O_matrix = [[0] * len(all_shortest_paths) for _ in range(8)]
    for j in range(len(all_shortest_paths)):
        last_node = edges[all_shortest_paths[j][-1]][1]
        O_matrix[node_index[last_node]][j] = 1
    return np.array(O_matrix)

O_matrix = get_O_matrix()

def is_sequenetial(smaller, larger):
    smaller_len = len(smaller)
    larger_len = len(larger)
    
    if smaller_len > larger_len:
        return False
    for i in range(larger_len - smaller_len + 1):
        if larger[i:i+smaller_len] == smaller:
            return True
    return False

def get_intersection(element):
    result = []
    from_edge = element[0]
    to_edge = element[1]
    aa = get_edge_include_routes(from_edge)
    bb = get_edge_include_routes(to_edge)
    set_aa= set(tuple(x) for x in aa)
    set_bb = set(tuple(x) for x in bb)
    intersections = [list(x) for x in set_aa.intersection(set_bb)]
    
    for intersection in intersections:
        if intersection in all_shortest_paths:
            result.append(all_shortest_paths.index(intersection))
    return result

def constraint_making(indices_list, directions, x):
    constraints = []
    total_x = sum(x[i] for group in indices_list for i in group)
    straight_sum = sum(x[j] for i, group in enumerate(indices_list) for j in group if directions[i] == "직진")
    left_sum = sum(x[j] for i, group in enumerate(indices_list) for j in group if directions[i] == "좌회전")
    right_sum = sum(x[j] for i, group in enumerate(indices_list) for j in group if directions[i] == "우회전")

    if len(directions) == 3:
        pass
        # constraints.append(10 * straight_sum - 5 * total_x >= 0)
        # constraints.append(10 * straight_sum - 7 * total_x <= 0)
        # constraints.append(10 * left_sum + 10 * right_sum - 5 * total_x <= 0)
        # constraints.append(100 * left_sum - 5 * total_x >= 0)
        # constraints.append(100 * right_sum - 5 * total_x >= 0)
        # constraints.append(left_sum - right_sum >= 0)
    elif '직진' in directions:
        if '우회전' in directions:
            constraints.append(100 * straight_sum - 70 * total_x <= 0)
            constraints.append(100 * right_sum - 5 * total_x >= 0)
        else:  # 좌회전 케이스
            constraints.append(100 * straight_sum - 70 * total_x <= 0)
            constraints.append(100 * left_sum - 5 * total_x >= 0)
    else:  # 좌회전, 우회전으로 길 갈림
        constraints.append(100 * left_sum - 70 * total_x <= 0)
        constraints.append(100 * right_sum - 5 * total_x >= 0)

    return constraints


def solve(P=p_matrix, keys=crossroad_var_keys, e_T=e_T):
    num_x = P.shape[1]
    num_e = P.shape[0]

    x = cp.Variable(num_x, nonneg=True)
    e = cp.Variable(num_e)
    squared_diff = cp.sum_squares(e_T - e)
    constraints = [e == P @ x]

    for key in keys:
        aa = crossroad_var[key]
        if len(aa) != 1:
            indices_list = []
            directions = []
            for a in aa:
                indices_list.append(get_intersection(a))
                first = position[edges[a[0]][0]]
                second = position[edges[a[0]][1]]
                third = position[edges[a[1]][1]]
                angle = angle_between_points(first, second, third)
                direction = direction_from_angle(angle)
                directions.append(direction)
            if indices_list:
                constraints += constraint_making(indices_list=indices_list, directions=directions, x=x)

    objective = cp.Minimize(squared_diff)
    problem = cp.Problem(objective, constraints)
    problem.solve(solver=cp.ECOS_BB)

    result = {}
    for i in range(num_x):
        if x.value[i] != 0:
            result[i] = int(x.value[i])

    return result, problem.status