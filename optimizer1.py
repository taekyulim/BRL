from BRL import *
import cvxpy as cp
## 첫번째 제약조건 케이스

def get_eT():
    return np.random.randint(low=4000, high=5550, size=10) # 랜덤 변수 실행

# e_T = get_eT()
e_T = np.array([4494, 4069, 4227, 4752, 4338, 4844, 4939, 4017, 5137, 4836])

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

def constraint_making(indices_list, is_end, x, alpha=1000, beta=100, gamma=600):
    constraints = []
    total_x = sum(x[i] for group in indices_list for i in group)

    for i in range(len(indices_list)):
        if is_end[i]:  # Ending node case
            constraints.append(cp.sum([x[j] for j in indices_list[i]]) - (beta/alpha) * total_x >= 0)
        else:  # Continuing node case
            constraints.append(cp.sum([x[j] for j in indices_list[i]]) - (gamma/alpha) * total_x <= 0)

    return constraints

def solve(P=p_matrix, keys=crossroad_var_keys, e_T=e_T,alpha=1000, beta=100, gamma=600):
    num_x = P.shape[1]
    num_e = P.shape[0]
    # print(e_T)

    x = cp.Variable(num_x, nonneg=True)
    e = cp.Variable(num_e, nonneg=True)

    squared_diff = cp.sum_squares(e_T - e)
    constraints = [e == P @ x]

    for key in keys:
        aa = crossroad_var[key]
        if len(aa) != 1:
            indices_list = []
            is_end = []
            for a in aa:
                indices_list.append(get_intersection(a))
                if edges[a[1]][1] in destination_nodes:
                    is_end.append(True)
                else:
                    is_end.append(False)
            if indices_list:
                constraints += constraint_making(indices_list=indices_list, is_end=is_end, x=x, alpha=alpha, beta=beta, gamma=gamma)

    objective = cp.Minimize(squared_diff)
    problem = cp.Problem(objective, constraints)
    problem.solve()

    # print(f"Objective value: {problem.value}")
    result = {}
    for i in range(num_x):
        if x.value[i] != 0:
            result[i] = int(x.value[i])
    return result
    # print(f"Objective value: {objective.Value()}")
    # print("Solution:")
    # for i in range(num_x):
    #     print(f"x_{i} = {x[i].solution_value()}")