import numpy as np
from BRL import *

def get_eT():
    return np.random.randint(low=4000, high=5550, size=10) # 랜덤 변수 실행

e_T = get_eT()

def objective_function(x):
    e = np.dot(p_matrix, x)
    return np.sum(np.abs(e_T-e))

def constraint_function(x, constraint_value):
    return constraint_value - np.sum(x)

def get_traffic_volume(route_number, objective_volume):
    bounds = [(0, None) for _ in range(p_matrix.shape[1])]
    constraints = [{'type' : 'ineq', 'fun' : lambda x: constraint_function(x, objective_volume)}]
    obj_func = objective_function_wrapper(p_matrix)
    
    x0 = np.zeros(p_matrix.shape[1]) # 초기값
    result = minimize(obj_func, x0, constraints=constraints, bounds=bounds)
    
    return list(np.array(result.x).astype(int))


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