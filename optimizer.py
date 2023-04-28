import numpy as np
from BRL import *
from ortools.linear_solver import pywraplp

def get_eT():
    return np.random.randint(low=4000, high=5550, size=10) # 랜덤 변수 실행

# e_T = get_eT()
e_T = np.array([4494, 4069, 4227, 4752, 4338, 4844, 4939, 4017, 5137, 4836])

# def objective_function(x):
#     e = np.dot(p_matrix, x)
#     return np.sum(np.abs(e_T-e))

# def constraint_function(x, constraint_value):
#     return constraint_value - np.sum(x)

# def get_traffic_volume(route_number, objective_volume):
#     bounds = [(0, None) for _ in range(p_matrix.shape[1])]
#     constraints = [{'type' : 'ineq', 'fun' : lambda x: constraint_function(x, objective_volume)}]
#     obj_func = objective_function_wrapper(p_matrix)
    
#     x0 = np.zeros(p_matrix.shape[1]) # 초기값
#     result = minimize(obj_func, x0, constraints=constraints, bounds=bounds)
    
#     return list(np.array(result.x).astype(int))


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

def constraint_making(indices_list, is_end, x, solver):
    total_x = sum(x[i] for group in indices_list for i in group)
    # total_x = sum(x[i]/all_shortest_lengths[i] for group in indices_list for i in group)
    for i in range(len(indices_list)):
        if is_end[i]:# 끝나는 노드 경우
            solver.Add(sum(x[j] for j in indices_list[i]) - 0.1* total_x >= 0)
            # solver.Add(sum(x[j]/all_shortest_lengths[j] for j in indices_list[i]) - 0.1* total_x >= 0)
        else: # 계속 진행되는 경우
            solver.Add(sum(x[j] for j in indices_list[i]) - 0.6* total_x <= 0)
            # solver.Add(sum(x[j]/all_shortest_lengths[j] for j in indices_list[i]) - 0.6* total_x <= 0)
    return solver


# def making_solution(e_T=e_T, P=p_matrix):
#     solver = pywraplp.Solver.CreateSolver('GLOP')
#     x = [solver.NumVar(0, solver.infinity(), f'x_{j}') for j in range(72)]

#     e = [solver.NumVar(0, solver.infinity(), f'e_{i}') for i in range(10)]
#     objective = solver.Objective()
#     for i in range(10):
#         e_i = sum(P[i, j]*x[j] for j in range(72))
        
#         objective.SetCoefficient(e[i], 1)
#         solver.Add(e[i] == e_i - e_T[i])
#     objective.SetMinimization()
    
#     for key in keys:
#         aa = crossroad_var[key]
#         if len(aa) != 1:
#             indices_list = []
#             is_end = []
#             for a in aa:
#                 indices_list.append(get_intersection(a))
#                 if edges[a[1]][1] in destination_nodes:
#                     is_end.append(True)
#                 else:
#                     is_end.append(False)
#             if indices_list:
#                 constraint_making(indices_list = indices_list, is_end = is_end, x=x, solver=solver)
#     solver.Solve()
#     print(f"Objective value: {objective.Value()}")
#     print("Solution:")
#     for i in range(72):
#         print(f"x_{i} = {x[i].solution_value()}")
        
# crossroad_var_keys = list(crossroad_var.keys())

# class OptimizationProblem:
#     def __init__(self, e_T = e_T, P=p_matrix):
#         self.e_T = e_T
#         self.P = P
#         self.solver = pywraplp.Solver.CreateSolver('GLOP')
#         self.x = [self.solver.NumVar(0, self.solver.infinity(), f'x_{j}') for j in range(72)]
#         self.e = [self.solver.NumVar(0, self.solver.infinity(), f'e_{i}') for i in range(10)]
#         self.objective = self.solver.Objective()
        
#     def build_objective(self):
#         for i in range(10):
#             e_i = sum(self.P[i, j] * self.x[j] for j in range(72))
#             self.objective.SetCoefficient(self.e[i], 1)
#             self.solver.Add(self.e[i] == e_i - self.e_T[i])
#         self.objective.SetMinimization()
        
#     def constraint_making(self, indices_list, is_end):
#         for i in range(len(indices_list)):
#             if is_end[i]:  # Ending node case
#                 constraint = self.solver.Constraint(0.1, self.solver.infinity())
#                 for j in indices_list[i]:
#                     constraint.SetCoefficient(self.x[j], 1)
#             else:  # Continuing node case
#                 constraint = self.solver.Constraint(-self.solver.infinity(), 0.6)
#                 for k in indices_list[i]:
#                     constraint.SetCoefficient(self.x[k], 1)
    
#     def build_constraints(self,crossroad_var = crossroad_var, get_intersection = get_intersection, edges = edges, destination_nodes = destination_nodes, keys=crossroad_var_keys):
#         for key in keys:
#             aa = crossroad_var[key]
#             if len(aa) != 1:
#                 indices_list = []
#                 is_end = []
#                 for a in aa:
#                     indices_list.append(get_intersection(a))
#                     if edges[a[1]][1] in destination_nodes:
#                         is_end.append(True)
#                     else:
#                         is_end.append(False)
#                 if indices_list:
#                     self.constraint_making(indices_list=indices_list, is_end=is_end)

#     def solve(self):
#         self.solver.Solve()
        
#     def print_solution(self):
#         print(f"Objective value: {self.objective.Value()}")
#         print("Solution:")
#         for i in range(72):
#             print(f"x_{i} = {self.x[i].solution_value()}")


def solve(P=p_matrix, keys=crossroad_var_keys):
    solver = pywraplp.Solver.CreateSolver('GLOP')
    num_x = P.shape[1]
    num_e = P.shape[0]
    print(e_T)

    x = [solver.NumVar(0, solver.infinity(), f'x_{i}' ) for i in range(num_x)]

    e = [solver.NumVar(0, solver.infinity(), f'e_{i}' ) for i in range(num_e)]
    
    abs_diff = [solver.NumVar(0, solver.infinity(), f'abs_diff_{i}') for i in range(num_e)]
    
    objective = solver.Objective()
    for i in range(num_e):
        e_i = sum(P[i, j]*x[j] for j in range(num_x))
        
        solver.Add(abs_diff[i] >= e_T[i] - e[i])

        # Add constraint abs_diff[i] >= e[i] - e_T[i]
        solver.Add(abs_diff[i] >= e[i] - e_T[i])
        
        solver.Add(e[i] == e_i)
        
        objective.SetCoefficient(abs_diff[i], 1)
        

    objective.SetMinimization()
    
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
                solver = constraint_making(indices_list = indices_list, is_end = is_end, x=x, solver=solver)
    solver.Solve()
    print(f"Objective value: {objective.Value()}")
    result = {}
    for i in range(num_x):
        if x[i].solution_value() != 0:
            result[i] = int(x[i].solution_value())
    return result
    # print(f"Objective value: {objective.Value()}")
    # print("Solution:")
    # for i in range(num_x):
    #     print(f"x_{i} = {x[i].solution_value()}")
        