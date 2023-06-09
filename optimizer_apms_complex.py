from apms_complex import *
from ortools.sat.python import cp_model
import numpy as np

# # e_T = get_eT()
# e_T = np.random.randint(low=100, high=450, size=10)
# I_T = np.random.randint(low=100, high=450, size=8)
# O_T = np.random.randint(low=100, high=450, size=8)

def is_sequenetial(smaller, larger):
    smaller_len = len(smaller)
    larger_len = len(larger)
    
    if smaller_len > larger_len:
        return False
    for i in range(larger_len - smaller_len + 1):
        if larger[i:i+smaller_len] == smaller:
            return True
    return False

# def get_intersection(element):
#     result = []
#     from_edge = element[0]
#     to_edge = element[1]
#     aa = get_edge_include_routes(from_edge)
#     bb = get_edge_include_routes(to_edge)
#     set_aa= set(tuple(x) for x in aa)
#     set_bb = set(tuple(x) for x in bb)
#     intersections = [list(x) for x in set_aa.intersection(set_bb)]
    
#     for intersection in intersections:
#         if intersection in all_shortest_paths:
#             result.append(all_shortest_paths.index(intersection))
#     return result

def solve(I_T,O_T,e_T,P=p_matrix, I=I_matrix, O=O_matrix, max_time = 10):
    model = cp_model.CpModel()
    solver = cp_model.CpSolver()
    num_x = P.shape[1]
    num_e = P.shape[0]
    etrue = [e_T[i] for i in range(num_e)] # numpy 자료형 에서 파이썬 형태로 바꿈.
    
    solver.parameters.max_time_in_seconds = max_time
    x = [model.NewIntVar(0, 10000, f'x_{i}') for i in range(num_x)] # 최단경로 활당용 변6543wsf수
    error = [model.NewIntVar(-10000, 10000, f"error_{i}") for i in range(num_e)]
    squared_error = [model.NewIntVar(0, 10000000000, f"squared_error_{i}") for i in range(num_e)]
    
    
    num_iv = I_matrix.shape[0]
    input_volume = [model.NewIntVar(0, 1000000, f"input_volume_{i}") for i in range(num_iv)]
    input_error =  [model.NewIntVar(-10000, 10000, f"input_error_{i}") for i in range(num_iv)]
    squared_input_error = [model.NewIntVar(0, 10000000000, f"squared_input_error_{i}") for i in range(num_iv)]
    
    for i in range(num_iv):
        model.Add(input_volume[i] == sum([I[i, j]*x[j] for j in range(num_x)]))
        model.Add(input_error[i] == input_volume[i] - I_T[i])
        model.AddMultiplicationEquality(squared_input_error[i], [input_error[i], input_error[i]])
    
    num_ov = O_matrix.shape[0]
    output_volume = [model.NewIntVar(0, 1000000, f"output_volume_{i}") for i in range(num_ov)]
    output_error =  [model.NewIntVar(-10000, 10000, f"output_error_{i}") for i in range(num_ov)]
    squared_output_error = [model.NewIntVar(0, 10000000000, f"squared_output_error_{i}") for i in range(num_ov)]
    
    for i in range(num_ov):
        model.Add(output_volume[i] == sum([O[i, j]*x[j] for j in range(num_x)]))
        model.Add(output_error[i] == output_volume[i] - O_T[i])
        model.Add(100*output_error[i] < 10*O_T[i])
        model.Add(100*output_error[i] > -10*O_T[i])
        model.AddMultiplicationEquality(squared_output_error[i], [output_error[i], output_error[i]])
        
    for i in range(num_e):
        model.Add(error[i] == sum([P[i, j]*x[j] for j in range(num_x)]) - etrue[i])
        model.AddMultiplicationEquality(squared_error[i], [error[i], error[i]])
        
    sum_squared_error = model.NewIntVar(0, 10000000000, 'sum_squared_error')
    model.Add(sum_squared_error == sum(squared_error[i] for i in range(num_e)))
    
    sum_squared_input_error = model.NewIntVar(0, 10000000000, 'sum_squared_input_error')
    model.Add(sum_squared_input_error == sum(squared_input_error[i] for i in range(num_iv)))
    
    sum_squared_output_error = model.NewIntVar(0, 10000000000, 'sum_squared_output_error')
    model.Add(sum_squared_output_error == sum(squared_output_error[i] for i in range(num_iv)))

    regularization_lambda = 1
        
    # Calculate the L2 regularization term
    squared_x = [model.NewIntVar(0, 1000000000000, f"squared_x_{i}") for i in range(num_x)]
    for i in range(num_x):
        model.AddMultiplicationEquality(squared_x[i], [x[i], x[i]])
    sum_squared_x = model.NewIntVar(0, 10000000000000, 'sum_squared_x')
    model.Add(sum_squared_x == sum(squared_x[i] for i in range(num_x)))
    # # Modify objective function to include L2 regularization term
    objective = model.NewIntVar(0, 1000000000000000, 'objective')
    model.Add(objective == sum_squared_input_error + sum_squared_output_error+sum_squared_x)
    model.Minimize(objective)
    
    status = solver.Solve(model)
    
    result = {}
    for i in range(num_x):
        if solver.Value(x[i]) != 0:
            result[i] = solver.Value(x[i])
    
    return result, status