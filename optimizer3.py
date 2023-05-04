from BRL import *
from ortools.sat.python import cp_model
import numpy as np

def get_eT():
    return np.random.randint(low=4000, high=5550, size=10) # 랜덤 변수 실행

# e_T = get_eT()
e_T = np.random.randint(low=100, high=450, size=10)

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

def constraint_making(indices_list, directions, x, model, w1=1000, w2=607, w3=51, w4=1000, w5=456, w6=8, w7=1000, w8=960, w9=66):

    total_x = sum(x[i] for group in indices_list for i in group)
    straight_sum = sum(x[j] for i, group in enumerate(indices_list) for j in group if directions[i] == "직진")
    left_sum = sum(x[j] for i, group in enumerate(indices_list) for j in group if directions[i] == "좌회전")
    right_sum = sum(x[j] for i, group in enumerate(indices_list) for j in group if directions[i] == "우회전")

    if len(directions) == 3:
        pass
        # model.Add(10*straight_sum - 5*total_x >= 0)
        # model.Add(10*straight_sum - 7*total_x <= 0)
        # model.Add(10*left_sum + 10*right_sum - 5*total_x <= 0)
        # model.Add(100*left_sum - 5*total_x >= 0)
        # model.Add(100*right_sum - 5*total_x >= 0)
        # model.Add(left_sum - right_sum >= 0)
    elif '직진' in directions:
        if '우회전' in directions:
            model.Add(w1*straight_sum - w2*total_x <= 0)
            model.Add(w1*right_sum - w3*total_x >= 0)
        else: # 좌회전 케이스
            model.Add(w4*straight_sum - w5*total_x <= 0)
            model.Add(w4*left_sum - w6*total_x >= 0)
    else: # 죄회전, 우회전으로 길 갈림
        model.Add(w7*left_sum - w8*total_x <= 0)
        model.Add(w7*right_sum - w9*total_x >= 0)

    return model

input_volume = [1549, 2560, 1157, 2065, 1801, 1514, 1842, 1211]
output_volume = [1597, 998, 1157, 2078, 1427, 2210, 1730, 1671]

def solve(P=p_matrix, I=I_matrix, O=O_matrix,keys=crossroad_var_keys, e_T=e_T, w1=1000, w2=70, w3=5, w4=1000, w5=70, w6=5, w7=1000, w8=70, w9=5, input_volume=input_volume, output_volume=output_volume,lambda1 = 10, lambda2 = 10, max_time=10):
    model = cp_model.CpModel()
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = max_time

    num_x = P.shape[1]
    num_e = P.shape[0]
    etrue = [e_T[i] for i in range(num_e)] # numpy 자료형 에서 파이썬 형태로 바꿈.

    x = [model.NewIntVar(0, 10000, f'x_{i}') for i in range(num_x)] # 최단경로 활당용 변수
    error = [model.NewIntVar(-100000, 100000, f"error_{i}") for i in range(num_e)]
    squared_error = [model.NewIntVar(0, 10000000, f"squared_error_{i}") for i in range(num_e)]

    for i in range(num_e):
        model.Add(error[i] == sum([P[i, j]*x[j] for j in range(num_x)]) - etrue[i])
        model.AddMultiplicationEquality(squared_error[i], [error[i], error[i]])
    
    num_iv = len(input_volume)
    input_volume_error = [model.NewIntVar(-100000, 100000, f"input_volume_error_{i}") for i in range(num_iv)]
    squared_input_volume_error = [model.NewIntVar(0, 10000000, f"squared_input_volume_error_{i}") for i in range(num_iv)]
    
    output_volume_error = [model.NewIntVar(-100000, 100000, f"output_volume_error_{i}") for i in range(num_iv)]
    squared_output_volume_error = [model.NewIntVar(0, 10000000, f"squared_output_volume_error_{i}") for i in range(num_iv)]

    for i in range(num_iv):
        calculated_input_volume = sum([I[i, j] * x[j] for j in range(num_x)])
        calculated_output_volume = sum([O[i, j]*x[j] for j in range(num_x)])
        model.Add(input_volume_error[i] == input_volume[i] - calculated_input_volume)
        model.Add(output_volume_error[i] == output_volume[i] - calculated_output_volume)
        model.AddMultiplicationEquality(squared_input_volume_error[i], [input_volume_error[i], input_volume_error[i]])
        model.AddMultiplicationEquality(squared_output_volume_error[i], [output_volume_error[i], output_volume_error[i]])
  
    sum_squared_error = model.NewIntVar(0, 10000000000, 'sum_squared_error')
    model.Add(sum_squared_error == lambda1*sum(squared_error[i] for i in range(num_e)) + lambda2*sum(squared_input_volume_error[i] for i in range(num_iv)) + lambda2*sum(squared_output_volume_error[i] for i in range(num_iv)))
    model.Minimize(sum_squared_error)


    for key in keys:
        aa = crossroad_var[key]
        if len(aa) != 1: # 하나일땐 전부 다 들어가므로 패스
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
                # directions : ['직진', '우회전', '좌회전'] 형태
                model = constraint_making(indices_list = indices_list, directions= directions, x=x, model=model, w1=int(w1), w2=int(w2), \
                    w3=int(w3), w4=int(w4), w5=int(w5), w6=int(w6), w7=int(w7), w8=int(w8), w9=int(w9))

    status = solver.Solve(model)
    
    result = {}
    for i in range(num_x):
        if solver.Value(x[i]) != 0:
            result[i] = solver.Value(x[i])

    return result