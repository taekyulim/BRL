import numpy as np
import random
import pickle
import math
import copy
from collections import deque
from xml.etree.ElementTree import Element, parse, ElementTree
from scipy.optimize import minimize

# ========전처리 부분========

## 도로에 대한 그래프 얻음
edge_file_path = "test2.edg.xml"
def extract_graph_from_xml_file(file_path):
    tree = parse(file_path)
    root = tree.getroot()
    graph = {}

    for edge in root.findall("edge"):
        from_node = edge.get("from")
        to_node = edge.get("to")

        if from_node not in graph:
            graph[from_node] = [to_node]
        else:
            graph[from_node].append(to_node)

    return graph

## 위치 좌표 얻음
node_file_path = "test2.nod.xml"
def extract_positions_from_xml_file(file_path):
    tree = parse(file_path)
    root = tree.getroot()
    position = {}

    for node in root.findall("node"):
        node_id = node.get("id")
        x = float(node.get("x"))
        y = float(node.get("y"))
        position[node_id] = [x, y]

    return position

def extract_edge_from_xml_file(file_path):
    tree= parse(file_path)
    root = tree.getroot()
    edges = {}
    
    for edge in root.findall("edge"):
        edge_id = edge.get("id")
        from_node = edge.get("from")
        to_node = edge.get("to")
        
        if edge_id not in edges:
            edges[edge_id] = [from_node, to_node]
    return edges

def extract_road_lengths(file_path):
    tree = parse(file_path)
    root = tree.getroot()
    lengths = {}
    
    for edge in root.findall("edge"):
        edge_id = edge.get("id")
        distance = float(edge.get("length"))
        
        if edge_id not in lengths:
            lengths[edge_id] = distance
    
    return lengths

# ----- 모델링에 사용할 전역 변수 입력 ----- #
graph = extract_graph_from_xml_file(edge_file_path)
position = extract_positions_from_xml_file(node_file_path)
edges = extract_edge_from_xml_file(edge_file_path)
reversed_edges = {tuple(value) : key for key, value in edges.items()}
lengths = extract_road_lengths(edge_file_path)

origin_nodes = ["0_0", "0_2", "1_3", "2_2", "4_0", "4_1", "3_1", "3_2"]
destination_nodes = ["0_0", "0_2", "1_3", "2_2", "4_0", "4_1", "3_1", "3_2"]
main_edges = ['r1_0_1', 'r1_1_0', 'r2_1_2', 'r2_2_1', 'r3_2_3', 'r3_3_2', 'r4_3_4', 'r4_4_3', 'r5_4_1', 'r5_1_4']

def bfs_shortest_paths(start, end, graph=graph):
    """
    graph와 시작 노드, 끝 노드가 있을때 모든 최단경로를 출력하는 함수
    """
    queue = deque()
    visited = set()
    dist = {start: 0}
    paths = {start: [[start]]}
    
    queue.append(start)
    visited.add(start)
    
    while queue:
        curr = queue.popleft()
        for neighbor in graph[curr]:
            if neighbor not in visited:
                visited.add(neighbor)
                dist[neighbor] = dist[curr] + 1
                paths[neighbor] = []
                for path in paths[curr]:
                    paths[neighbor].append(path + [neighbor])
                queue.append(neighbor)
            elif dist[neighbor] == dist[curr] + 1:
                for path in paths[curr]:
                    paths[neighbor].append(path + [neighbor])
    
    return paths[end]

def get_shortest_edge_distance(input_list):
    # 위에 bfs_shortest_paths로 구한 결과를 edge 결과로 바꿔줌.
    shortest_path = []
    shortest_length = 0
    for i in range(len(input_list)-1):
        temp = reversed_edges.get(tuple(input_list[i:i+2]))
        shortest_path.append(temp)
        shortest_length += lengths[temp]
    
    return shortest_path, shortest_length

def return_all_routes(start_node, graph=graph):
    # start_node를 받으면 그 start node를 origin node로 하는 모든 최단경로 만듬
    result = []
    temp = copy.deepcopy(origin_nodes)
    temp.remove(start_node)
    for i in temp:
        routes = bfs_shortest_paths(start=start_node, end=i)
        for j in routes:
            shortest_path, _ = get_shortest_edge_distance(j)
            result.append(shortest_path)
    return result

def get_all_shortest_paths():
    all_shortest_paths = []
    for start_node in origin_nodes:
        temps = return_all_routes(start_node)
        for temp in temps:
            if temp not in all_shortest_paths:
                all_shortest_paths.append(temp)
    filtered_list = [sublist for sublist in all_shortest_paths if any(elem.startswith('r') for elem in sublist)]
    return filtered_list

all_shortest_paths = get_all_shortest_paths() # 모든 최단 경로 출력

def filter_routes(target_edge, start_node):
    # 특정 start node를 origin node로 했을 때 target_edge가 포함된 최단경로만 나오도록 함.
    all_routes = return_all_routes(start_node)
    filtered_routes = []
    for route in all_routes:
        if target_edge in route:
            filtered_routes.append(route)
    return filtered_routes

def get_edge_include_routes(target_edge):
    # 우리가 조사할 target edge를 입력했을때 그 target edge를 포함하는 모든 경로들 나옴.
    results = []
    for start_node in origin_nodes:
        all_routes = filter_routes(target_edge, start_node)
        for j in all_routes:
            if j != None:
                results.append(j)
    return results

def get_p_matrix():
    # 최적화에 쓰일 p matrix 만듬
    # p matrix를 활용하여 66개의 최단경로에 할당되는 교통량을 활용하여
    # main edge의 교통량을 계산.
    p_matrix = [[0] * 66 for _ in range(10)]
    for j in range(len(all_shortest_paths)):
        shortest_paths = all_shortest_paths[j]
        for i in range(len(main_edges)):
            target_edge = main_edges[i]
            if target_edge in shortest_paths:
                p_matrix[i][j] = 1
    return np.array(p_matrix)

p_matrix = get_p_matrix()

def get_eT():
    return np.random.randint(low=4000, high=5550, size=10) # 랜덤 변수 실행

e_T = get_eT()

def objective_function(x):
    e = np.dot(p_matrix, x)
    return np.sum(np.abs(e_T-e))

def constraint_function(x, constraint_value):
    return constraint_value - np.sum(x)