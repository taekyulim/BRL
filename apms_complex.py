import numpy as np
import random
import pickle
import math
import copy
from collections import deque
from xml.etree.ElementTree import Element, parse, ElementTree

# ========전처리 부분========

## 도로에 대한 그래프 얻음
edge_file_path = "incheon.edg.xml"
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
node_file_path = "incheon.nod.xml"
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

def get_adjacent_edges(file_path):
    tree = parse(file_path)
    root = tree.getroot()
    edges = root.findall('edge')

    adj_list = {}
    edge_dict = {}

    for edge in edges:
        from_node = edge.get("from")
        to_node = edge.get("to")

        if from_node not in edge_dict:
            edge_dict[from_node] = []
        edge_dict[from_node].append(edge)

    for edge in edges:
        edge_id = edge.get("id")
        to_node = edge.get("to")
        if edge_id.startswith("r") is False:
            if to_node in edge_dict:
                adj_list[edge_id] = [adj_edge.get("id") for adj_edge in edge_dict[to_node] if adj_edge.get("from") != edge.get("to") or adj_edge.get("to") != edge.get("from")]

    return adj_list

# ----- 모델링에 사용할 전역 변수 입력 ----- #
graph = extract_graph_from_xml_file(edge_file_path)
position = extract_positions_from_xml_file(node_file_path)
edges = extract_edge_from_xml_file(edge_file_path)
reversed_edges = {tuple(value) : key for key, value in edges.items()}
# lengths = extract_road_lengths(edge_file_path)
crossroad = get_adjacent_edges(edge_file_path)

origin_nodes = ["R109", "R106", "R104", "R99", "R91", "R81", "R48", "R39", "R38", "R37", "R67", "R69", "R62", "R28", "R6", "R23", "R15"]
destination_nodes = ["R109", "R106", "R104", "R99", "R91", "R81", "R48", "R39", "R38", "R37", "R67", "R69", "R62", "R28", "R6", "R23", "R15", "R49", "R40", "R14", "R21", "R72", "R66"]

main_edges = [
    "r0_1",
    "r1_0",
    "r1_2",
    "r2_1",
    "r2_3",
    "r3_2",
    "r3_4",
    "r4_3",
    "r4_5",
    "r5_4",
    "r5_6",
    "r6_5",
    "r6_7",
    "r7_6",
    "r7_8",
    "r8_7",
    "r8_9",
    "r9_8",
    "r9_10",
    "r10_9",
    "r10_11",
    "r11_10",
    "r11_12",
    "r12_11",
    "r3_12",
    "r12_3"
]

crossroad_keys = list(crossroad.keys())

node_index_origin = {node: index for index, node in enumerate(origin_nodes)}
node_index_destination = {node: index for index, node in enumerate(destination_nodes)}

def get_crossroad_var(keys=crossroad_keys):
    crossroad_var = []
    for i in keys:
        if crossroad[i]:
            for j in crossroad[i]:
                crossroad_var.append([i ,j])
        else:
            pass
    groups = {}  # 첫번째 원소를 기준으로 그룹을 만들기 위한 딕셔너리
    for pair in crossroad_var:  # pairs는 주어진 이중 리스트
        key = pair[0]  # 첫번째 원소를 기준으로 그룹을 만들어야 하므로, key 변수에 첫번째 원소를 저장
        if key not in groups:  # key가 딕셔너리의 키에 없다면 새로운 리스트를 value로 추가
            groups[key] = []
        if key != pair:
            groups[key].append(pair) 
    return groups

crossroad_var = get_crossroad_var()
crossroad_var_keys = list(crossroad_var.keys())

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
    # shortest_length = 0
    for i in range(len(input_list)-1):
        temp = reversed_edges.get(tuple(input_list[i:i+2]))
        shortest_path.append(temp)
        # shortest_length += lengths[temp]
    
    return shortest_path

def return_all_routes(start_node, graph=graph):
    # start_node를 받으면 그 start node를 origin node로 하는 모든 최단경로 만듬
    result = []
    temp = copy.deepcopy(destination_nodes)
    temp.remove(start_node)
    for i in temp:
        routes = bfs_shortest_paths(start=start_node, end=i)
        for j in routes:
            shortest_path = get_shortest_edge_distance(j)
            result.append(shortest_path)
    return result

def get_all_shortest_paths():
    """모든 최단 경로들을 반환하는 함수. 
    이때 메인 도로 'r{number}_{number}_{number}' 를 포함하지 않은 최단경로는 제거 하였음.
    그 결과 66개의 최단 경로가 나옴.
    """
    all_shortest_paths = []
    for start_node in origin_nodes:
        temps = return_all_routes(start_node)
        for temp_route in temps:
            all_shortest_paths.append(temp_route)
    # filtered_list = [sublist for sublist in all_shortest_paths if any(elem.startswith('r') for elem in sublist)]
    return all_shortest_paths

all_shortest_paths = get_all_shortest_paths() # 모든 최단 경로 출력

def get_I_matrix():
    I_matrix = [[0] * len(all_shortest_paths) for _ in range(len(origin_nodes))]
    for j in range(len(all_shortest_paths)):
        first_node = edges[all_shortest_paths[j][0]][0]
        I_matrix[node_index_origin[first_node]][j] = 1
    return np.array(I_matrix)

I_matrix = get_I_matrix()

def get_O_matrix():
    O_matrix = [[0] * len(all_shortest_paths) for _ in range(len(destination_nodes))]
    for j in range(len(all_shortest_paths)):
        last_node = edges[all_shortest_paths[j][-1]][1]
        O_matrix[node_index_destination[last_node]][j] = 1
    return np.array(O_matrix)

O_matrix = get_O_matrix()

def get_p_matrix():
    # 최적화에 쓰일 p matrix 만듬
    # p matrix를 활용하여 66개의 최단경로에 할당되는 교통량을 활용하여
    # main edge의 교통량을 계산.
    p_matrix = [[0] * len(all_shortest_paths) for _ in range(26)]
    for j in range(len(all_shortest_paths)):
        shortest_paths = all_shortest_paths[j]
        for i in range(len(main_edges)):
            target_edge = main_edges[i]
            if target_edge in shortest_paths:
                p_matrix[i][j] = 1
    return np.array(p_matrix)

p_matrix = get_p_matrix()

def return_results(x):
    result = {}
    for i in range(len(x)):
        if x[i] != 0:
            path = all_shortest_paths[i]
            origin_edge, destination_edge = edges[path[0]][0], edges[path[-1]][-1]
            if (origin_edge, destination_edge) not in result:
                result[(origin_edge, destination_edge)] = 0
            result[(origin_edge, destination_edge)] += x[i]
    return result

def return_ratio(result):
    grouped_dict = {}

    # 딕셔너리의 키-값 쌍에 대해 반복
    for key, value in result.items():
        first_element = key[0]
        
        if first_element not in grouped_dict:
            grouped_dict[first_element] = []
        
        grouped_dict[first_element].append((key, value))

    # 비율을 저장할 딕셔너리
    ratios = {}

    # 각 그룹에 대해 반복
    for first_element, group in grouped_dict.items():
        total_value = sum(value for _, value in group)
        
        # 그룹 내의 키-값 쌍에 대해 반복하며 비율 계산
        for key, value in group:
            ratio = value / total_value
            ratios[key] = round(100*ratio, 2)
    
    updated_ratios = {}

    for key, value in ratios.items():
        first_key, second_key = key
        new_key = (first_key, second_key)
        updated_ratios[new_key] = value
    
    return updated_ratios


def get_ratio_matrix(x):
    ratio_matrix = np.zeros((23,17))
    result = return_results(x)
    ratios = return_ratio(result)
    for index in ratios:
        ratio_matrix[node_index_destination[index[1]], node_index_origin[index[0]]] = ratios[index]
    return ratio_matrix    
    