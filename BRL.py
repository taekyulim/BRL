import numpy as np
import random
import pickle
import math
import copy
from collections import deque
from xml.etree.ElementTree import Element, parse, ElementTree

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
lengths = extract_road_lengths(edge_file_path)
crossroad = get_adjacent_edges(edge_file_path)

origin_nodes = ["0_0", "0_2", "1_3", "2_2", "4_0", "4_1", "3_1", "3_2"]
destination_nodes = ["0_0", "0_2", "1_3", "2_2", "4_0", "4_1", "3_1", "3_2"]
main_edges = ['r1_0_1', 'r1_1_0', 'r2_1_2', 'r2_2_1', 'r3_2_3', 'r3_3_2', 'r4_3_4', 'r4_4_3', 'r5_4_1', 'r5_1_4']
crossroad_keys = list(crossroad.keys())

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

def return_all_lengths(start_node, graph=graph):
    # start_node를 받으면 그 start node를 origin node로 하는 모든 최단경로의 길이를 반환
    result = []
    temp = copy.deepcopy(origin_nodes)
    temp.remove(start_node)
    for i in temp:
        routes = bfs_shortest_paths(start = start_node, end=i)
        for j in routes:
            _, shortest_length = get_shortest_edge_distance(j)
            result.append(shortest_length)
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
    filtered_list = [sublist for sublist in all_shortest_paths if any(elem.startswith('r') for elem in sublist)]
    return filtered_list


all_shortest_paths = get_all_shortest_paths() # 모든 최단 경로 출력

def get_all_shortest_lengths():
    """
    모든 최단 경로의 길이들을 반환 하는 함수.
    인덱스를 바탕으로 경로와 길이에 접근 가능.
    딕셔너리로 안하는 이유는 메모리 문제.
    """
    all_shortest_lengths = []
    for start_node in origin_nodes:
        temps = return_all_lengths(start_node)
        for temp_length in temps:
            if temp_length > 500:
            # # 500 보다 큰 경우에는 subnode에 의해 생성되는 거리 필터링 가능.
            # # 이거는 추후 논의 해볼 것.
                all_shortest_lengths.append(temp_length)
    return all_shortest_lengths

all_shortest_lengths = get_all_shortest_lengths()


def get_I_matrix():
    I_matrix = [[0] * len(all_shortest_paths) for _ in range(8)]
    for j in range(len(all_shortest_paths)):
        first_node = edges[all_shortest_paths[j][0]][0]
        I_matrix[node_index[first_node]][j] = 1
    return np.array(I_matrix)

I_matrix = get_I_matrix()

def get_O_matrix():
    O_matrix = [[0] * len(all_shortest_paths) for _ in range(8)]
    for j in range(len(all_shortest_paths)):
        last_node = edges[all_shortest_paths[j][-1]][1]
        O_matrix[node_index[last_node]][j] = 1
    return np.array(O_matrix)

O_matrix = get_O_matrix()

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
    p_matrix = [[0] * len(all_shortest_paths) for _ in range(10)]
    for j in range(len(all_shortest_paths)):
        shortest_paths = all_shortest_paths[j]
        for i in range(len(main_edges)):
            target_edge = main_edges[i]
            if target_edge in shortest_paths:
                p_matrix[i][j] = 1
    return np.array(p_matrix)

p_matrix = get_p_matrix()

def angle_between_points(p1, p2, p3):
    """
    세 점이 있을때 각도를 구해주는 함수
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3

    angle1 = math.atan2(y2-y1, x2-x1)
    angle2 = math.atan2(y3-y2, x3-x2)

    angle = angle2 - angle1
    angle = math.degrees(angle)

    return angle%360

def direction_from_angle(angle, straight_threshold=30, left_threshold=150):
    """
    angle_between_points로 부터 나온 각도값을 활용하여 직진, 우회전, 죄회전 하는 지 판단하는 함수
    """
    if straight_threshold <= angle <= 180 - straight_threshold:
        return "좌회전"
    elif 180 + straight_threshold <= angle <= 360 - straight_threshold:
        return "우회전"
    else:
        return "직진"
    
    
def find_input_list_index(index, input_list):
    cumulative_sum = 0
    for i, item in enumerate(input_list):
        cumulative_sum += item
        if index < cumulative_sum:
            return i
    return -1  # not found

def get_edge_id_from_value(value, edges=edges):
    for key, val in edges.items():
        if val == value:
            return key
    return False

def sorting_vehicle(root):
    vtypes = root.findall('vType')
    routes = root.findall('route')
    vehicles = root.findall('vehicle')
    
    sorted_vehicles = sorted(vehicles, key=lambda v: int(v.get('depart')))
    
    new_routes = Element('routes')
    
    for vtype in vtypes:
        new_routes.append(vtype)
    
    for route in routes:
        new_routes.append(route)
        
    for vehicle in sorted_vehicles:
        new_routes.append(vehicle)
    
    return new_routes

def indent(elem, level=0):
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

def make_route_xml(result):
    """
    route_number : 모델링할 vertex 번호
    traffic_volume : 
    """
    root = Element("routes")
    vType_attrib = {
        "accel" : "1.0",
        "decel" : "5.0",
        "id" : "Car",
        "length" : "2.0",
        "maxSpeed" : "70.0",
        "sigma" : "0.0"
    }
    # result = optimizer.solve()
    vType = Element("vType", attrib=vType_attrib)
    root.append(vType)
    temp = 0
    routes = [all_shortest_paths[i] for i in list(result.keys())]
    volumes = list(result.values())
    for i in range(len(routes)):
        temp_edges = ' '.join(routes[i])
        route_attrib = {
            "id" : f"route{i}",
            "edges" : temp_edges
        }
        route_tag = Element("route", attrib=route_attrib)
        root.append(route_tag)
        # route 태그까지 추가 완료.
    indices = []
    index = 0
    for item in volumes:
        for _ in range(item):
            indices.append(index)
            index += 1
    for i in indices:
        depart = np.random.randint(0, 3600)
        route_index = find_input_list_index(i, volumes)
        vehicle_attrib = {
            "depart" : f"{depart}",
            "id" : f"veh{i}",
            "route" : f"route{route_index}",
            "type" : "Car"
        }
        vehicle = Element("vehicle", attrib=vehicle_attrib)
        root.append(vehicle)
    

    root = sorting_vehicle(root)
    indent(root)
    tree = ElementTree(root)
    file_name = "pangyo.rou.xml"
    tree.write(file_name, encoding="utf-8", xml_declaration=True)