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


graph = extract_graph_from_xml_file(edge_file_path)
position = extract_positions_from_xml_file(node_file_path)
edges = extract_edge_from_xml_file(edge_file_path)
reversed_edges = {tuple(value) : key for key, value in edges.items()}
lengths = extract_road_lengths(edge_file_path)

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

origin_nodes = ["0_0", "0_2", "1_3", "2_2", "4_0", "4_1", "3_1", "3_2"]
destination_nodes = ["0_0", "0_2", "1_3", "2_2", "4_0", "4_1", "3_1", "3_2"]