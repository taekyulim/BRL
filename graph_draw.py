import xml.etree.ElementTree as ET
import networkx as nx
import matplotlib.pyplot as plt

# Read and parse the nodes XML
with open("test2.nod.xml", "r") as file:
    nodes_data = file.read()

nodes_root = ET.fromstring(nodes_data)
nodes = {}
for node in nodes_root.findall('node'):
    id = node.attrib['id']
    x = float(node.attrib['x'])
    y = float(node.attrib['y'])
    nodes[id] = (x, y)

# Read and parse the edges XML
with open("test2.edg.xml", "r") as file:
    edges_data = file.read()

edges_root = ET.fromstring(edges_data)
edges = []
edge_lengths = {}
for edge in edges_root.findall('edge'):
    from_node = edge.attrib['from']
    to_node = edge.attrib['to']
    length = float(edge.attrib['length'])
    edge_id = (from_node, to_node)
    edges.append(edge_id)
    edge_lengths[edge_id] = length

# Create the graph
G = nx.DiGraph()
G.add_nodes_from(nodes.keys())
G.add_edges_from(edges)

# Draw the graph
fig, ax = plt.subplots(figsize=(20, 20))
pos = {node: (coords[0], coords[1]) for node, coords in nodes.items()}
nx.draw(G, pos, with_labels=True, node_size=1000, node_color='skyblue', edgecolors='black', font_size=10, font_weight='bold', arrows=True)

# Add edge labels
edge_labels = {(u, v): f"{edge_lengths[(u, v)]:.2f}" for u, v in G.edges()}
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=8)

# Invert y-axis

plt.show()