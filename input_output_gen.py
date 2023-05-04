main_roads = {
    'r1_0_1': ('0_1', '1_0', ['0_2', '0_0'], ['1_3']),
    'r1_1_0': ('1_0', '0_1', ['1_3'], ['0_0', '0_2']),
    'r2_1_2': ('1_2', '2_0', ['1_3'], ['2_2']),
    'r2_2_1': ('2_0', '1_2', ['2_2'], ['1_3']),
    'r3_2_3': ('2_1', '3_3', ['2_2'], ['3_1', '3_2']),
    'r3_3_2': ('3_3', '2_1', ['3_1', '3_2'], ['2_2']),
    'r4_3_4': ('3_0', '4_2', ['3_1', '3_2'], ['4_0', '4_1']),
    'r4_4_3': ('4_2', '3_0', ['4_0', '4_1'], ['3_1', '3_2']),
    'r5_4_1': ('4_3', '1_1', ['4_0', '4_1'], ['1_3']),
    'r5_1_4': ('1_1', '4_3', ['1_3'], ['4_0', '4_1'])
}

def data_gen(traffic_volume):
    nodes = ['0_0', '0_2', '1_3', '2_2', '4_0', '4_1', '3_1', '3_2']
    traffic_count = {node: {'enter': 0, 'exit': 0} for node in nodes}

    # Initialize traffic count for each node
    nodes = ['0_0', '0_2', '1_3', '2_2', '4_0', '4_1', '3_1', '3_2']
    traffic_count = {node: {'enter': 0, 'exit': 0} for node in nodes}

    # Calculate traffic entering and exiting each node
    for road, volume in traffic_volume.items():
        origin_nodes, dest_nodes = main_roads[road][2], main_roads[road][3]

        # Filter out nodes not in the specified list
        origin_nodes = [node for node in origin_nodes if node in nodes]
        dest_nodes = [node for node in dest_nodes if node in nodes]

        for origin_node in origin_nodes:
            traffic_count[origin_node]['enter'] += volume / len(origin_nodes)

        for dest_node in dest_nodes:
            traffic_count[dest_node]['exit'] += volume / len(dest_nodes)

    # Round traffic count to the nearest integer
    for node in traffic_count:
        traffic_count[node]['enter'] = round(traffic_count[node]['enter'])
        traffic_count[node]['exit'] = round(traffic_count[node]['exit'])

    # Print traffic count for each node
    return traffic_count

# {'0_0': {'enter': 502, 'exit': 776},
#  '0_2': {'enter': 502, 'exit': 776},
#  '1_3': {'enter': 4187, 'exit': 3771},
#  '2_2': {'enter': 3593, 'exit': 2604},
#  '4_0': {'enter': 1289, 'exit': 1541},
#  '4_1': {'enter': 1289, 'exit': 1541},
#  '3_1': {'enter': 1526, 'exit': 1702},
#  '3_2': {'enter': 1526, 'exit': 1702}}