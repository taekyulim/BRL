from xml.etree.ElementTree import Element, parse, ElementTree
from optimizer import *

result = solve()

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

def make_route_xml(result=result):
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