import svgwrite, csv
from lxml import etree
from time import time
import heapq
import collections
import haversine
from collections import namedtuple
from math import sqrt


Edge = namedtuple('Edge', ['start_node', 'end_node', 'weight'])


def coordinate_input(latRange=(-90, 90), lonRange=(-180, 180)):
    minLat, maxLat = latRange
    minLon, maxLon = lonRange
    while True:
        lat = float(input('Enter latitude (%s < lat < %s): ' % (minLat, maxLat)))
        lon = float(input('Enter longitude (%s < lon < %s): ' % (minLon, maxLon)))
        if (lat < minLat or lat > maxLat or lon < minLon or lon > maxLon):
            print('Wrong input!\nTry again', end='\n\n')
            continue
        else:
            return lat, lon



def Euclid(a, b):
    return sqrt((coordMap[a][0] - coordMap[b][0])**2 + (coordMap[a][1] - coordMap[b][1])**2)


def Manhattan(a, b):
    return abs(coordMap[a][0] - coordMap[b][0]) + abs(coordMap[a][1] - coordMap[b][1])


def Chebyshev(a, b):
    return max(abs(coordMap[a][0] - coordMap[b][0]), abs(coordMap[a][1] - coordMap[b][1]))


def Astar_method(start, goal, heuristic=Euclid):
    priority_queue = []
    heapq.heappush(priority_queue, (0, start))

    cameFrom = {}
    cameFrom[start] = None
    cameFrom[goal] = None

    weight = collections.defaultdict(lambda: float('inf'))
    weight[start] = 0

    while priority_queue:
        cur = heapq.heappop(priority_queue)[1] # [(prior1, id1) , (prior2, id2)]

        if cur == goal:
            break

        for neighbour in resMap[cur]:
            new_weight = weight[cur] + haversine.haversine((coordMap[cur][0], coordMap[cur][1]),
                                                           (coordMap[neighbour][0], coordMap[neighbour][1]))
            if new_weight < weight[neighbour]:
                weight[neighbour] = new_weight
                priority = new_weight + heuristic(neighbour, goal)
                heapq.heappush(priority_queue, (priority, neighbour))
                cameFrom[neighbour] = cur

    if not cameFrom[goal]:
        return float('inf'), []

    cur = goal
    route = [cur]
    while cur != start:
        cur = cameFrom[cur]
        route.append(cur)
    route.reverse()

    return weight[goal], route

def closeNode(goal):
    minNode, attrs = resMap.popitem()
    minDist =  haversine.haversine((coordMap[minNode][0], coordMap[minNode][1]),
                                                           (coordMap[goal][0], coordMap[goal][1]))
    resMap[minNode] = attrs

    for node in resMap:
        distance =  haversine.haversine((coordMap[node][0], coordMap[node][1]),
                                                           (coordMap[goal][0], coordMap[goal][1]))
        if distance < minDist:
            minDist = distance
            minNode = node

    return minNode


def generate_tsp_distance_matrix(coordMap):
    edge_list = []
    path_matrix = {}

    for start in coordMap:
        path_matrix[start] = {}
        for end in coordMap:
            if start != end:
                distance, path = Astar_method(start, end)
                edge_list.append(Edge(start, end, distance))
                path_matrix[start][end] = distance, path

    return edge_list, path_matrix


def get_closest(hospital, map_hospitals, visited):
    best_distance = float('inf')

    for h in map_hospitals:
        if h not in visited:
            distance, _path = map_hospitals[hospital][h]
            if distance < best_distance:
                closest_hospital = h
                best_distance = distance

    print(closest_hospital)
    return closest_hospital, best_distance


def tsp_nearest_neighbour(map_hospitals, hospitals):
    order = []
    order.append(hospitals[0])

    length = 0

    next_, dist = get_closest(order[0], map_hospitals, order)
    length += dist
    order.append(next_)

    while len(order) < len(map_hospitals):
        next_, dist = get_closest(next_, map_hospitals, order)
        length += dist
        order.append(next_)

    order.append(order[0])
    distance, _path = map_hospitals[order[-2]][order[-1]]
    length += distance

    return order, length

def min_spanning_tree(root, hospitals, edges):
    result = []
    edges.sort(key=lambda edge: edge.weight)

    selected_nodes = [root]
    while len(result) != len(hospitals):
        for edge in edges:
            if edge.start_node in selected_nodes and edge.end_node not in selected_nodes:
                result.append(edge)
                selected_nodes.append(edge.end_node)
                break

    return result

def tsp_double_min_spanning_tree(root, hospitals, path_matrix, edges):
    path = [root]
    length = 0
    double_span_tree = []
    for edge in min_spanning_tree(root, hospitals, edges):
        double_span_tree.append(edge)
        reverse_edge = Edge(edge.end_node, edge.start_node, edge.weight)
        double_span_tree.append(reverse_edge)

    stack = [root]
    while stack:
        cur_node = stack[-1]
        for edge in double_span_tree:
            if edge.start_node == cur_node:
                stack.append(edge.end_node)
                double_span_tree.remove(edge)
                break
        if cur_node == stack[-1]:
            node = stack.pop()
            if node not in path:
                path.append(node)

    path.append(path[0])
    i = 0
    while i < len(path) - 1:
        distance, _path = path_matrix[path[i]][path[i + 1]]
        length += distance
        i += 1

    return path, length


main_time = time()

minlat = 90.0
minlong = 180.0
maxlat = -90.0
maxlong = -180.0

graph = etree.parse("map.osm")

resMap = {}
coordMap = {}
hospitals = []

for node in graph.iterfind('/node'):
    coordMap[node.get('id')] = [float(node.get('lat')),
                                       float(node.get('lon'))]

for node in graph.iterfind('/node'):
    for second_node in node.iterfind('tag[@v="hospital"]'):
        # hospitals[node.get('id')] = [float(node.get('lat')),
        #                                float(node.get('lon'))]
        hospitals.append(node.get('id'))
hospitals = hospitals[:10]

for first_node in graph.iterfind('/way/tag[@k="highway"]'):
    if first_node.get("v") in ["motorway", "trunk", "primary", "secondary",
                               "tertiary", "unclassified", "residential",
                               "living_street"]:
        for second_node in first_node.iterfind('../tag'):
            buf_node = None

            for third_node in first_node.iterfind('../nd'):
                if third_node.get('ref') in coordMap:
                    minlat = min(coordMap[third_node.get('ref')][0], minlat)
                    minlong = min(coordMap[third_node.get('ref')][1], minlong)
                    maxlat = max(coordMap[third_node.get('ref')][0], maxlat)
                    maxlong = max(coordMap[third_node.get('ref')][1], maxlong)
                    if buf_node is None:
                        buf_node = third_node
                    else:
                        if second_node.get('k') == 'oneway' and second_node.get('v') == 'yes':
                            if buf_node.get('ref') not in resMap:
                                resMap[buf_node.get('ref')] = set()
                            resMap[buf_node.get('ref')].add(third_node.get('ref'))
                            buf_node = third_node
                        else:
                            if buf_node.get('ref') not in resMap:
                                resMap[buf_node.get('ref')] = set()
                            if third_node.get('ref') not in resMap:
                                resMap[third_node.get('ref')] = set()
                            resMap[buf_node.get('ref')].add(third_node.get('ref'))
                            resMap[third_node.get('ref')].add(buf_node.get('ref'))
                            buf_node = third_node


scale = 2000.0





scaleLat = (maxlat - minlat) / scale
scaleLong = (maxlong - minlong) / scale


# lat, lon = coordinate_input((minlat, maxlat), (minlong, maxlong))
lat, lon = 44.9037, 34.0725

origin = 0
coordMap[origin] = [lat, lon]
closest_node = closeNode(origin)
resMap[origin] = set()
resMap[origin].add(closest_node)
resMap[closest_node].add(origin)

for hospital in hospitals:
    closest_node = closeNode(hospital)
    resMap[hospital] = set()
    resMap[hospital].add(closest_node)
    resMap[closest_node].add(hospital)


tsp_edge_list, tsp_path_matrix = generate_tsp_distance_matrix([origin] + hospitals)

cycle, length = tsp_nearest_neighbour(tsp_path_matrix, [origin] + hospitals)

print('NNA distance: ', length)

routes = []
i = 0
while i < len(cycle) - 1:
    _distance, path = tsp_path_matrix[cycle[i]][cycle[i + 1]]
    routes.append(path)
    i += 1

resGraph = svgwrite.Drawing("nearest_neighbour_graph.svg", size=(str(scale) + 'px', str(scale) + 'px'))

for first_point in resMap:
    for second_point in resMap[first_point]:
        resGraph.add(resGraph.line((scale - (maxlong - coordMap[first_point][1]) / scaleLong,
             (maxlat - coordMap[first_point][0]) / scaleLat),
            (scale - (maxlong - coordMap[second_point][1])/ scaleLong,
             (maxlat - coordMap[second_point][0])/ scaleLat), stroke="black"))

for route in routes:
    i = 0
    while i < len(route) - 1:
        resGraph.add(resGraph.line((scale - (maxlong - coordMap[route[i]][1]) / scaleLong,
             (maxlat - coordMap[route[i]][0]) / scaleLat),
            (scale - (maxlong - coordMap[route[i + 1]][1])/ scaleLong,
             (maxlat - coordMap[route[i + 1]][0])/ scaleLat), stroke="purple", stroke_width=5))
        i += 1

g = resGraph.g(style="font-size:25;stroke:green")

for i, point in enumerate(cycle[:-1], start=1):
    resGraph.add(resGraph.circle((scale - (maxlong - coordMap[point][1]) / scaleLong,
         (maxlat - coordMap[point][0]) / scaleLat), 10, fill="purple"))
    g.add(resGraph.text(i, insert=(scale - (maxlong - coordMap[point][1]) / scaleLong,
         (maxlat - coordMap[point][0]) / scaleLat), ))
    resGraph.add(g)

resGraph.save()

with open('nearest_neighbour.csv', 'w') as csvfile:
    writer = csv.writer(csvfile)
    for route in routes:
        writer.writerow(route)

cycle, length = tsp_double_min_spanning_tree(origin, hospitals, tsp_path_matrix, tsp_edge_list)

print('Double Min Span Tree distance: ', length)

routes = []
i = 0
while i < len(cycle) - 1:
    _distance, path = tsp_path_matrix[cycle[i]][cycle[i + 1]]
    routes.append(path)
    i += 1

resGraph = svgwrite.Drawing("double_min_span_tree_graph.svg", size=(str(scale) + 'px', str(scale) + 'px'))

for first_point in resMap:
    for second_point in resMap[first_point]:
        resGraph.add(resGraph.line((scale - (maxlong - coordMap[first_point][1]) / scaleLong,
             (maxlat - coordMap[first_point][0]) / scaleLat),
            (scale - (maxlong - coordMap[second_point][1])/ scaleLong,
             (maxlat - coordMap[second_point][0])/ scaleLat), stroke="black"))

for route in routes:
    i = 0
    while i < len(route) - 1:
        resGraph.add(resGraph.line((scale - (maxlong - coordMap[route[i]][1]) / scaleLong,
             (maxlat - coordMap[route[i]][0]) / scaleLat),
            (scale - (maxlong - coordMap[route[i + 1]][1])/ scaleLong,
             (maxlat - coordMap[route[i + 1]][0])/ scaleLat), stroke="purple", stroke_width=5))
        i += 1

g = resGraph.g(style="font-size:25;stroke:green")

for i, point in enumerate(cycle[:-1], start=1):
    resGraph.add(resGraph.circle((scale - (maxlong - coordMap[point][1]) / scaleLong,
         (maxlat - coordMap[point][0]) / scaleLat), 10, fill="purple"))
    g.add(resGraph.text(i, insert=(scale - (maxlong - coordMap[point][1]) / scaleLong,
         (maxlat - coordMap[point][0]) / scaleLat), ))
    resGraph.add(g)

resGraph.save()

with open('double_min_span_tree_tsp.csv', 'w') as csvfile:
    writer = csv.writer(csvfile)
    for route in routes:
        writer.writerow(route)


print('Totaltime:', time() - main_time)

