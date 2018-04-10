import svgwrite, csv
from lxml import etree
from time import time
import heapq
import collections
import haversine
from queue import Queue
from math import sqrt
import random

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


def Dijkstra_method(start, goal):
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
                priority = new_weight
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

def Levit_method(start, goal):
    weight = collections.defaultdict(lambda: float('inf'))
    weight[start] = 0

    state = collections.defaultdict(lambda: 2)

    primary_queue = Queue()
    urgent_queue = Queue()

    primary_queue.put(start)
    state[start] = 1

    cameFrom = {}
    cameFrom[start] = None
    cameFrom[goal] = None

    while not primary_queue.empty() or not urgent_queue.empty():
        cur = primary_queue.get() if urgent_queue.empty() else urgent_queue.get()
        state[cur] = 0
        for neighbour in resMap[cur]:
            length = haversine.haversine((coordMap[cur][0], coordMap[cur][1]),
                                                           (coordMap[neighbour][0], coordMap[neighbour][1]))
            if state[neighbour] == 2:
                primary_queue.put(neighbour)
                state[neighbour] = 1
                if weight[cur] + length < weight[neighbour]:
                    cameFrom[neighbour] = cur
                    weight[neighbour] = weight[cur] + length
            elif state[neighbour] == 1 and weight[cur] + length < weight[neighbour]:
                    cameFrom[neighbour] = cur
                    weight[neighbour] = weight[cur] + length
            elif state[neighbour] == 0 and weight[neighbour] > weight[cur] + length:
                urgent_queue.put(neighbour)
                state[neighbour] = 1
                weight[neighbour] = weight[cur] + length
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

with open("adjList.csv", 'w') as filecsv:
    csv.writer(filecsv).writerows(resMap.items())

# with open("matrix.csv", 'w') as filecsv:
#     output = csv.writer(filecsv)
#     output.writerow([''] + list(resMap.keys()))
#
#     for first_vertex in resMap:
#         matrix_row = []
#         for second_vertex in resMap:
#             if second_vertex in resMap[first_vertex]:
#                 matrix_row.append(1)
#             else:
#                 matrix_row.append(0)
#         output.writerow([first_vertex] + list(matrix_row))

scale = 2000.0

scaleLat = (maxlat - minlat) / scale
scaleLong = (maxlong - minlong) / scale
resGraph = svgwrite.Drawing("city_graph_with_paths.svg", size=(str(scale) + 'px', str(scale) + 'px'))

for first_point in resMap:
    # resGraph.add(resGraph.circle((scale - (maxlong - coordMap[first_point][1]) / scaleLong,
    #      (maxlat - coordMap[first_point][0]) / scaleLat), 2))
    for second_point in resMap[first_point]:
        # resGraph.add(resGraph.circle((scale - (maxlong - coordMap[second_point][1]) / scaleLong,
        #      (maxlat - coordMap[second_point][0]) / scaleLat), 2))
        resGraph.add(resGraph.line((scale - (maxlong - coordMap[first_point][1]) / scaleLong,
             (maxlat - coordMap[first_point][0]) / scaleLat),
            (scale - (maxlong - coordMap[second_point][1])/ scaleLong,
             (maxlat - coordMap[second_point][0])/ scaleLat), stroke="black"))

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


routes = []

for destination in hospitals:
    distance, route = Dijkstra_method(origin, destination)
    routes.append((distance, route))

routes.sort()

for _, route in routes[1:]:
    i = 0
    while i < len(route) - 1:
        resGraph.add(resGraph.line((scale - (maxlong - coordMap[route[i]][1]) / scaleLong,
             (maxlat - coordMap[route[i]][0]) / scaleLat),
            (scale - (maxlong - coordMap[route[i + 1]][1])/ scaleLong,
             (maxlat - coordMap[route[i + 1]][0])/ scaleLat), stroke="purple", stroke_width=5))
        i += 1

route = routes[0][1]
i = 0
while i < len(route) - 1:
    resGraph.add(resGraph.line((scale - (maxlong - coordMap[route[i]][1]) / scaleLong,
         (maxlat - coordMap[route[i]][0]) / scaleLat),
        (scale - (maxlong - coordMap[route[i + 1]][1])/ scaleLong,
         (maxlat - coordMap[route[i + 1]][0])/ scaleLat), stroke="green", stroke_width=5))
    i += 1

for point in hospitals:
    resGraph.add(resGraph.circle((scale - (maxlong - coordMap[point][1]) / scaleLong,
         (maxlat - coordMap[point][0]) / scaleLat), 10, fill="purple"))

resGraph.add(resGraph.circle((scale - (maxlong - coordMap[origin][1]) / scaleLong,
         (maxlat - coordMap[origin][0]) / scaleLat), 10, fill="blue"))

resGraph.save()

with open('routes.csv', 'w') as csvfile:
    writer = csv.writer(csvfile)
    for _, route in routes:
        writer.writerow(route)

starting_points = random.sample(list(resMap), 100)


dijkstra_total_time, levit_total_time, a_star_total_time = 0, 0, 0
all_dijkstra_distances, all_levit_distances, all_astar_distances = [], [], []
dijkstra_err, levit_err, a_euclid_err, a_manhattan_err, a_chebyshev_err = 0, 0, 0, 0, 0
destination = hospitals[0]


print('Number\tDistance\tDijkstra\tLevit\t\tAstar_Euclid\t\tAstar_Cheb\t\tAstar_Manhat')

for i, point in enumerate(starting_points):
    start_time = time()
    dijkstra_distance, dijkstra_path = Dijkstra_method(point, destination)
    dijkstra_time = time() - start_time
    dijkstra_total_time += dijkstra_time
    all_dijkstra_distances.append(dijkstra_distance)

    start_time = time()
    levit_distance, levit_path = Levit_method(point, destination)
    levit_time = time() - start_time
    levit_total_time += levit_time
    all_levit_distances.append(levit_distance)

    start_time = time()
    a_star_distance, a_star_path = Astar_method(point, destination, Euclid)
    a_star_time = time() - start_time
    a_star_total_time += a_star_time
    all_astar_distances.append(a_star_distance)

    start_time = time()
    a_star_cheb_distance, a_star_cheb_path = Astar_method(point, destination, Chebyshev)
    a_star_cheb_time = time() - start_time

    start_time = time()
    a_star_manhat_distance, a_star_manhat_path = Astar_method(point, destination, Manhattan)
    a_star_manhat_time = time() - start_time

    print("{0}\t\t{1:.3f}\t\t{2:.3f}\t\t{3:.3f}\t\t{4:.3f}\t\t\t\t{5:.3f}\t\t\t{6:.3f}".format(i + 1,
          dijkstra_distance, dijkstra_time, levit_time, a_star_time, a_star_cheb_time, a_star_manhat_time))

    exact_distance = min(dijkstra_distance, levit_distance)
    if exact_distance != float('inf'):
        dijkstra_err += dijkstra_distance/exact_distance - 1
        levit_err += levit_distance/exact_distance - 1
        a_euclid_err += a_star_distance/exact_distance - 1
        a_chebyshev_err += a_star_cheb_distance/exact_distance - 1
        a_manhattan_err += a_star_manhat_distance/exact_distance - 1

print('\nDijkstra total time: {0:.3f} sec'.format(dijkstra_total_time))
print('Levit total time: {0:.3f} sec'.format(levit_total_time))
print('Astar total time: {0:.3f} sec'.format(a_star_total_time))

try:
    all_dijkstra_distances.remove(float('inf'))
    all_levit_distances.remove(float('inf'))
    all_astar_distances.remove(float('inf'))
except ValueError:
    pass

n = len(all_dijkstra_distances)

print('\nAverage dijkstra error: {0:.1f}%'.format(dijkstra_err * 100 / n))
print('Average levit error: {0:.1f}%'.format(levit_err * 100 / n))
print('Average astar (Euclid) error: {0:.1f}%'.format(a_euclid_err * 100 / n))
print('Average astar (Chebyshev) error: {0:.1f}%'.format(a_chebyshev_err * 100 / n))
print('Average astar (Manhattan) error: {0:.1f}%'.format(a_manhattan_err * 100 / n))

print('\nDijkstra average arrival time: {0:.3f} min'.format(sum(all_dijkstra_distances)/len(all_dijkstra_distances)/40*60))
print('Levit average arrival time: {0:.3f} min'.format(sum(all_levit_distances)/len(all_levit_distances)/40*60))
print('Astar average arrival time: {0:.3f} min'.format(sum(all_astar_distances)/len(all_astar_distances)/40*60))

print('Totaltime:', time() - main_time)

