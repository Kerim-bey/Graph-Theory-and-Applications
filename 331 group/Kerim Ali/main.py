import svgwrite, csv
from lxml import etree

minlat = 90.0
minlong = 180.0
maxlat = -90.0
maxlong = -180.0

graph = etree.parse("map.osm")

resMap = {}
coordMap = {}

for node in graph.iterfind('/node'):
    coordMap[node.get('id')] = [float(node.get('lat')),
                                       float(node.get('lon'))]

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

with open("matrix.csv", 'w') as filecsv:
    output = csv.writer(filecsv)
    output.writerow([''] + list(resMap.keys()))

    for first_vertex in resMap:
        matrix_row = []
        for second_vertex in resMap:
            if second_vertex in resMap[first_vertex]:
                matrix_row.append(1)
            else:
                matrix_row.append(0)
        output.writerow([first_vertex] + list(matrix_row))

scale = 2000.0

scaleLat = (maxlat - minlat) / scale
scaleLong = (maxlong - minlong) / scale
resGraph = svgwrite.Drawing("city_graph.svg", size=(str(scale) + 'px', str(scale) + 'px'))

for first_point in resMap:
    resGraph.add(resGraph.circle((scale - (maxlong - coordMap[first_point][1]) / scaleLong,
         (maxlat - coordMap[first_point][0]) / scaleLat), 2))
    for second_point in resMap[first_point]:
        resGraph.add(resGraph.circle((scale - (maxlong - coordMap[second_point][1]) / scaleLong,
             (maxlat - coordMap[second_point][0]) / scaleLat), 2))
        resGraph.add(resGraph.line((scale - (maxlong - coordMap[first_point][1]) / scaleLong,
             (maxlat - coordMap[first_point][0]) / scaleLat),
            (scale - (maxlong - coordMap[second_point][1])/ scaleLong,
             (maxlat - coordMap[second_point][0])/ scaleLat), stroke=svgwrite.rgb(0, 255, 0, "%")))

resGraph.save()