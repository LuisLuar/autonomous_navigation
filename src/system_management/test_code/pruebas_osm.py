import pandas as pd
import matplotlib.pyplot as plt
import numpy as np  
import time

import os
import networkx as nx
import osmnx as ox
import folium


start_point_name = "Pametnik Vasil Levski, Sofia, Bulgaria"
end_point_name = "Sveti Sedmochislenitsi Church, Sofia, Bulgaria"

#start_point_geocode = ox.geocode(start_point_name)
#end_point_geocode = ox.geocode(end_point_name)

start_point_geocode = (-0.314527, -78.442983)
end_point_geocode = (-0.315528, -78.446338)

print("Start Point Geocode:", start_point_geocode)
print("End Point Geocode:", end_point_geocode)

print(f'from {start_point_geocode[0]}, {start_point_geocode[1]} to {end_point_geocode[0]}, {end_point_geocode[1]}')

center_point = (
    (start_point_geocode[0] + end_point_geocode[0]) / 2,
    (start_point_geocode[1] + end_point_geocode[1]) / 2
)

graph = ox.graph_from_point(center_point, dist=1000, network_type='drive')

ox.plot_graph(
    graph,
    bgcolor='yellow',
    edge_color='black',
    node_color = 'black',
    edge_linewidth=0.6,
    node_size=50,
    figsize=(10, 10)
)

origin_node = ox.distance.nearest_nodes(graph, start_point_geocode[1], start_point_geocode[0])
destination_node = ox.distance.nearest_nodes(graph, end_point_geocode[1], end_point_geocode[0])

fig, ax = ox.plot_graph(
    graph,
    bgcolor="black",
    edge_color="white",
    edge_linewidth=0.6,
    node_size=10,
    show=False,
    close=False
)

time.sleep(2)

# Get (x, y) coordinates of the nodes
origin_xy = (graph.nodes[origin_node]['x'], graph.nodes[origin_node]['y'])
destination_xy = (graph.nodes[destination_node]['x'], graph.nodes[destination_node]['y'])

print("Origin XY:", origin_xy)
print("Destination XY:", destination_xy)
# Plot origin (in red) and destination (in lime)
ax.scatter(*origin_xy, s=80, c='red', label='Origin', zorder=3)
ax.scatter(*destination_xy, s=80, c='green', label='Destination', zorder=3)

# Optional: add legend and title
ax.legend(facecolor='white')
plt.title("Graph with Origin (red) and Destination (green) nodes")
plt.show()

route = nx.shortest_path(graph, origin_node, destination_node, weight='length', method='dijkstra')
route_length_m = nx.shortest_path_length(graph, origin_node, destination_node, weight='length', method='dijkstra')
print(f"Shortest road distance: {route_length_m/1000:.2f} km")


ox.plot_graph_route(graph, route, bgcolor='black', edge_color='white', route_color='yellow', node_size=0, edge_linewidth=0.6, route_linewidth=3)

m = folium.Map(
    location=center_point,
    zoom_start=15,
    tiles='cartodbpositron',
    zoom_control=False,
    scrollWheelZoom=False,
    dragging=False
)