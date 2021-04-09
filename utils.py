import math
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib import lines

def euclidean_distance(a, b):
    """Returns the euclidean distance between a and b. Parameters have to be vectors."""
    return math.hypot(a[0]-b[0], a[1]-b[1])

def show_map(node_locs, graph):
    """Given node 2Dlocations as list and a graph shows a graphical representation of it."""
    # initialise a graph
    G = nx.Graph()
    
    # use this while labeling nodes in the map
    node_labels = dict()
    # use this to modify colors of nodes while exploring the graph.
    # This is the only dict we send to `show_map(node_colors)` while drawing the map
    node_colors = dict()
    
    for n, p in node_locs.items():
        # add nodes from romania_locations
        G.add_node(n)
        # add nodes to node_labels
        node_labels[n] = n
        # node_colors to color nodes while exploring romania map
        node_colors[n] = "black"
    
    #TODO: maybe not necessary--------------------------------------
    # we'll save the initial node colors to a dict to use later
    initial_node_colors = dict(node_colors)
        
    # positions for node labels
    node_label_pos = { k:[v[0],v[1]-10]  for k,v in node_locs.items() }
    #----------------------------------------------------------
    
    # use this while labeling edges
    edge_labels = dict()
    
    # add edges between cities in romania map - UndirectedGraph defined in search.py
    for node in graph.nodes():
        connections = graph.get(node)
        for connection in connections.keys():
            distance = connections[connection]
    
            # add edges to the graph
            G.add_edge(node, connection)
            # add distances to edge_labels
            edge_labels[(node, connection)] = distance
    
    # set the size of the plot
    plt.figure(figsize=(18,13))
    
    # draw the graph (both nodes and edges) with locations from romania_locations
    nx.draw(G, pos = node_locs, node_color = [node_colors[node] for node in G.nodes()])

    # draw labels for nodes
    node_label_handles = nx.draw_networkx_labels(G, pos = node_label_pos, labels = node_labels, font_size = 14)
    # add a white bounding box behind the node labels
    [label.set_bbox(dict(facecolor='white', edgecolor='none')) for label in node_label_handles.values()]

    # add edge lables to the graph
    nx.draw_networkx_edge_labels(G, pos = node_locs, edge_labels=edge_labels, font_size = 14)
    
    # add a legend
    black_circle = lines.Line2D([], [], color="black", marker='o', markersize=15, markerfacecolor="black")
    orange_circle = lines.Line2D([], [], color="orange", marker='o', markersize=15, markerfacecolor="orange")
    red_circle = lines.Line2D([], [], color="red", marker='o', markersize=15, markerfacecolor="red")
    gray_circle = lines.Line2D([], [], color="gray", marker='o', markersize=15, markerfacecolor="gray")
    green_circle = lines.Line2D([], [], color="green", marker='o', markersize=15, markerfacecolor="green")
    plt.legend((black_circle, orange_circle, red_circle, gray_circle, green_circle),
               ('Un-explored', 'Frontier', 'Currently Exploring', 'Explored', 'Final Solution'),
               numpoints=1,prop={'size':16}, loc=(.8,.75))
    
    # show the plot. No need to use in notebooks. nx.draw will show the graph itself.
    plt.show()

def show_path(path, node_locs, graph):
    """Given node 2Dlocations as list and a graph shows a graphical representation of it."""
    # initialise a graph
    G = nx.Graph()
    
    # use this while labeling nodes in the map
    node_labels = dict()
    # use this to modify colors of nodes while exploring the graph.
    # This is the only dict we send to `show_map(node_colors)` while drawing the map
    node_colors = dict()
    
    for n, p in node_locs.items():
        # add nodes from romania_locations
        G.add_node(n)
        # add nodes to node_labels
        node_labels[n] = n
        # node_colors to color nodes while exploring romania map
        node_colors[n] = "black" if n not in path else "green"
    
    #TODO: maybe not necessary--------------------------------------
    # we'll save the initial node colors to a dict to use later
    initial_node_colors = dict(node_colors)
        
    # positions for node labels
    node_label_pos = { k:[v[0],v[1]-10]  for k,v in node_locs.items() }
    #----------------------------------------------------------
    
    # use this while labeling edges
    edge_labels = dict()
    
    # add edges between cities in romania map - UndirectedGraph defined in search.py
    for node in graph.nodes():
        connections = graph.get(node)
        for connection in connections.keys():
            distance = connections[connection]
    
            # add edges to the graph
            G.add_edge(node, connection)
            # add distances to edge_labels
            edge_labels[(node, connection)] = distance
    
    # set the size of the plot
    plt.figure(figsize=(18,13))
    
    # draw the graph (both nodes and edges) with locations from romania_locations
    nx.draw(G, pos = node_locs, node_color = [node_colors[node] for node in G.nodes()])

    # draw labels for nodes
    node_label_handles = nx.draw_networkx_labels(G, pos = node_label_pos, labels = node_labels, font_size = 14)
    # add a white bounding box behind the node labels
    [label.set_bbox(dict(facecolor='white', edgecolor='none')) for label in node_label_handles.values()]

    # add edge lables to the graph
    nx.draw_networkx_edge_labels(G, pos = node_locs, edge_labels=edge_labels, font_size = 14)
    
    # add a legend
    black_circle = lines.Line2D([], [], color="black", marker='o', markersize=15, markerfacecolor="black")
    orange_circle = lines.Line2D([], [], color="orange", marker='o', markersize=15, markerfacecolor="orange")
    red_circle = lines.Line2D([], [], color="red", marker='o', markersize=15, markerfacecolor="red")
    gray_circle = lines.Line2D([], [], color="gray", marker='o', markersize=15, markerfacecolor="gray")
    green_circle = lines.Line2D([], [], color="green", marker='o', markersize=15, markerfacecolor="green")
    plt.legend((black_circle, orange_circle, red_circle, gray_circle, green_circle),
               ('Un-explored', 'Frontier', 'Currently Exploring', 'Explored', 'Final Solution'),
               numpoints=1,prop={'size':16}, loc=(.8,.75))
    
    # show the plot. No need to use in notebooks. nx.draw will show the graph itself.
    plt.show()