import networkx as nx
import matplotlib.pyplot as plt

G = nx.Graph()
G.add_nodes_from([1,10])
G.add_edge(1,2, length = 10)
pos = nx.spring_layout(G)
nx.draw(G, pos)
#nx.draw_networkx_edge_labels(G, pos)
plt.show()