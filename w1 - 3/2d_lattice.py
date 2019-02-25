'''
This code creates and draws a node-and-link 2D network.
'''

import matplotlib.pyplot as plt
import networkx as nx 
import random



'''
G = nx.grid_2d_graph(4,4) # 4 by 4 grid
pos = nx.spring_layout(G, iterations=100)

plt.subplot(221)
nx.draw(G, pos, font_size=8)

plt.subplot(222)
nx.draw(G, pos, node_color='k', node_size=0, with_labels=False)

plt.subplot(223)
nx.draw(G, pos, node_color='g', node_size=250, with_labels=False, width=6)

plt.subplot(224)
H = G.to_directed()
nx.draw(H, pos, node_color='b', node_size=20, with_labels=False)

plt.show()
'''

'''
G = nx.grid_2d_graph(4,4) # 4 by 4 grid

plt.subplot(221)
nx.draw(G, font_size=8)

plt.subplot(222)
nx.draw(G, node_color='k', node_size=0, with_labels=False)

plt.subplot(223)
nx.draw(G, node_color='g', node_size=250, with_labels=False, width=6)

plt.subplot(224)
H = G.to_directed()
nx.draw(H, node_color='b', node_size=20, with_labels=False)

plt.show()
'''

G = nx.grid_2d_graph(3,3)

pos = dict((n,n) for n in G.nodes())

plt.subplot(111, frame_on=True)
fig = nx.draw(G, pos, with_labels=True, node_size=1)

plt.axis([-1,5,-1,5])

# print(G.nodes)
print pos
print 'Number of nodes:', G.number_of_nodes()
print 'Number of edges:', G.number_of_edges()
print random.choice(list(G.nodes)) #pick a random node

plt.show()
