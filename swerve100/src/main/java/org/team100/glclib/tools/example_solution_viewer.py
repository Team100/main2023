import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt
from matplotlib import cm
from matplotlib.patches import Circle
from matplotlib.collections import LineCollection

### Pendulum swingup demo

#Load results from txt file
path = np.transpose(np.genfromtxt('pendulum_swingup_demo.txt', delimiter=','))
nodes = np.transpose(np.genfromtxt('pendulum_swingup_demo_nodes.txt', delimiter=','))

#Plot data
fig = plt.figure(figsize=(10,10))
ax = plt.gca()
ax.set_xlim([-6.0,6.0])
ax.set_ylim([-6.0,6.0])
#Solution from planner
plt.plot(path[0],path[1])
#Nodes of search tree
plt.scatter(nodes[0],nodes[1],s=0.5)

ax.add_patch(Circle([3.14159, 0.0],0.25, facecolor="green",alpha=0.3,edgecolor="none"))
plt.show()

### Shortest path demo

#Load results from txt file
path = np.transpose(np.genfromtxt('shortest_path_demo.txt', delimiter=','))
nodes = np.transpose(np.genfromtxt('shortest_path_demo_nodes.txt', delimiter=','))

#Plot data
fig = plt.figure(figsize=(10,10))
ax = plt.gca()
#ax.set_xlim([-1,12])
#ax.set_ylim([-1,12])
#Solution from planner
plt.plot(path[0],path[1])
#Nodes of search tree
# new! colored by cost!
# new! lines!
edges = list(zip(zip(nodes[1],nodes[2]), zip(nodes[3],nodes[4])))
lc = LineCollection(edges, array=nodes[0])
ax.add_collection(lc)
#plt.scatter(nodes[1],nodes[2],c=nodes[0],s=5)

ax.add_patch(Circle([3.0, 2.0],2.0, facecolor="black",alpha=0.7,edgecolor="none"))
ax.add_patch(Circle([6.0, 8.0],2.0, facecolor="black",alpha=0.7,edgecolor="none"))
ax.add_patch(Circle([10.0, 10.0],0.25, facecolor="green",alpha=0.3,edgecolor="none"))
plt.autoscale()
plt.show()

### Nonholonomic car demo

#Load results from txt file
path = np.transpose(np.genfromtxt('nonholonomic_car_demo.txt', delimiter=','))
nodes = np.transpose(np.genfromtxt('nonholonomic_car_demo_nodes.txt', delimiter=','))

#Plot data
fig = plt.figure(figsize=(10,10))
ax = plt.gca()
ax.set_xlim([-1,12])
ax.set_ylim([-1,12])
#Solution from planner
plt.plot(path[0],path[1])
#Nodes of search tree
plt.scatter(nodes[0],nodes[1],s=0.5)

ax.add_patch(Circle([3.0, 2.0],2.0, facecolor="black",alpha=0.7,edgecolor="none"))
ax.add_patch(Circle([6.0, 8.0],2.0, facecolor="black",alpha=0.7,edgecolor="none"))
ax.add_patch(Circle([10.0, 10.0],0.5, facecolor="green",alpha=0.3,edgecolor="none"))
plt.show()
