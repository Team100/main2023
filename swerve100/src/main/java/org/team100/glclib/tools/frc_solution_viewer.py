import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt
from matplotlib import cm
from matplotlib.patches import Circle, Rectangle
from matplotlib.collections import LineCollection
import matplotlib.ticker as ticker


### Shortest path demo

#Load results from txt file
actualpath = np.transpose(np.genfromtxt('path.txt', delimiter=','))
path = np.transpose(np.genfromtxt('frc_shortest_path_demo.txt', delimiter=','))
nodes = np.transpose(np.genfromtxt('frc_shortest_path_demo_nodes.txt', delimiter=','))
cost = nodes[0]
px1 = nodes[1]
py1 = nodes[2]
vx1 = nodes[3]
vy1 = nodes[4]
px2 = nodes[5]
py2 = nodes[6]
vx2 = nodes[7]
vy2 = nodes[8]

#Plot data
fig = plt.figure(figsize=(16,8))
ax = plt.gca()
#ax.set_xlim([-1,12])
#ax.set_ylim([-1,12])
#Nodes of search tree
# new! colored by cost!
# new! lines!
# these are in position space
edges = list(zip(zip(px1,py1), zip(px2,py2)))
# these are in velocity space
# edges = list(zip(zip(vx1,vy1), zip(vx2,vy2)))
lc = LineCollection(edges, array=cost, alpha=0.1)
ax.add_collection(lc)
#plt.scatter(nodes[1],nodes[2],c=nodes[0],s=5)

# outline
ax.add_patch(Rectangle([0,0], 16.54,8.02, facecolor="none", alpha=1.0, edgecolor="black"))
# red charge station
ax.add_patch(Rectangle([2.98, 1.51], 1.93, 2.44, facecolor="red", alpha=0.7, edgecolor="none"))
# blue charge station
ax.add_patch(Rectangle([11.63, 1.51], 1.93, 2.44, facecolor="blue", alpha=0.7, edgecolor="none"))
# nodes
ax.add_patch(Rectangle([0, 0], 1.43, 5.49, facecolor="red", alpha=0.7, edgecolor="none"))
# opponent community
ax.add_patch(Rectangle([13.18, 0], 3.36, 5.49, facecolor="blue", alpha=0.7, edgecolor="none"))
# opponent loading
ax.add_patch(Rectangle([0,5.49], 3.36, 1.26, facecolor="blue", alpha=0.7, edgecolor="none"))
# opponent loading
ax.add_patch(Rectangle([0, 6.75], 6.71, 1.26, facecolor="blue", alpha=0.7, edgecolor="none"))
# example opponents in motion, 1s snapshots
ax.add_patch(Rectangle([3.5, 5.5], 1, 1, facecolor="blue", alpha=0.3, edgecolor="black"))
ax.add_patch(Rectangle([3.5, 6.5], 1, 1, facecolor="blue", alpha=0.3, edgecolor="black"))

ax.add_patch(Rectangle([5.5, 4.5], 1, 1, facecolor="blue", alpha=0.3, edgecolor="black"))
ax.add_patch(Rectangle([5.5, 6.25], 1, 1, facecolor="blue", alpha=0.3, edgecolor="black"))

ax.add_patch(Rectangle([7.5, 3.5], 1, 1, facecolor="blue", alpha=0.3, edgecolor="black"))
ax.add_patch(Rectangle([7.5, 6.0], 1, 1, facecolor="blue", alpha=0.3, edgecolor="black"))

ax.add_patch(Rectangle([9.5, 2.5], 1, 1, facecolor="blue", alpha=0.3, edgecolor="black"))
ax.add_patch(Rectangle([9.5, 5.75], 1, 1, facecolor="blue", alpha=0.3, edgecolor="black"))

ax.add_patch(Rectangle([11.5, 1.5], 1, 1, facecolor="blue", alpha=0.3, edgecolor="black"))
ax.add_patch(Rectangle([11.5, 5.5], 1, 1, facecolor="blue", alpha=0.3, edgecolor="black"))

# defender
ax.add_patch(Rectangle([7.5, 4.5], 1, 1, facecolor="blue", alpha=0.3, edgecolor="black"))

# ax.add_patch(Circle([3.0, 2.0],2.0, facecolor="black",alpha=0.7,edgecolor="none"))
# ax.add_patch(Circle([6.0, 8.0],2.0, facecolor="black",alpha=0.7,edgecolor="none"))
# start
ax.add_patch(Circle([16.179, 6.75],0.25, facecolor="green",alpha=0.3,edgecolor="none"))
# end, note this is not tag pose
ax.add_patch(Circle([1.93, 2.748],0.25, facecolor="green",alpha=0.3,edgecolor="none"))



#Solution from planner
# this is actually a spline
#pathx = path[0]
#pathy = path[1]
#pathvx = path[2]
#pathvy = path[3]
#pathv = np.sqrt(pathvx * pathvx + pathvy * pathvy)
#plt.scatter(pathx,pathy,c=pathv,marker='s',s=200,zorder=2,linewidth=2,edgecolor='black')

# actual path
actualpathtime= actualpath[0]
actualpathx = actualpath[1]
actualpathy = actualpath[2]
actualpathvx = actualpath[3]
actualpathvy = actualpath[4]
ux = actualpath[5]
uy = actualpath[6]
plt.scatter(actualpathx, actualpathy, marker='s',s=200,zorder=3,linewidth=2, color='red',edgecolor='black')

ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
ax.set_aspect('equal', adjustable='box')

plt.autoscale()

plt.figure()

ax = plt.gca()
ax.set_aspect('equal', adjustable='box')
edges = list(zip(zip(vx1,vy1), zip(vx2,vy2)))
lc = LineCollection(edges, array=cost, alpha=0.1)
ax.add_collection(lc)
plt.scatter(actualpathvx, actualpathvy, marker='s', s=100, zorder=2, linewidth=2, edgecolor='black')
plt.plot(actualpathvx, actualpathvy)
plt.xlim([-5,5])
plt.ylim([-5,5])
plt.xlabel('x velocity')
plt.ylabel('y velocity')



plt.figure()
plt.plot(actualpathtime, ux)
plt.xlabel('time')
plt.ylabel('ux')
plt.figure()
plt.plot(actualpathtime, uy)
plt.xlabel('time')
plt.ylabel('uy')
plt.figure()

plt.plot(ux, uy)
plt.xlabel('ux')
plt.ylabel('uy')
ax = plt.gca()
ax.set_aspect('equal', adjustable='box')
plt.show()