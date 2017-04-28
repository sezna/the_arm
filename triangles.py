import math
from matplotlib import pyplot
import pylab
from mpl_toolkits.mplot3d import Axes3D


fig = pylab.figure()
ax = Axes3D(fig)

def get_second_point(initial_x, initial_y, initial_z, theta_1, theta_2):
	return initial_x + (10 * math.cos(theta_1)), initial_y + (15 * math.sin(theta_2)), initial_z

xs = [0, 3, 20]
ys = [0, 13, 20]
zs = [10, 15, 20]
ax.scatter(xs, ys, zs)

pyplot.scatter(10, 10, 10)
pyplot.ion()

for i in range(100):
	x, y, z = get_second_point(10, 10, 10, i, i)
	pyplot.scatter(x, y, z)
	pyplot.pause(0.05)
