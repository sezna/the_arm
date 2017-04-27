import numpy as np
'''
import pylab as plt
import math
import decimal
import numpy as np

X = np.linspace(0,2,1000)
Y = X**2 + np.random.random(X.shape)

plt.ion()
graph = plt.plot(X,Y)[0]
while True:
    Y = X**2 + np.random.random(X.shape)
    graph.set_ydata(Y)
    plt.pause(0.01)
    plt.draw()
'''

import matplotlib.pylab as plt
from matplotlib import cm 
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure(figsize=(8, 8))
ax = fig.gca(projection='3d')

t = np.linspace(-3, 2, 31)
s = np.linspace(-3, 2, 31)

T, S = np.meshgrid(t, s)

ax.plot_surface(T * T, math.sqrt(2)* T * S, S * S, cmap=cm.jet, rstride=1, cstride=1)

ax.set_xlabel('$t^2$')
ax.set_ylabel('$\sqrt{2} s t$')
ax.set_zlabel('$s^2$')

ax.set_title('line $s = t$ in $\cal F$')

plt.show()
