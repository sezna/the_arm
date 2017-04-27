import numpy as np
import matplotlib.pylab as plt
from matplotlib import cm 
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure(figsize=(8, 8))
ax = fig.gca(projection='3d')

t = np.linspace(-3, 2, 10)
s = np.linspace(-3, 2, 10)

T, S = np.meshgrid(t, s)

ax.plot_surface(T,  S, S, cmap=cm.jet, rstride=1, cstride=1)

ax.set_xlabel('$t^2$')
ax.set_ylabel('$\sqrt{2} s t$')
ax.set_zlabel('$s^2$')

ax.set_title('line $s = t$ in $\cal F$')

plt.show()
