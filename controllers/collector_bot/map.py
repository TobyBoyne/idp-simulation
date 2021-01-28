import matplotlib.pyplot as plt
import numpy as np



ARENA_SIZE = 1.2
arena = ARENA_SIZE * np.array([[-1., -1.], [-1., 1.], [1., 1.], [1., -1.], [-1., -1.]]).T

np_data = np.load('listnp.npy')
X, Y = np_data.T

fig, ax = plt.subplots()
ax.scatter(X, Y)
ax.plot(*arena)

ax.set_xlim(-1.4, 1.4)
ax.set_ylim(-1.4, 1.4)
ax.set_aspect('equal')
plt.show()