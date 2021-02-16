import numpy as np
import matplotlib.pyplot as plt

np.random.seed(10)

red = np.random.rand(2, 4)
blue = np.random.rand(2, 4)

fig, ax = plt.subplots()

ax.plot(*red, 'ro')
ax.plot(*blue, 'bo')

ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_aspect('equal')

plt.show()
