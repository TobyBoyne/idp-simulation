import matplotlib.pyplot as plt
import numpy as np

ARENA_SIZE = 1.2
arena = ARENA_SIZE * np.array([[-1., -1.], [-1., 1.], [1., 1.], [1., -1.], [-1., -1.]]).T

red_home = np.array([[0.8, 1.2, 1.2, 0.8], [0.8, 0.8, 1.2, 1.2]])
blue_home = (red_home.T + np.array([0., -2., ])).T

fig, ax = plt.subplots()
ax.add_artist(plt.Polygon(red_home.T, color='tab:red'))
ax.add_artist(plt.Polygon(blue_home.T, color='blue'))
ax.plot(*arena)

scanning_points = np.array([[1., 1.], [0.6, 0.5], [0.1, 0.4], [0., 0.]])
for pt in scanning_points:
	circle = plt.Circle(pt, radius=0.4+0.3, alpha=0.05, color='green')
	ax.add_artist(circle)
	ax.plot(*pt, 'x', color='green')


np.random.seed(2)

def findClusters(data):
	"""Find the clusters within the scanned datapoints"""
	centroids = data[0, None]
	min_R = 0.1

	for _ in range(8):
		dists = np.linalg.norm(data[:, None, :] - centroids[None, :, :], axis=-1)
		potentials = (1 / dists).sum(axis=1)

		new_c_idx = np.argmin(potentials)

		if np.min(dists[new_c_idx]) < min_R:
			# if this is close to an existing centroid, stop finding centroids
			break

		centroids = np.concatenate([centroids, data[new_c_idx, None]], axis=0)

	ax.scatter(*centroids.T, color='tab:orange')

	# run a single k-means to find the centroid of each cluster
	k = centroids.shape[0]
	dists = np.linalg.norm(data[:, None, :] - centroids[None, :, :], axis=-1)
	closest_centroid = np.argmin(dists, axis=-1)

	for n in range(k):
		new_centroid = data[closest_centroid == n].mean(axis=0)
		centroids[n] = new_centroid

	ax.scatter(*centroids.T, color='tab:blue')





np_data = np.load('listnp.npy')
X, Y = np_data.T

ax.scatter(X, Y, alpha=0.1, color='grey')

# findClusters(np_data)

np_boxes = np.load('box_locations.npy')
x, y = np_boxes.T
ax.scatter(x, y)

ax.set_xlim(-1.4, 1.4)
ax.set_ylim(1.4, -1.4)
ax.set_aspect('equal')
plt.show()