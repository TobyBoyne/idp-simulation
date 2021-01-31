import matplotlib.pyplot as plt
import numpy as np

ARENA_SIZE = 1.2
arena = ARENA_SIZE * np.array([[-1., -1.], [-1., 1.], [1., 1.], [1., -1.], [-1., -1.]]).T
fig, ax = plt.subplots()

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
ax.plot(*arena)

findClusters(np_data)

ax.set_xlim(0, 1.4)
ax.set_ylim(0, 1.4)
ax.set_aspect('equal')
plt.show()