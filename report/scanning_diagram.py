import numpy as np
import matplotlib.pyplot as plt

ARENA_SIZE = 1.2

arena = 2 * ARENA_SIZE * np.array([[0., 0.], [0., 1.], [1., 1.], [1., 0.], [0., 0.]]).T - ARENA_SIZE
print(arena)
red_home = np.array([[0.8, 1.2, 1.2, 0.8], [0.8, 0.8, 1.2, 1.2]])
blue_home = (red_home.T + np.array([0., -2., ])).T

def plot_scanning_path(ax, R=1., step_multiple=1., start_multiple=0.7):
    ax.set_aspect('equal')
    ax.plot(*arena, color='darkgrey', linewidth=3)
    ax.tick_params(axis='both', which='both', left=False, bottom=False,
                   labelbottom=False, labelleft=False)

    step = step_multiple * R
    # start = start_multiple * R
    start = 0.4
    X = np.arange(start, ARENA_SIZE / 2, step)
    Y = np.arange(start, ARENA_SIZE, step)

    for x in X:
        for y in Y:
            circle = plt.Circle((x, y), radius=R, alpha=0.1)
            ax.add_artist(circle)

            circle = plt.Circle((ARENA_SIZE-x, y), radius=R, alpha=0.1, color='tab:red')
            ax.add_artist(circle)

    path_xs = np.repeat(X, len(Y))
    path_ys = np.concatenate([Y[::(-1)**(i%2)] for i in range(len(X))])

    ax.plot(path_xs, path_ys, 'o-', color='tab:blue', linewidth=2)
    ax.plot(ARENA_SIZE - path_xs, path_ys, 'o-', color='tab:red', linewidth=2)


def plot_scanning_waypoints(ax, waypoints, radius=1., plot_reflection=True):
    ax.set_xlim(-1.4, 1.4)
    ax.set_ylim(1.4, -1.4)
    ax.set_aspect('equal')
    ax.plot(*arena, color='darkgrey', linewidth=3)
    ax.tick_params(axis='both', which='both', left=False, bottom=False,
                   labelbottom=False, labelleft=False)

    ax.add_artist(plt.Polygon(red_home.T, color='tab:red', alpha=0.2))
    ax.add_artist(plt.Polygon(blue_home.T, color='tab:blue', alpha=0.2))

    for x, y in waypoints:
        circle = plt.Circle((x, y), radius=radius, color='tab:red', alpha=0.2)
        ax.add_artist(circle)
        if plot_reflection:
            circle = plt.Circle((x, - y), radius=radius, alpha=0.1, color='tab:blue')
            ax.add_artist(circle)

    ax.plot(*waypoints.T, 'o-', color='tab:red', linewidth=2)
    if plot_reflection:
        blue_waypoints = waypoints * np.array([1, -1])
        ax.plot(*blue_waypoints.T, 'o-', color='tab:blue', linewidth=2)


# ROWS = 3
# COLS = 3
#
# fig, axes = plt.subplots(nrows=ROWS, ncols=COLS)
#
# rs, steps = np.mgrid[0.4:1:1j*ROWS, 0.7:1.3:1j * COLS]
# for i, (ax, r, step) in enumerate(zip(axes.flat, rs.flat, steps.flat)):
#     plot_scanning_path(ax, r, step_multiple=step)
#
#     if i % ROWS == 0:
#         ax.set_ylabel(f'$R={r}$m',           fontsize=14)
#
#     if i // COLS == 0:
#         ax.set_title(f'step size=${step}R$', fontsize=14)

waypoints = np.array([
    [1., 1.],
    [.8, 1.],
    [.6, 1.],
])


fig, ax = plt.subplots()
plot_scanning_waypoints(ax, waypoints, radius=0.5, plot_reflection=True)
plt.show()