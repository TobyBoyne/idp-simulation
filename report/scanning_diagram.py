import numpy as np
import matplotlib.pyplot as plt

ARENA_SIZE = 2.4

arena = ARENA_SIZE * np.array([[0., 0.], [0., 1.], [1., 1.], [1., 0.], [0., 0.]]).T


def plot_scanning(ax, R=1., step_multiple=1., start_multiple=0.7):
    ax.set_aspect('equal')
    ax.plot(*arena, color='darkgrey', linewidth=3)
    ax.tick_params(axis='both', which='both', left=False, bottom=False,
                   labelbottom=False, labelleft=False)

    step = step_multiple * R
    start = start_multiple * R

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

ROWS = 3
COLS = 3

fig, axes = plt.subplots(nrows=ROWS, ncols=COLS)

rs, steps = np.mgrid[0.4:1:1j*ROWS, 0.7:1.3:1j * COLS]
for i, (ax, r, step) in enumerate(zip(axes.flat, rs.flat, steps.flat)):
    plot_scanning(ax, r, step_multiple=step)

    if i % ROWS == 0:
        ax.set_ylabel(f'$R={r}$m',           fontsize=14)

    if i // COLS == 0:
        ax.set_title(f'step size=${step}R$', fontsize=14)



plt.show()