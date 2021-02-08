import numpy as np
import matplotlib.pyplot as plt

# TODO:
#  check target is within arena walls
#  intelligently turn left/right
#  only move a maximum of SCAN_R distance



# plotting arena
ARENA_SIZE = 1.2
arena = ARENA_SIZE * np.array([[-1., -1.], [-1., 1.], [1., 1.], [1., -1.], [-1., -1.]]).T

red_home = np.array([[0.8, 1.2, 1.2, 0.8], [0.8, 0.8, 1.2, 1.2]])
blue_home = (red_home.T + np.array([0., -2., ])).T

fig, ax = plt.subplots()
ax.add_artist(plt.Polygon(red_home.T, color='tab:red'))
ax.add_artist(plt.Polygon(blue_home.T, color='blue'))
ax.plot(*arena)

ax.set_xlim(-1.4, 1.4)
ax.set_ylim(1.4, -1.4)
ax.set_aspect('equal')

# defining position of boxes
box_coords = np.array([
    [0.6, 1.],
    [0.7, 0.8],
    [0., 0.],
])

boxes = [0, 1, 2]

found_boxes = box_coords[boxes]

SCAN_R = 0.4

# factors radius of robot and boxes to give space
CLEARANCE = 0.2


def getWaypoints(R=0.4, step_multiple=1.):
    """Create a series of commands that make the robot take a path to scan the field"""
    start = np.array([0.92, 0.95])
    step = -step_multiple * R

    X = np.arange(start[0], -1.1, step)
    Z = np.arange(start[1], 0, step)

    # plot a 'snaking' path across half of arena
    path_xs = np.concatenate([X[::(-1) ** (i % 2)] for i in range(len(Z))])
    path_zs = np.repeat(Z, len(X))

    # create a list of commands to move and scan at each point
    waypoints = np.stack([path_xs, path_zs], axis=-1)

    return waypoints

waypoints = getWaypoints()


# calculations
def pointInsideWalls(p):
    x, y = p
    # L is slightly less than the actual width to account for noise in distance sensor
    L = 1.16
    return -L <= x <= L and -L <= y <= L

def minDistance(u, v, B):
    """Find the minimum distance between the line segment defined by u and v, and all of the boxes B
    If v ~ =, then the two points are the same and no movement is needed"""
    # clamp the point to the line segment using parameter t
    # r = u + vt
    len_sqrd = np.linalg.norm(v) ** 2
    if len_sqrd < 1e-4: return np.inf
    t = np.clip(np.dot(B - u, v) / len_sqrd, 0, 1)
    projection = u + t[:, None] * v
    return np.min(np.linalg.norm(projection - B, axis=-1), initial=np.inf)


pos = waypoints[0]
waypoints_list = [pt for pt in waypoints]
for pt in waypoints:
    move_vec = pt - pos
    # move_length = np.clip(np.linalg.norm(move_vec),)

    # rotate the movement vector until a possible direction to move in is found
    for i in range(100):
        t = i * (np.pi / 50) * (-1)**(i % 2)
        # TODO: check target is within arena!
        if pointInsideWalls(pos + move_vec):
            if minDistance(pos, move_vec, found_boxes) > CLEARANCE:
                break

        rot = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
        move_vec = np.dot(rot, pt - pos)

    else:
        # this runs if the for loop does not break
        # stuck in current circle
        plt.show()
        raise RuntimeError('Stuck in a loop')

    x1, y1 = pos
    x2, y2 = pos + move_vec
    ax.plot([x1, x2], [y1, y2], color='tab:green')
    pos = pos + move_vec



# plot boxes
w = 0.05
for box_idx in boxes:
    box_pos = box_coords[box_idx]
    square = plt.Rectangle(box_pos - w/2, w, w, color='black')
    ax.add_artist(square)

# plot waypoints
ax.plot(*waypoints.T, color='tab:green', linestyle='--', marker='x')



plt.show()