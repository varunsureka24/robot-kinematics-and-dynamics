import numpy as np
import matplotlib.pyplot as plt

waypoints_loaded = []

with open('whiteboard_pose.txt', 'r') as f:
    lines = f.readlines()


if len(lines) % 4 != 0:
    raise ValueError("The number of lines in the file is not a multiple of 4. Each waypoint should consist of 4 lines.")

for i in range(0, len(lines), 4):
    matrix = []
    for j in range(4):
        line = lines[i + j].strip()
        # Split the line into individual float values
        row = list(map(float, line.split()))
        if len(row) != 4:
            raise ValueError(f"Invalid row length in waypoint starting at line {i + 1}: {line}")
        matrix.append(row)
    # Convert to a NumPy array
    mat = np.array(matrix)
    waypoints_loaded.append(mat)
# Now waypoints_loaded is a list of 4x4 NumPy arrays
print("Loaded waypoints:")
for idx, wpt in enumerate(waypoints_loaded):
    print(f"Waypoint {idx}:")
    print(wpt)
    print()

xs = []
ys = []
zs = []
for wpt in waypoints_loaded:
    pos = wpt[:3, 3]
    xs.append(pos[0])
    ys.append(pos[1])
    zs.append(pos[2])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(xs, ys, zs, marker='o')
ax.set_title("3D Trajectory")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
plt.show()
