import json
import matplotlib.pyplot as plt

f = open(r"/home/misha/workspace/Created_file.json")

data = json.load(f)

length_tree = data["tree_size"]
length_path = data["path_size"]

goal_radius = data["goal_radius"]

circle_goal = plt.Circle((data["goal"][0], data["goal"][1]), goal_radius, color='purple', fill=True)
circle_start = plt.Circle((data["start"][0], data["start"][1]), goal_radius, color='green', fill=True)

fig, ax = plt.subplots()
ax.add_patch(circle_goal)
ax.add_patch(circle_start)


obstacles_number = data["obstacles_number"]
for i in range(1, obstacles_number):
    radius = data["obstacle"+str(i)][0]
    x = data["obstacle" + str(i)][1]
    y = data["obstacle" + str(i)][2]

    circle_obstacle = plt.Circle((x, y), radius, fc='red', ec='black', fill=True)
    ax.add_patch(circle_obstacle)


tree2D = []
path2D= []
for i in range(1, length_tree):
    tree2D.append([data["tree"+str(i)][0], data["tree"+str(i)][1], data["tree"+str(i)][3], data["tree"+str(i)][4]])

for i in range(1, length_path):
    path2D.append([data["path"+str(i)][0], data["path"+str(i)][1]])

for i in tree2D:
    # print([i[0], i[2]], [i[1], i[3]])
    ax.plot([i[0], i[2]], [i[1], i[3]], "-o", linewidth=0.5, color='blue', markersize=1)

x = []
y = []
for i in range(0, length_path - 1):
    x.append(path2D[i][0])
    y.append(path2D[i][1])

ax.plot(x, y, "-o", color='black', linewidth=2, markersize=5)
ax.grid(color='k', linestyle='-', linewidth=0.2)
ax.set_aspect('equal', adjustable='box')

ax.plot(data["goal"][0], data["goal"][1], "o", color='black')
ax.plot(data["start"][0], data["start"][1], "o", color='black')

plt.title(data["graph_name"], loc='center')
plt.show()

# Closing file
f.close()

