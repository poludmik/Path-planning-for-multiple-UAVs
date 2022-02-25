import json
import matplotlib.pyplot as plt

f = open(r"/home/misha/workspace/src/uav-usv-path-planning/src/Multiple_trajectories.json")

data = json.load(f)

fig, ax = plt.subplots()

colors = ["crimson", "red", "brown", "maroon", "chocalate", "peru", "salmon"]

for drone_idx in range(1, data["number_of_drones"] + 1):

    path_size = data["path_size_of_uav" + str(drone_idx)]

    goal_radius = data["uav" + str(drone_idx) + "_goal_radius"]

    circle_goal = plt.Circle((data["goal_of_uav"+str(drone_idx)][0], data["goal_of_uav"+str(drone_idx)][1]), goal_radius, color='royalblue', fill=True)
    circle_start = plt.Circle((data["start_of_uav"+str(drone_idx)][0], data["start_of_uav"+str(drone_idx)][1]), goal_radius, color='limegreen', fill=True)

    ax.plot(data["goal_of_uav"+str(drone_idx)][0], data["goal_of_uav"+str(drone_idx)][1], "o", color='black')
    ax.plot(data["start_of_uav"+str(drone_idx)][0], data["start_of_uav"+str(drone_idx)][1], "o", color='black')

    ax.add_patch(circle_goal)
    ax.add_patch(circle_start)

    path2D= []
    length_path = data["path_size_of_uav" + str(drone_idx)]
    for i in range(0, length_path):
        path2D.append([data["uav" + str(drone_idx) + "_path_point" + str(i)][0], data["uav" + str(drone_idx) + "_path_point" + str(i)][1]])

    x = []
    y = []
    for i in range(0, length_path):
        x.append(path2D[i][0])
        y.append(path2D[i][1])

    ax.plot(x, y, "-o", color=colors[drone_idx-1], linewidth=2, markersize=5)



obstacles_number = data["obstacles_number"]

# changed index to 0 while writing, don't know why I had it from 1
for i in range(0, obstacles_number):
    radius = data["obstacle"+str(i)][0]
    x = data["obstacle" + str(i)][1]
    y = data["obstacle" + str(i)][2]

    circle_obstacle = plt.Circle((x, y), radius, fc='red', ec='black', fill=True)
    ax.add_patch(circle_obstacle)




ax.grid(color='k', linestyle='-', linewidth=0.2)
ax.set_aspect('equal', adjustable='box')

plt.title(data["graph_title"], loc='center')
plt.show()

# Closing file
f.close()
