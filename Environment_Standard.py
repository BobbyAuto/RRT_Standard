import datetime

import matplotlib.pyplot as plt

from autonomous.Obstacles import load_obstacles
from autonomous.ObstaclesSpecial import ObstaclesSpecial, generate_obstacles
from autonomous.OptimizePath import OptimizePath
from autonomous.RRTStandard import *
from autonomous.Tools import Tools

departure = (-380, -50)
destination = (400, 100)

# departure = (0, -25)
# destination = (0, -65)

safeRadius = 5

fig, ax = plt.subplots(figsize=(10, 5))

txt_title = ax.set_title('Path Planning')

obstacles = load_obstacles(50)
# obstacles = generate_obstacles(4, style='S')
# draw obstacles
for obstacle in obstacles:
    x, y = obstacle.get_obstacle_axis()
    plt.plot(x, y, color='black')
    plt.fill(x, y, color='gray')

# draw a departure and destination on map
plt.scatter(destination[0], destination[1], s=28, c='red')
plt.plot(departure[0], departure[1], 'b.')

# draw safety circuit around the robot
thia = np.arange(0, 2 * np.pi, 0.01)
x_circuit = departure[0] + safeRadius * np.cos(thia)
y_circuit = departure[1] + safeRadius * np.sin(thia)
plt.plot(x_circuit, y_circuit, color='gray', linewidth=1, alpha=0.3)

search_space = Tools.get_search_space(obstacles, departure, destination, safeRadius)
plt.plot([search_space['min_right'], search_space['min_right'], search_space['max_left'], search_space['max_left'],
          search_space['min_right']],
         [search_space['max_down'], search_space['min_top'], search_space['min_top'], search_space['max_down'],
          search_space['max_down']],
         color='green', linestyle='-.', linewidth=1, alpha=0.5)


# withAda = []
# noAda = []
# x = []
# for i in range(20):
#     time1 = datetime.datetime.now()
#     algo = RRTStandard(departure, destination, obstacles, safeRadius, maxIterations=5000)
#     algo.grow_motion_path(goal_adaptive=True)
#     time2 = datetime.datetime.now()
#     cost_withAda = (time2 - time1).total_seconds() * 1000
#     withAda.append(cost_withAda)
#
#     time3 = datetime.datetime.now()
#     algo2 = RRTStandard(departure, destination, obstacles, safeRadius, maxIterations=5000)
#     algo2.grow_motion_path(goal_adaptive=False)
#     time4 = datetime.datetime.now()
#     cost_noAda = (time4 - time3).total_seconds() * 1000
#     noAda.append(cost_noAda)
#
#     x.append(i+1)
#
# plt.plot(x, withAda, c='red')
# plt.plot(x, noAda, c='blue')


#
time1 = datetime.datetime.now()
algo = RRTStandard(departure, destination, obstacles, safeRadius, maxIterations=5000)
algo.NOS = 1
algo.grow_motion_path(goal_adaptive=True)
time2 = datetime.datetime.now()
print(f"time1 = {time1}, time2 = {time2}")
print(f"the time cost on searching a path = {(time2 - time1).total_seconds() * 1000}")
randomTree = algo.randomTree


def visualize_tree(treeNodes):
    for node in treeNodes:
        plt.scatter(node.locationX, node.locationY, s=8, c='green')
        if node.parent is not None:
            nodes_x = [node.parent.locationX, node.locationX]
            nodes_y = [node.parent.locationY, node.locationY]
            plt.plot(nodes_x, nodes_y, c='green')
        else:  # destination node
            for parent in node.parents:
                nodes_x = [parent.locationX, node.locationX]
                nodes_y = [parent.locationY, node.locationY]
                plt.plot(nodes_x, nodes_y, c='green')


def visualize_paths(paths, color, alpha=1):
    for solutionPath in paths:
        solution_x = np.array([])
        solution_y = np.array([])
        for node in solutionPath:
            plt.scatter(node.locationX, node.locationY, s=8, c=color, alpha=alpha)
            solution_x = np.append(solution_x, node.locationX)
            solution_y = np.append(solution_y, node.locationY)

        plt.plot(solution_x, solution_y, c=color, alpha=alpha)


visualize_tree(algo.treeNodes)
solutionsAll = algo.get_solution_paths()

visualize_paths(solutionsAll, 'blue')

# time3 = datetime.datetime.now()
# opp = OptimizePath(departure, destination, obstacles, safeRadius)
# optimized_solutions = opp.optimize_solutions(solutionsAll)
# time4 = datetime.datetime.now()
# print(f"the time cost on optimization path is {(time4 - time3).total_seconds() * 1000}")
#
# visualize_paths(optimized_solutions, 'red')
# time5 = datetime.datetime.now()
# print(f"the time cost on visualization path is {(time5 - time4).total_seconds() * 1000}")

ax.set_xlabel("Xm")
ax.set_ylabel("Ym")
plt.axis('equal')
# fig.align_labels()
plt.grid(True)
plt.show()
plt.close()
