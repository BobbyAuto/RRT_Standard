<div align="center">
  <h1>The Standard RRT Algorithm</h1>
</div>
</br>

## Introduction
The Rapid Exploration Random Tree (RRT) algorithm is a random sampling algorithm for state space, which avoids the large computational burden caused by precise modeling of space by detecting collisions at sampling points. It can effectively solve path-planning problems in high-dimensional spaces and complex constraints. This method is probability complete and non-optimal. It can easily handle obstacles and differential constraints and is widely used in the area of autonomous robot path planning.

In this project, I aimed to apply the standard RRT algorithm to solve the obstacle avoidance problem for robots in given environments, simulate the standard RRT algorithm and observe the performance, and then optimize the standard RRT algorithm with an adaptive lead point method and compare the performance between them in different environments.

## The Process of Standard RRT
<ol>
  <li>Departure point as a seed, begins to grow branches;</li>
  <li>Randomly generate a lead point P in the search space;</li>
  <li>Find the closest point to P on the tree and mark it as C;</li>
  <li>Grow a step size in the direction of P if there are no obstacles to collision. If there is an obstacle to collision then repeat the process from steps 2-4;</li>
</ol>
<img width=80%; src="https://github.com/BobbyAuto/RRT_Standard/blob/main/images/Standard%20RRT%20Process.png"/>

## The Process of Standard RRT with an adaptive lead point
In the standard RRT algorithm, the lead point P was randomly generated, which enables the tree to explore the search space more. The strategy of this method is to set the lead point to the destination point with a 50% of possibility which enables the tree more targeted to grow towards the destination point. Therefore, theoretically, it has a higher performance of finding a feasible path to the destination.

```python
rand = random.random()
if rand < 0.5:
    x, y = self.destination
  else:
    x = np.random.uniform(self.searchSpace['min_right'], self.searchSpace['max_left'])
    y = np.random.uniform(self.searchSpace['max_down'], self.searchSpace['min_top'])
return (x, y)
```
## Definition of Key Parameters

<ul>
  <li><b>safeRadius</b>: </br>
    Because the robot has its size, we need to define a safe radius to represent the size. In this project, the value of safeRadius is 5.</li>
  <li><b>stepSize</b>: </br>In each iteration, the growth length of the tree. In this project, the value of stepSize is 1.5 times of the value of safeRadius.</li>
  <li><b>targetRadius</b>: </br>When a tree is growing and the distance between one node and the destination point is less than the value of targetRadius, we believe that the RRT tree has discovered a feasible path. In this project, the value of targetRadius is 5 times of the value of safeRadius.</li>
</ul>

## Definition of Environment-1
departure = (-380, -50) was represented by the blue dot on the below environment map</br>
destination = (400, 100) was represented by the red dot on the below environment map.

<img width=80%; src="https://github.com/BobbyAuto/RRT_Standard/blob/main/images/Environment-1.png"/>

## Experiment-RRT Standard
<img width=80%; src="https://github.com/BobbyAuto/RRT_Standard/blob/main/images/Result_Standard.png"/>

## Experiment-RRT Standard with an adaptive lead point
<img width=80%; src="https://github.com/BobbyAuto/RRT_Standard/blob/main/images/Result_Adaptive.png"/> 

## Performance Comparision
