<div align="center">
  <h1>The Standard RRT Algorithm</h1>
</div>
</br>

## Introduction
The Rapid Exploration Random Tree (RRT) algorithm is a random sampling algorithm for state space, which avoids the large computational burden caused by precise modeling of space by detecting collisions at sampling points. It can effectively solve path-planning problems in high-dimensional spaces and complex constraints. This method is probability complete and non-optimal. It can easily handle obstacles and differential constraints and is widely used in the area of autonomous robot path planning.

In this project, I aimed to apply the standard RRT algorithm to solve the obstacle avoidance problem for robots in given environments, simulate the standard RRT algorithm and observe the performance, and then optimize the standard RRT algorithm with an adaptive lead point method and compare the performance between them in different environments.

## Definition of Key Parameters

<ul>
  <li><b>safeRadius</b>: </br>
    Because the robot has its size, we need to define a safe radius to represent the size.</li>
  <li><b>stepSize</b>: </br>In each iteration, the growth length of the tree.</li>
  <li><b>targetRadius</b>: </br>When a tree is growing and the distance between one node and the destination point is less than the value of targetRadius, we believe that the RRT tree has discovered a feasible path.</li>
</ul>
