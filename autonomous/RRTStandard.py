import random

import numpy as np

from autonomous.Tools import Tools
from autonomous.TreeNode import TreeNode


class RRTStandard():

    def __init__(self, departure, destination, obstacles, safeRadius, maxIterations=200):

        self.nodeCount = 1

        self.departure = departure
        self.departureNode = TreeNode(departure[0], departure[1])
        self.randomTree = self.departureNode
        self.treeNodes = [self.departureNode]  # consist of all tree nodes

        self.destination = destination
        self.destinationNode = TreeNode(destination[0], destination[1])

        self.maxIterations = min(maxIterations, 7000)

        self.obstacles = obstacles
        self.safeRadius = safeRadius
        self.stepSize = safeRadius * 1.5
        self.targetRadius = self.safeRadius * 5

        self.searchSpace = self.calculate_search_space()

        self.closestNode = None
        self.minDistance = 0  # the minimum distance between a leadpoint and nodes
        self.newPoint_closest = None

        self.NOS = 1  # Number of solutions to be produced

    def grow_motion_path(self, goal_adaptive=False):
        """
        Rapidly grow random tree, find some path solutions
        :return:
        """
        for iterate in range(self.maxIterations):
            self.get_legal_children_point(goal_adaptive)

            newChildNode = TreeNode(self.newPoint_closest[0], self.newPoint_closest[1])
            newChildNode.parent = self.closestNode
            self.closestNode.children.append(newChildNode)
            self.treeNodes.append(newChildNode)

            # if the new point can directly reach the destination, then a path was found, stop the iteration
            if self.is_reach_destination(self.newPoint_closest) is True:
                self.destinationNode.parents.append(newChildNode)

                if len(self.destinationNode.parents) >= self.NOS or iterate == self.maxIterations - 1:
                    for parent in self.destinationNode.parents:
                        parent.children.append(self.destinationNode)
                    self.treeNodes.append(self.destinationNode)
                    print(f'iteration = {iterate}')
                    break

    def get_solution_paths(self):
        """
        all possible path solutions
        :return: return a list with all possible path solutions
        """
        solution_paths = []
        for parent in self.destinationNode.parents:
            solution = [self.destinationNode]
            # solution.append(parent)
            self.get_parent_node(parent, solution)
            solution_paths.append(solution)

        return solution_paths

    def get_parent_node(self, node, solution):
        '''
        put all the nodes which have a parent node into a solution list
        :param solution:
        :param node:
        :return:
        '''
        solution.append(node)
        if node.parent is not None:
            self.get_parent_node(node.parent, solution)

    def is_reach_destination(self, newPoint):
        '''
        check if the new point can directly reach the destination
        :param newPoint:
        :return: True or False
        '''
        if Tools.is_legal_point(newPoint, self.destination, self.obstacles, self.safeRadius) is True and Tools.getDistance(newPoint, self.destination) <= self.targetRadius:
            return True
        else:
            return False

    def get_legal_children_point(self, goal_adaptive):
        '''
        recursive function util find a legal new point
        :return: null, if a legal new point was fond, it will be passed to self.newPoint
        '''
        leadPoint = self.lead_point(goal_adaptive)
        self.minDistance = 0
        self.find_closest_node(self.randomTree, leadPoint)

        newPoint_closest = self.get_children_point(leadPoint, self.closestNode)
        isLegal = Tools.is_legal_point((self.closestNode.locationX, self.closestNode.locationY), newPoint_closest,
                                       self.obstacles, self.safeRadius)
        if isLegal is True:
            # avoid find more local optimal possible solution
            for parent in self.destinationNode.parents:
                if self.closestNode.locationX == parent.locationX and self.closestNode.locationY == parent.locationY:
                    isLegal = False
                    break

        if isLegal is not True:
            self.get_legal_children_point(goal_adaptive)
        else:
            self.newPoint_closest = newPoint_closest

    def get_children_point(self, leadPoint, closestNode):
        '''
        get a next child location
        :return: a child location(x, y)
        '''
        vectorStart = (closestNode.locationX, closestNode.locationY)
        vectorEnd = leadPoint

        return Tools.find_point_on_vector(vectorStart, vectorEnd, self.stepSize)

    def find_closest_node(self, root, leadPoint):
        """
        recursive function, find a closest node to the lead point.
        :param root:
        :param leadPoint:
        :return:
        """
        distance = Tools.getDistance((root.locationX, root.locationY), leadPoint)
        if self.minDistance == 0 or self.minDistance > distance:  # find the closest node
            self.minDistance = distance
            self.closestNode = root

        for child in root.children:
            self.find_closest_node(child, leadPoint)

    def calculate_search_space(self):
        """
        calculate the search space
        :param self:
        :return: search space = {
                                    'max_left':
                                    'min_right':
                                    'min_top':
                                    'max_down':
                                }
        """
        return Tools.get_search_space(self.obstacles, self.departure, self.destination, self.safeRadius)

    def lead_point(self, goal_adaptive):
        """
        a lead point determin the direction of a growing new node.
        :param self:
        :return: a lead point
        """
        if goal_adaptive is True:
            rand = random.random()
            if rand < 0.5:
                x, y = self.destination
            else:
                x = np.random.uniform(self.searchSpace['min_right'], self.searchSpace['max_left'])
                y = np.random.uniform(self.searchSpace['max_down'], self.searchSpace['min_top'])
        else:
            x = np.random.uniform(self.searchSpace['min_right'], self.searchSpace['max_left'])
            y = np.random.uniform(self.searchSpace['max_down'], self.searchSpace['min_top'])

        return (x, y)
