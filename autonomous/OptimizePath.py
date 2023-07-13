import copy

from autonomous.Tools import Tools
from autonomous.TreeNode import TreeNode


class OptimizePath():

    def __init__(self, departure, destination, obstacles, safeRadius):
        self.departure = departure
        self.destination = destination
        self.obstacles = obstacles
        self.safeRadius = safeRadius
        self.stepSize = safeRadius * 1.5

    def optimize_a_path(self, solutionPath):
        solutionPathCopy = copy.deepcopy(solutionPath)
        i = 1
        nodeIndex = len(solutionPathCopy) - i
        OptimizedSolutionPath = self.trim_a_path(nodeIndex, solutionPathCopy)

        while 1:
            nodeIndex = len(OptimizedSolutionPath) - i
            OptimizedSolutionPath = self.generate_nodes_for_path(OptimizedSolutionPath[nodeIndex], nodeIndex - 1,
                                                                 OptimizedSolutionPath)
            if nodeIndex < 3:
                break
            i += 1
            nodeIndex = len(OptimizedSolutionPath) - i
            OptimizedSolutionPath = self.trim_a_path(nodeIndex, OptimizedSolutionPath)

        return OptimizedSolutionPath

    def optimize_solutions(self, solutionPaths):
        '''
        optimize the possible solution paths
        :param solutionPaths: the solution paths which are needed to be optimized
        :return: optimized solution paths
        '''
        self.generate_nodes_for_solutions(solutionPaths)

        optimized_solutions = []
        for solutionPath in solutionPaths:
            length_before = self.calculate_path_length(solutionPath)
            OptimizedSolutionPath = self.optimize_a_path(solutionPath)
            while 1:
                length_after = self.calculate_path_length(OptimizedSolutionPath)
                if length_before == length_after:
                    break
                else:
                    length_before = length_after
                    OptimizedSolutionPath = self.optimize_a_path(OptimizedSolutionPath)
            optimized_solutions.append(OptimizedSolutionPath)
        return optimized_solutions

    def calculate_path_length(self, path):
        """
        calculate the total lenght of a path
        :param path:
        :return:
        """
        length = 0
        for i, node in enumerate(path):
            if i > len(path) - 2:
                break
            point1 = (path[i].locationX, path[i].locationY)
            point2 = (path[i + 1].locationX, path[i + 1].locationY)
            distance = Tools.getDistance(point1, point2)
            length += distance
        return length

    def trim_a_path(self, startIndex, path):
        '''
        start trime a single path
        :param startIndex: the indice of the start node
        :param path: the path which is need to be trimmed
        :return: a trimmed path
        '''
        OptimizedSolutionPath = []
        startPoint = (path[startIndex].locationX, path[startIndex].locationY)

        for j, node in enumerate(path):

            nextPoint = (node.locationX, node.locationY)
            if Tools.is_legal_point(startPoint, nextPoint, self.obstacles, self.safeRadius):
                path[startIndex].children.remove(path[startIndex - 1])
                path[startIndex].children.append(node)
                if node.locationX == self.destination[0] and node.locationY == self.destination[1]:
                    node.parents.append(path[startIndex])
                else:
                    node.parent = path[startIndex]

                OptimizedSolutionPath.append(node)
                OptimizedSolutionPath.append(path[startIndex])

                for i in range(len(path) - startIndex - 1):
                    OptimizedSolutionPath.append(path[startIndex + i + 1])
                break
            else:
                OptimizedSolutionPath.append(node)

            if j + 3 > startIndex:
                for i in range(len(path) - startIndex + 1):
                    OptimizedSolutionPath.append(path[startIndex + i - 1])
                break

        return OptimizedSolutionPath

    def generate_nodes_for_solutions(self, solutionPaths):
        '''
        generate some nodes between the destination node and its parent node for all solutions
        :return:
        '''
        for solutionPath in solutionPaths:
            closestNode = solutionPath[1]

            self.generate_nodes_for_path(closestNode, 0, solutionPath)

    def generate_nodes_for_path(self, startNode, endNodeIndice, solutionPath):
        """
        generate some nodes between the startNode and a leadPoint in the specific path
        :param startNode:
        :param endNode:
        :param solutionPath:
        :return: solutionPath with generated nodes
        """
        endNode = solutionPath[endNodeIndice]
        leadPoint = (endNode.locationX, endNode.locationY)

        # if the distance between the two nodes is smaller than the step size, then no need to generate more nodes,
        # return solutionPath directly
        if Tools.cut(Tools.getDistance((startNode.locationX, startNode.locationY),
                                       (endNode.locationX, endNode.locationY)), 1) <= self.stepSize:
            return solutionPath

        # Dissolve the relationship
        for child in startNode.children:
            if child.locationX == endNode.locationX and child.locationY == endNode.locationY:
                startNode.children.remove(child)
                if endNode.locationX == self.destination[0] and endNode.locationY == self.destination[1]:
                    endNode.parents.remove(startNode)

        while 1:
            if Tools.getDistance((startNode.locationX, startNode.locationY),
                                 (endNode.locationX, endNode.locationY)) < self.stepSize:
                startNode.children.append(endNode)
                if endNode.locationX == self.destination[0] and endNode.locationY == self.destination[1]:
                    endNode.parents.append(startNode)
                else:
                    endNode.parent = startNode
                break

            newPoint = self.get_children_point(leadPoint, startNode)
            newChildNode = TreeNode(newPoint[0], newPoint[1])
            newChildNode.parent = startNode
            startNode.children.append(newChildNode)

            solutionPath.insert(endNodeIndice + 1, newChildNode)

            startNode = newChildNode

        return solutionPath

    def get_children_point(self, leadPoint, closestNode):
        """
        get a next child location
        :return: a child location(x, y)
        """
        vectorStart = (closestNode.locationX, closestNode.locationY)
        vectorEnd = leadPoint

        return Tools.find_point_on_vector(vectorStart, vectorEnd, self.stepSize)
