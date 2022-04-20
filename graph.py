import math
import random


def euclid(p, q):
    x = p[0]-q[0]
    y = p[1]-q[1]
    return math.sqrt(x*x+y*y)


class Graph:


    # Complete as described in the specification, taking care of two cases:
    # the -1 case, where we read points in the Euclidean plane, and
    # the n>0 case, where we read a general graph in a different format.
    # self.perm, self.dists, self.n are the key variables to be set up.
    def __init__(self, n, filename):
        # special case
        if n == -1:
            # local variables to store all x and y co-ordinates
            x_y_pairs = []
            with open(filename) as file:
                lines = file.readlines()
                # set/ initialise all key variables
                self.n = len(lines)
                self.perm = [i for i in range(self.n)]
                self.dists = []
                for line in lines:
                    # remove all spaces
                    values = line.split()
                    # and add the integer values as a tuple (x,y) to the list of (x,y) pairs
                    x_y_pairs.append( (int(values[0]), int(values[1])) )
                # loop through all pairs (x,y)
                for first_pair in x_y_pairs:
                    # for each pair (x,y)
                    distances = []
                    for second_pair in x_y_pairs:
                        # compute the euclidian distance to all other pairs (x,y)
                        distances.append(euclid(first_pair, second_pair))
                    # add these distances to the self.dists table
                    self.dists.append(distances)
        # other n>0 cases
        else:
            # set/ initialise all the key variables
            self.n = n
            self.dists = [[0] * self.n for _ in range(self.n)]
            self.perm = [i for i in range(self.n)]
            with open(filename) as file:
                lines = file.readlines()
                for i in range(len(lines)):
                    # remove all spaces
                    values = lines[i].split()
                    # add the distance value to the dists list (using the appropriate co-ordinates)
                    self.dists[int(values[0])][int(values[1])] = int(values[2])
            # for all node pairs (i,j), if i>j, update cell [i][j] to hold the same value as [j][i]
            # e.g. since the distance from 0 to 2 is the same as 2 to 0
            # store this value in both cells [0][2] and [2][0]
            for i in range(self.n):
                for j in range(self.n):
                    if j != i and self.dists[i][j] == 0 and self.dists[j][i] != 0:
                        self.dists[i][j] = self.dists[j][i]
                    elif j != i and self.dists[j][i] == 0 and self.dists[i][j] != 0:
                        self.dists[j][i] = self.dists[i][j]


    # Complete as described in the spec, to calculate the cost of the
    # current tour (as represented by self.perm).
    def tourValue(self):
        total_cost = 0
        for i in range(len(self.perm)):
            # consider the special case where the whole list has been explored
            if i == len(self.perm) - 1:
                # add the 'wraparound' edge to the total cost
                total_cost = total_cost + self.dists[self.perm[i]][self.perm[0]]
            else:
                # add the cost of the next edge to the total cost
                total_cost = total_cost + self.dists[self.perm[i]][self.perm[i+1]]
        return total_cost

    # Attempt the swap of cities i and i+1 in self.perm and commit
    # to the swap if it improves the cost of the tour.
    # Return True/False depending on success.
    def trySwap(self,i):
        # obtain the current cost
        current_cost = self.tourValue()
        # perform the potential swap
        temp = self.perm[i]
        self.perm[i] = self.perm[(i+1) % self.n]
        self.perm[(i+1) % self.n] = temp
        # compute the new cost
        new_cost = self.tourValue()
        # if the new cost doesn't cause an improvement, then reset to the original self.perm list
        if new_cost >= current_cost:
            new_temp = self.perm[i]
            self.perm[i] = self.perm[(i + 1) % self.n]
            self.perm[(i + 1) % self.n] = new_temp
            return False
        return True
        
    # Consider the effect of reversing the segment between
    # self.perm[i] and self.perm[j], and commit to the reversal
    # if it improves the tour value.
    # Return True/False depending on success.              
    def tryReverse(self, i, j):
        # obtain the current cost
        current_cost = self.tourValue()
        # perform the potential swap
        before_i = self.perm[:i]  # all elements before i
        list_to_reverse = self.perm[i:(j + 1)]  # elements to be reversed (currently in original order)
        reversed_list = list_to_reverse[::-1]  # elements to be reversed (in reversed order)
        after_j = self.perm[(j+1):]  # all elements after j
        self.perm = before_i + reversed_list + after_j  # update self.perms to contain the reversed segment
        # compute the new cost
        new_cost = self.tourValue()
        # if the new cost doesn't cause an improvement, then restore to the original self.perm list
        if new_cost >= current_cost:
            self.perm = before_i + list_to_reverse + after_j
            return False
        return True
    def swapHeuristic(self, k):
        better = True
        count = 0
        while better and (count < k or k == -1):
            better = False
            count += 1
            for i in range(self.n):
                if self.trySwap(i):
                    better = True

    def TwoOptHeuristic(self, k):
        better = True
        count = 0
        while better and (count < k or k == -1):
            better = False
            count += 1
            for j in range(self.n-1):
                for i in range(j):
                    if self.tryReverse(i, j):
                        better = True
    # Implement the Greedy heuristic which builds a tour starting
    # from node 0, taking the closest (unused) node as 'next'
    # each time.
    def Greedy(self):
        # copy the contents of self.perm
        copy_perm = [i for i in range(self.n)]
        # and remove 0 (the first node in self.perm)
        copy_perm.remove(0)
        # we begin from index 1, since we first explore node 0
        # then we explore whatever comes after in self.perm
        index = 1
        # stores the value of the current node
        curr_node = 0
        # whilst there are still nodes to explore
        while len(copy_perm) > 0:
            # compute the minimum distance between all unexplored nodes and the current node
            shortest_distance = min([self.dists[curr_node][node] for node in copy_perm])
            for node in copy_perm:
                # find the corresponding node that is closest to our current node
                # (we stop after finding the first one, as per the instructions)
                if self.dists[curr_node][node] == shortest_distance:
                    curr_node = node
                    break
            # change the order in self.perm to next explore the node closest to our initial node
            self.perm[index] = curr_node
            # update the index for the following search
            index = index + 1
            # remove the node to be explored from our copy of self.perm
            copy_perm.remove(curr_node)

    # Own implementation
    # Similar to the greedy algorithm, but with an additional look-ahead
    # Decides which node to choose based on the follow-up shortest distance
    # (the distance from the node with the shortest path to our current node,
    # which also has the shortest distance to a future node)
    # Takes in flag n as a parameter, to indicate if the graph is euclidian or not
    def ownSolverTSP(self, n):
        for start in range(self.n):
            # copy the contents of self.perm
            copy_perm = [i for i in range(self.n)]
            # and remove 0 (the first node in self.perm)
            copy_perm.remove(start)
            # we begin from index 1, since we first explore node 0
            # then we explore whatever comes after in self.perm
            index = 0
            # stores the value of the current node
            curr_node = start
            # as long as there is more than one node to be explored
            while index <= len(copy_perm):
                # compute the minimum distance between all unexplored nodes and the current node
                shortest_distance = min([self.dists[curr_node][node] for node in copy_perm])
                # list to store the nodes closest to the current node
                best_nodes = []
                # store the follow-up distances for the best nodes
                best_followup_distances = []
                for node in copy_perm:
                    # for all nodes with the shortest distance
                    if self.dists[curr_node][node] == shortest_distance:
                        best_nodes.append(node)
                        # create a new list, to look ahead
                        new_copy = [i for i in copy_perm]
                        # and remove the current node with shortest distance
                        new_copy.remove(node)
                        # compute the new follow-up shortest distance
                        shortest_path = min([self.dists[node][k] for k in new_copy])
                        # and add this follow-up distance to the list of best distances
                        best_followup_distances.append(shortest_path)
                # pick the smallest follow-up distance
                best_distance = min(best_followup_distances)
                # get its index
                diff_index = best_followup_distances.index(best_distance)
                # and find the corresponding node with that follow-up distance
                curr_node = best_nodes[diff_index]
                # update the order in self.perm
                # so that the next node to be explored is based on the follow-up shortest distance
                temp = copy_perm[index]
                temp_index = copy_perm.index(curr_node)
                copy_perm[index] = curr_node
                copy_perm[temp_index] = temp
                # update the index for the following search
                index = index + 1
            x = self.perm
            currentCost = self.tourValue()
            self.perm = copy_perm
            if self.tourValue() >= currentCost: self.perm = x
        # lastly, apply the two heuristics for further optimisation
        self.swapHeuristic(n)
        self.TwoOptHeuristic(n)
        




