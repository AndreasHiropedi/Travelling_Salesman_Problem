import math
import graph
import random
import itertools as itr


# TESTING FUNCTIONALITY OF PART A AND B ALGORITHMS

def tourValueTest():
    g1 = graph.Graph(12, "twelvenodes")
    assert(g1.tourValue() == 34)
    print("Method tourValue works for twelvenodes file")
    g2 = graph.Graph(6, "sixnodes")
    assert(g2.tourValue() == 9)
    print("Method tourValue works for sixnodes file")
    g3 = graph.Graph(-1, "cities50")
    assert(g3.tourValue() == 11842.557992162187)
    print("Method tourValue works for cities50 file")


def swapHeuristicTest():
    g1 = graph.Graph(12, "twelvenodes")
    g1.swapHeuristic(12)
    assert(g1.tourValue() == 32)
    print("Method swap works for twelvenodes file")
    g2 = graph.Graph(6, "sixnodes")
    g2.swapHeuristic(6)
    assert(g2.tourValue() == 9)
    print("Method swap works for sixnodes file")
    g3 = graph.Graph(-1, "cities50")
    g3.swapHeuristic(-1)
    assert(g3.tourValue() == 8707.056392932445)
    print("Method swap works for cities50 file")


def twoOptHeuristicTest():
    g1 = graph.Graph(12, "twelvenodes")
    g1.swapHeuristic(12)
    g1.TwoOptHeuristic(12)
    assert(g1.tourValue() == 26)
    print("Method two-opt works for twelvenodes file")
    g2 = graph.Graph(6, "sixnodes")
    g2.swapHeuristic(6)
    g2.TwoOptHeuristic(6)
    assert(g2.tourValue() == 9)
    print("Method two-opt works for sixnodes file")
    g3 = graph.Graph(-1, "cities50")
    g3.swapHeuristic(-1)
    g3.TwoOptHeuristic(-1)
    assert(g3.tourValue() == 2720.4534972261745)
    print("Method two-opt works for cities50 file")


def greedyTest():
    g1 = graph.Graph(12, "twelvenodes")
    g1.Greedy()
    assert(g1.tourValue() == 26)
    print("Method greedy works for twelvenodes file")
    g2 = graph.Graph(6, "sixnodes")
    g2.Greedy()
    assert(g2.tourValue() == 8)
    print("Method greedy works for sixnodes file")
    g3 = graph.Graph(-1, "cities50")
    g3.Greedy()
    assert(g3.tourValue() == 3011.5931919734803)
    print("Method greedy works for cities50 file")


def mainTester():
    tourValueTest()
    swapHeuristicTest()
    twoOptHeuristicTest()
    greedyTest()


# BEGINNING OF THE EXPERIMENTS

# euclidian TSPs do not have any weights in the file
# since the weights are calculated using the euclidian distance between any two points
def euclidian(nodes, graph_no):
    filename = "euclidian" + str(graph_no)
    # create the actual input files
    with open(filename, 'w') as file:
        for _ in range(nodes):
            # we will use values in the range 0-40 to make the calculations for optimal solution easier
            x_val = random.randint(0, 40)
            y_val = random.randint(0, 40)
            line = str(x_val) + " " + str(y_val)
            file.write(line)
            file.write("\n")
    # output the tour value after running each one of the algorithms on the given graph
    g_euclid = graph.Graph(-1, filename)
    print("Normal tour value: ", g_euclid.tourValue())
    g_euclid.swapHeuristic(-1)
    print("Swap heuristic tour value: ", g_euclid.tourValue())
    g_euclid = graph.Graph(-1, filename)
    g_euclid.TwoOptHeuristic(-1)
    print("Two-opt heuristic tour value: ", g_euclid.tourValue())
    g_euclid = graph.Graph(-1, filename)
    g_euclid.swapHeuristic(-1)
    g_euclid.TwoOptHeuristic(-1)
    print("Both heuristics tour value: ", g_euclid.tourValue())
    g_euclid = graph.Graph(-1, filename)
    g_euclid.Greedy()
    print("Greedy tour value: ", g_euclid.tourValue())
    g_euclid = graph.Graph(-1, filename)
    g_euclid.ownSolverTSP(-1)
    print("Own algorithm tour value: ", g_euclid.tourValue())
    print("Optimal solution: ", optimalSol(g_euclid))


def metric(nodes, weights, graph_no):
    filename = "metric" + str(graph_no)
    with open(filename, 'w') as file:
        index = 0
        for i in range(nodes):
            for j in range(i+1, nodes):
                line = str(i) + " " + str(j) + " " + str(weights[index])
                index = index + 1
                file.write(line)
                file.write("\n")
    # output the tour value after running each one of the algorithms on the given graph
    g_metric = graph.Graph(nodes, filename)
    print("Normal tour value: ", g_metric.tourValue())
    g_metric.swapHeuristic(nodes)
    print("Swap heuristic tour value: ", g_metric.tourValue())
    g_metric = graph.Graph(nodes, filename)
    g_metric.TwoOptHeuristic(nodes)
    print("Two-opt heuristic tour value: ", g_metric.tourValue())
    g_metric = graph.Graph(nodes, filename)
    g_metric.swapHeuristic(nodes)
    g_metric.TwoOptHeuristic(nodes)
    print("Both heuristics tour value: ", g_metric.tourValue())
    g_metric = graph.Graph(nodes, filename)
    g_metric.Greedy()
    print("Greedy tour value: ", g_metric.tourValue())
    g_metric = graph.Graph(nodes, filename)
    g_metric.ownSolverTSP(nodes)
    print("Own algorithm tour value: ", g_metric.tourValue())
    print("Optimal solution: ", optimalSol(g_metric))


def nonMetric(nodes, weights, graph_no):
    filename = "non-metric" + str(graph_no)
    with open(filename, 'w') as file:
        index = 0
        for i in range(nodes):
            for j in range(i+1, nodes):
                line = str(i) + " " + str(j) + " " + str(weights[index])
                index = index + 1
                file.write(line)
                file.write("\n")
    # output the tour value after running each one of the algorithms on the given graph
    g_non_metric = graph.Graph(nodes, filename)
    print("Normal tour value: ", g_non_metric.tourValue())
    g_non_metric.swapHeuristic(nodes)
    print("Swap heuristic tour value: ", g_non_metric.tourValue())
    g_non_metric = graph.Graph(nodes, filename)
    g_non_metric.TwoOptHeuristic(nodes)
    print("Two-opt heuristic tour value: ", g_non_metric.tourValue())
    g_non_metric = graph.Graph(nodes, filename)
    g_non_metric.swapHeuristic(nodes)
    g_non_metric.TwoOptHeuristic(nodes)
    print("Both heuristics tour value: ", g_non_metric.tourValue())
    g_non_metric = graph.Graph(nodes, filename)
    g_non_metric.Greedy()
    print("Greedy tour value: ", g_non_metric.tourValue())
    g_non_metric = graph.Graph(nodes, filename)
    g_non_metric.ownSolverTSP(nodes)
    print("Own algorithm tour value: ", g_non_metric.tourValue())
    print("Optimal solution: ", optimalSol(g_non_metric))


# TESTER METHODS FOR GENERATING EXPERIMENTS


def mainEuclidianTester():
    # both graphs will be smaller than the cities50 example
    nodes1 = random.randint(5, 10)
    print("--- RESULTS FOR FIRST EUCLIDIAN GRAPH ---")
    print("Number of nodes: ", nodes1)
    euclidian(nodes1, 1)
    print("--- END OF RESULTS FOR FIRST EUCLIDIAN GRAPH ---")
    print(" ")
    nodes2 = random.randint(5, 10)
    print("--- RESULTS FOR SECOND EUCLIDIAN GRAPH ---")
    print("Number of nodes: ", nodes2)
    euclidian(nodes2, 2)
    print("--- END OF RESULTS FOR SECOND EUCLIDIAN GRAPH ---")
    print(" ")


def mainMetricTester():
    # both graphs will have a size between the sixnodes and twelvenodes examples
    nodes1 = random.randint(7, 11)
    weights1 = []
    total_no_weights1 = round(nodes1 * (nodes1 - 1) / 2)
    for k in range(total_no_weights1):
        # we will keep all weights in the range 2-7 to make calculations easier
        weight = random.randint(2, 7)
        weights1.append(weight)
    weights1 = checkTriangle(weights1)
    print("--- RESULTS FOR FIRST METRIC GRAPH ---")
    print("Number of nodes: ", nodes1)
    print("Weights used: ", weights1)
    metric(nodes1, weights1, 1)
    print("--- END OF RESULTS FOR FIRST METRIC GRAPH ---")
    print(" ")
    nodes2 = random.randint(7, 11)
    weights2 = []
    total_no_weights2 = round(nodes2 * (nodes2 - 1) / 2)
    for k in range(total_no_weights2):
        # we will keep all weights in the range 2-7 to make calculations easier
        weight = random.randint(2, 7)
        weights2.append(weight)
    weights2 = checkTriangle(weights2)
    print("--- RESULTS FOR SECOND METRIC GRAPH ---")
    print("Number of nodes: ", nodes2)
    print("Weights used: ", weights2)
    metric(nodes2, weights2, 2)
    print("--- END OF RESULTS FOR SECOND METRIC GRAPH ---")
    print(" ")


def mainNonMetricTester():
    # both graphs will have a size between the sixnodes and twelvenodes examples
    nodes1 = random.randint(7, 11)
    weights1 = []
    total_no_weights1 = round(nodes1 * (nodes1 - 1) / 2)
    for k in range(total_no_weights1):
        # we will keep all weights in the range 1-5 to make calculations easier
        weight = random.randint(1, 5)
        weights1.append(weight)
    print("--- RESULTS FOR FIRST NON-METRIC GRAPH ---")
    print("Number of nodes: ", nodes1)
    print("Weights used: ", weights1)
    nonMetric(nodes1, weights1, 1)
    print("--- END OF RESULTS FOR FIRST NON-METRIC GRAPH ---")
    print(" ")
    nodes2 = random.randint(7, 11)
    weights2 = []
    total_no_weights2 = round(nodes2 * (nodes2 - 1) / 2)
    for k in range(total_no_weights2):
        # we will keep all weights in the range 1-5 to make calculations easier
        weight = random.randint(1, 5)
        weights2.append(weight)
    print("--- RESULTS FOR SECOND NON-METRIC GRAPH ---")
    print("Number of nodes: ", nodes2)
    print("Weights used: ", weights2)
    nonMetric(nodes2, weights2, 2)
    print("--- END OF RESULTS FOR SECOND NON-METRIC GRAPH ---")
    print(" ")


# CHECKER FOR TRIANGLE INEQUALITY (Only for metric graphs)


def checkTriangle(weights):
    for i in range(len(weights)-2):
        for j in range(i+2, len(weights)):
            # check each group of 3 weights
            # and ensure all possible pairings obey the triangle inequality
            while weights[i] + weights[i+1] <= weights[j]:
                weights[j] = random.randint(2, 7)
            while weights[i] + weights[j] <= weights[i+1]:
                weights[i+1] = random.randint(2, 7)
            while weights[i+1] + weights[j] <= weights[i]:
                weights[i] = random.randint(2, 7)
    return weights


# BRUTE FORCE ALGORITHM FOR FINDING OPTIMAL SOLUTION TO A GIVEN TSP


def optimalSol(g):
    optimal_solution = g.tourValue()
    all_permutations = itr.permutations(g.perm)
    for permutation in all_permutations:
        g.perm = permutation
        cost = g.tourValue()
        if cost < optimal_solution:
            optimal_solution = cost
    return optimal_solution


# MAIN METHOD THAT RUNS ALL THE EXPERIMENTS


def main():
    mainTester()
    mainEuclidianTester()
    mainMetricTester()
    mainNonMetricTester()



