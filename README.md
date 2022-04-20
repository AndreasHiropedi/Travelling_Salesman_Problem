# Travelling Salesman Problem

- Several algorithms, including my own one, to solve multiple instances of the Travelling salesman problem. 
- These instances include metric and non-metric graphs, as well as euclidian graphs.
- Some of the instances were already given (cities50, sixnodes and twelvenodes), and the others were generated from the tests.py file.
- The graph.py file contains all the algorithms one can use to solve any of the TSP instances, although the only one to give an optimal solution is the brute force algorithm inside the tests.py file (takes O(n!) time though, so it only works somewhat efficiently on small graphs of up to 10 nodes).
