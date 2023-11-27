"""
Algorithm for finding the shortest Hamiltonian Path in a weighted graph
"""
import math
import itertools
from typing import TypeVar, Hashable, Sequence

from graph.graph import Graph


V = TypeVar('V', bound=Hashable)


def brute_force_tsp(graph: Graph, source: V) -> tuple[float, Sequence[V]]:
    """Brute force solution for finding the shortest Hamiltonian path in a
    weighted graph. Practical for graphs with |V| <= 11. Runs in O(n!)

    :param graph: An undirected, weighted graph
    :param source: The starting vertex for the Hamiltonian path
    :return: The weight of the optimal path, and the path itself
    """

    # push all nodes except source into bundle
    nodes: list[V] = [node for node in graph.vertices if node != source]

    shortest_path: float = math.inf
    path = []

    # generating permutations and tracking the minimum cost
    for nodes_ in itertools.permutations(nodes):
        path_weight: float = 0.0

        node_j: V = source
        path_ = [node_j]
        for node_i in nodes_:
            path_weight += graph.weight((node_i, node_j))
            node_j = node_i
            path_.append(node_j)

        if path_weight < shortest_path:
            shortest_path = path_weight
            path = path_

    return shortest_path, path
