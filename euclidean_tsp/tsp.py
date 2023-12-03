"""
Algorithm for finding the shortest Hamiltonian Path in a weighted graph
"""
import itertools
import math
from typing import TypeVar, Hashable

from scipy.cluster.hierarchy import DisjointSet  # type: ignore

from graph.graph import Graph


V = TypeVar('V', bound=Hashable)


def brute_force_tsp(graph: Graph, source: V) -> tuple[float, list[V]]:
    """Brute force solution for finding the shortest Hamiltonian path in a
    weighted graph. Practical for graphs with |V| <= 11. Runs in O(n!)

    :param graph: An undirected, weighted graph
    :param source: The starting vertex for the Hamiltonian path
    :return: The weight of the optimal path, and the path itself
    """

    # push all nodes except source into bundle
    nodes: list[V] = [node for node in graph.vertices if node != source]

    shortest_weight: float = math.inf
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

        if path_weight < shortest_weight:
            shortest_weight = path_weight
            path = path_

    return shortest_weight, path


def approx_min_path(graph: Graph, source: V, find_weight: bool = True
                    ) -> tuple[float, list[V]]:
    """Returns a 2-approximation of the minimum Hamiltonian Path

    :param graph: A complete, metrically weighted graph G=(V,E)
    :param source: The starting vertex for the Hamiltonian Path in G,
                   where v is in V
    :param find_weight: Whether or not the weight of the path should be found
    :return: A Hamiltonian Path in G , and the weight of path
             if find_weight is false, the weight will be 0. Finding the weight
             is O(|V|)
    """
    # let T be MinSpanningTree(G)
    tree: list[tuple[V, V]] = min_spanning_tree(graph)
    explored: set[V] = set()  # for all v in V, let v be "unexplored"

    # let s be the source vertex in V, and P be the vertices of Explore(T, s),
    # sorted in ascending order by previsit number
    # for edge in tree:
    #     print(edge[0][0], edge[1][0])
    path: list[V] = explore(Graph(tree), source, explored)

    # If necessary, finds the weight of the Hamiltonian path
    weight = 0.0
    if find_weight:
        for idx in range(len(path)-1):
            if (path[idx], path[idx+1]) in graph.edges:
                weight += graph.weight((path[idx], path[idx+1]))
            else:
                weight += graph.weight((path[idx+1], path[idx]))

    return weight, path


def min_spanning_tree(graph: Graph) -> list[tuple[V, V]]:
    """Implementation of Kruskal's Algorithm
    Finds the minimum spanning tree of G=(V,E)

    :param graph: A connected, weighted graph G
    :return: A minimum spanning tree of G
    """
    # let T be an empty tree and S be an empty dijoint set
    tree: list[tuple[V, V]] = []
    disj_set = DisjointSet()

    for v in graph.vertices:  # for all v in V do
        disj_set.add(v)  # make a new subset containing v in S

    # let A be E, sorted in increasing order by weight
    edges: list[tuple[V, V, float]] = [
        (key[0], key[1], val) for key, val in graph.edges.items()
    ]
    ascending: list[tuple[V, V, float]] = sorted(edges, key=lambda x: x[2])
    # NOTE: ERROR Too many edges
    # for x in ascending:
    #     print(f'({x[0][0]}, {x[1][0]}, {x[2]})', end=', ')

    for u, v, _ in ascending:  # for all e=(u,v) in A do
        # The cut property, if e is the edge of strictly minimum weight in the
        # cut set, then e must be in every MST
        if disj_set.connected(u, v):  # if u and v in different subsets in S
            disj_set.merge(u, v)  # Union the subsets containing u and v in S
            tree.append((u, v))  # Add e to T

        # (Implicit) The Cycle Property, if e is the edge of strictly maximum
        # weight, then e cannot be in any MST

    return tree  # return T


def explore(graph: Graph, v: V, explored: set[V]) -> list[V]:
    """Finds a spanning tree rooted at V

    :param graph: A graph with every vertex either "explored" or "unexplored"
    :param v: A vertex to start exploration from
    :param explored: The set of all explored vertices
    :return: The previously "unexplored" vertices reachable from V, sorted by
             previsit number
    """
    explored.add(v)  # a tree vertex with no children
    spanning_tree: list[V] = [v]
    for u in graph.neighbors(v):
        if u not in explored:
            # EXPLORE(G,u) is a subtree of v
            spanning_tree.extend(explore(graph, u, explored))

    return spanning_tree  # a tree rooted at v
