"""
Algorithm for finding the shortest Hamiltonian Path in a weighted graph
"""
import itertools
import math
import multiprocessing as mp
from typing import TypeVar, Hashable

from scipy.cluster.hierarchy import DisjointSet  # type: ignore

from graph.graph import Graph


V = TypeVar('V', bound=Hashable)


def brute_force_tsp(graph: Graph, source: V) -> tuple[float, list[V]]:
    """Brute force solution for finding the shortest Hamiltonian path in a
    weighted graph. Practical for graphs with |V| <= 11. Runs in O(|V|!)

    :param graph: An undirected, weighted graph
    :param source: The starting vertex for the Hamiltonian path
    :return: The weight of the optimal path, and the path itself
    """

    # push all nodes except source into bundle
    nodes: list[V] = [node for node in graph.vertices if node != source]

    smallest_weight: float = math.inf
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

        if path_weight < smallest_weight:
            smallest_weight = path_weight
            path = path_

    return smallest_weight, path


def calc_path_weight(nodes: tuple[V, ...], source: V, graph: Graph
                     ) -> tuple[float, list[V]]:
    """Calculates the weight of a path naively

    :param nodes: nodes of the path, minus the source
    :param source: starting vertex of path
    :param graph: graph where path exists
    :return: the weight of the path, and the path itself
    """
    path_weight: float = 0.0

    node_j: V = source
    path = [node_j]
    for node_i in nodes:
        path_weight += graph.weight((node_i, node_j))
        node_j = node_i
        path.append(node_j)

    return path_weight, path


def parallel_naive_tsp(graph: Graph, source: V, num_processes: int = 5
                       ) -> tuple[float, list[V]]:
    """Parallelized naive solution for finding the shortest Hamiltonian path in
    a weighted graph. Practical for graphs with |V| <= 11. Runs in O(|V|!)

    :param graph: An undirected, weighted graph
    :param source: The starting vertex for the Hamiltonian path
    :return: The weight of the optimal path, and the path itself
    """

    # push all nodes except source into bundle
    nodes: list[V] = [node for node in graph.vertices if node != source]

    smallest_weight: float = math.inf
    shortest_path = []

    # generating permutations and sorting into buckets of jobs
    buckets: list[list[tuple[V, ...]]] = [
        [] for _ in range(num_processes)]

    i: int = -1
    for nodes_ in itertools.permutations(nodes):
        buckets[(i := i+1) % num_processes].append(nodes_)

    # run all jobs to calculate weights and paths
    results: mp.Queue[tuple[float, list[V]]] = mp.Queue()
    processes: list[mp.Process] = []

    for jobs in buckets:
        def run_jobs(jobs: list[tuple[V, ...]], results: mp.Queue):
            for job in jobs:
                weight, path_ = calc_path_weight(job, source, graph)
                results.put((weight, path_))

        p = mp.Process(target=run_jobs, args=(jobs, results))
        p.start()

    for p in processes:
        p.join()

    # find the minimum weight of all the paths
    while not results.empty():
        weight, path = results.get()
        if weight < smallest_weight:
            smallest_weight = weight
            shortest_path = path

    return smallest_weight, shortest_path


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

    # perform re-rooting of MST, to root tree at source
    mst_graph: Graph = Graph(tree)

    # let s be the source vertex in V, and P be the vertices of Explore(T, s),
    # sorted in ascending order by previsit number
    path: list[V] = explore(mst_graph, source, explored)

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
    O(|E|log(|E|))

    :param graph: A connected, weighted graph G
    :return: A minimum spanning tree of G
    """
    # let T be an empty tree and S be an empty dijoint set
    tree: list[tuple[V, V]] = []
    disj_set: DisjointSet[set[V]] = DisjointSet()

    for v in graph.vertices:  # for all v in V do
        disj_set.add(v)  # make a new subset containing v in S

    # let A be E, sorted in increasing order by weight
    edges: list[tuple[frozenset[V], float]] = sorted([
        (frozenset(key), val) for key, val in graph.edges.items()
    ], key=lambda x: x[1])

    for edge, _ in edges:  # for all e=(u,v) in A do
        u, v = edge

        # The cut property, if e is the edge of strictly minimum weight in the
        # cut set, then e must be in every MST
        if not disj_set.connected(u, v):  # if u and v are in different subsets
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
