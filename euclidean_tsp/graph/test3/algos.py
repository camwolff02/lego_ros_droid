"""
Implementation of explore, dfs, and custom findbridges algorithm
to search a graph for bridges. A bridge is an edge that, if removed,
would bisect the graph into multiple graphs.
"""
from typing import TypeVar, Hashable
import copy

from graph import Graph

V = TypeVar("V", bound=Hashable)


def EXPLORE(G: Graph, v: V, explored: set[V]) -> set[V]:
    """Finds a spanning tree rooted at V
    :param G: A graph where every vertex is either "explored" or "unexplored"
    :param v: A vertex to start exploration from
    :param explored: The set of all explored vertices
    :return: The previously "unexplored: vertices reachable from V
    """
    explored.add(v)
    S: set[V] = {v}
    for u in G.neighbors(v):
        if u not in explored:
            S = S.union(EXPLORE(G, u, explored))
    return S


def DFS(G: Graph) -> list[set[V]]:
    """Finds the spanning forest of the graph G
    :param G: A graph
    :return: the components of G
    """
    S: list[set[V]] = []
    explored: set[V] = set()
    for v in G.vertices:
        if v not in explored:
            S.append(EXPLORE(G, v, explored))
    return S


def FINDBRIDGES(G: Graph) -> set[tuple[V, V]]:
    """Given a connected graph G = (V,E), an edge e = (u,v) is a bridge iff 
    G' = (V,E-{e}) is not connected. That is to say, it is an edge whose
    removal disconnectes the graph. FINDBRIDGES finds all such edges.
    complexity: O(|E|^2), Quadratic

    :param G: A conected graph
    :return: All the Bridges of G
    """
    bridges: set[tuple[V, V]] = set()

    for edge in copy.deepcopy(G.edges):  # O(|E|)
        G.remove_edge(edge)  # O(1)
        spanning_forest: list[set[V]] = DFS(G)  # O(|V| + |E| - 1)
        G.add_edge(edge)  # O(1)
        if len(spanning_forest) > 1:
            bridges.add(edge)

    return bridges
