"""
Implementation of Hierholzer's Algorithm for finding eulerian paths
"""
import random
from typing import Hashable, Optional, TypeVar

from graph import Graph

V = TypeVar('V', bound=Hashable)


def find_s_and_t(G: Graph, numeric=True) -> dict[str, list[V]]:
    """ returns candidates for starting and target locations """
    same_in_and_out = True
    candidates: dict[str, list[V]] = {'s': [], 't': []}

    for u in G.vertices:
        if G.in_deg(u) != G.out_deg(u):
            same_in_and_out = False
        if G.in_deg(u) == G.out_deg(u) - 1:
            candidates['s'].append(u)
        if G.in_deg(u) == G.out_deg(u) + 1:
            candidates['t'].append(u)

    if same_in_and_out:
        vert = tuple(G.vertices)
        u = min(vert) if numeric else random.choice(vert)
        candidates['s'] = [u]
        candidates['t'] = [u]

    return candidates


def find_eulerian_path(G: Graph, s: V, t: Optional[V] = None) -> list[V]:
    """Given a a simple, weakly connected graph, finds a Eulerian 
    paths, if any. Note that the Eulerian path may be a Eulerian cycle
    if it starts and ends at the same vertex.

    :param G: A simple, weakly connected directed graph.
    :param s: The starting vertex for the path
    :param t: Optional for debugging, the ending vertex for the path
    :return: A sequence of edges representing the Eulerian path.
    """
    if G.deg(s) == 0:  # deg(s) = in_deg(s) + out_deg(s)
        return [s]

    else:
        # remove any edge (s, v) from G
        v = random.choice(tuple(G.successors(s).keys()))
        G.remove_edge((s, v))
        
        P: list = find_eulerian_path(G, v, t)
        if G.deg(s) == 0:  # deg(s) = in_deg(s) + out_deg(s)
            return [s] + P
        else:
            return find_eulerian_path(G, s, s) + P
