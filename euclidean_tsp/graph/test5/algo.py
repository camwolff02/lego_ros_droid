from graph import Graph
import sys
from typing import TypeVar, Hashable

V = TypeVar('V', bound=Hashable)


def k_cores(G: Graph) -> list[set[V]]:
    # 1. Find the max degree
    max_deg = -1
    for v in G.vertices:
        max_deg = max(max_deg, G.deg(v))

    # 2. create a list of all the possible cores
    cores: list[set[V]] = [set() for _ in range(max_deg+1)]

    # Repeat until all vertices are handled
    while len(G.vertices) > 1:  
        # 3. Find the set of vertices I need to prune
        min_deg = sys.maxsize
        prune_set = set()
        for v in G.vertices: 
            deg = G.deg(v)
            if deg < min_deg:
                min_deg = deg
                prune_set = {v}
            elif deg == min_deg:
                prune_set.add(v)

        # 3. Prune the vertices of minimum degree, and add to all cores
        while len(prune_set) > 0: 
            vertex = prune_set.pop()
            # NOTE: Siu's suggestion: if removing a vertex would create another 
            # vertex we need to prune, add that vertex to the prune set
            for neighbor in G.neighbors(vertex):
                if G.deg(neighbor) == min_deg+1:
                    prune_set.add(neighbor)
            # Greedy choice: any vertex with degree k cannot be in a k+1 core
            G.remove_vertex(vertex) 
            # Greedy choice: Any vertex in a k-core must also be in k-1 core
            for i in range(min_deg+1):
                cores[i].add(vertex)

    # prune cores with no vertices
    for i in range(len(cores)-1, 0, -1):
        if len(cores[i]) == 0:
            cores.pop()
        else:
            break

    return cores
