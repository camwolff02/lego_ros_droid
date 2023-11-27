"""
Loads GNSS Waypoints from in1.txt. Constructs a weighted graph from waypoints,
with weights corresponding to their euclidean distance. Uses the TSP algorithm
to find the shortest Hamiltonian path in the graph.
"""
import sys
import math
from typing import NamedTuple

from graph.graph import Graph
from tsp import brute_force_tsp

PRECISION: int = 10**8
DEG_TO_M: float = 111_100

class Vertex(NamedTuple):
    name: str
    lat: float
    lon: float

    def __str__(self) -> str:
        return self.name


def construct_graph(filename: str) -> Graph:
    """Constructs graph from file"""
    vertices: list[Vertex] = []

    with open(filename, 'r') as file:
        for line in file.readlines():
            nums = line.split(' ')
            vertex = Vertex(nums[0],
                            int(float(nums[1]) * PRECISION),
                            int(float(nums[2]) * PRECISION))
            vertices.append(vertex)

    edges: set[tuple[Vertex, Vertex, float]] = set()
    for vertex_u in vertices:
        for vertex_v in vertices:
            weight = math.dist((vertex_u.lat, vertex_u.lon),
                               (vertex_v.lat, vertex_v.lon))
            # print(weight)
            edges.add((vertex_u, vertex_v, weight))

    # for edge in edges:
    #     print(edge)
    return Graph(edges)


def main() -> None:
    """Loads graph, runs TSP algorithm, and prints solution"""
    filename: str = sys.argv[1] if len(sys.argv) > 1 else 'in1.txt'
    graph: Graph = construct_graph(filename)

    for vertex in graph.vertices:
        if vertex.name == 'START':
            source = vertex

            length, path = brute_force_tsp(graph, source)
            print(f'Shortest path: {length / PRECISION * DEG_TO_M:.2f}m')
            for vertex in path[:-1]:
                print(f'{vertex}, ', end='')
            print(path[-1])


if __name__ == '__main__':
    main()
