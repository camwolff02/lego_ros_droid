"""
Loads GNSS Waypoints from filename command line argument. Constructs a
weighted graph from waypoints, with weights corresponding to their euclidean
distance. Uses the TSP Algorithm to find the shortest Hamiltonian path in the
graph. Uses Kruskal's Algorithm to find a 2-approximation to the shortest
Hamiltonian path. Benchmarks both algorithms and prints results
"""
import math
import sys
import time
from typing import NamedTuple

from graph.graph import Graph
from tsp import brute_force_tsp, approx_min_path

PRECISION: int = 10**8
DEG_TO_M: float = 111_100

ENABLE_BRUTE_FORCE: bool = True
ENABLE_APPROXIMATION: bool = False


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
            edges.add((vertex_u, vertex_v, weight))

    return Graph(edges)


def main() -> None:
    """Loads graph, runs and benchmarks both algorithms"""
    filename: str = sys.argv[1] if len(sys.argv) > 1 else 'in1.txt'
    graph: Graph = construct_graph(filename)

    for vertex in graph.vertices:
        if vertex.name == 'START':
            source = vertex

            # Benchmark brute force algorithm
            if ENABLE_BRUTE_FORCE:
                print('Brute force algorithm')
                start1 = time.perf_counter()
                length, path = brute_force_tsp(graph, source)
                end1 = time.perf_counter()

                print(f'Shortest path: {length/PRECISION * DEG_TO_M:.4f}m')
                for vertex in path[:-1]:
                    print(f'{vertex}, ', end='')
                print(path[-1])
                print(f'Elapsed time: {(time1 := end1-start1)}s')

            # Benchmark approximation
            if ENABLE_APPROXIMATION:
                print('\nApproximation algorithm')
                start2 = time.perf_counter()
                length, path = approx_min_path(graph, source)
                end2 = time.perf_counter()

                print(f'Approximage path: {length/PRECISION * DEG_TO_M:.4f}m')
                for vertex in path[:-1]:
                    print(f'{vertex}, ', end='')
                print(path[-1])
                print(f'Elapsed time: {(time2 := end2-start2)}s')

            if ENABLE_APPROXIMATION and ENABLE_BRUTE_FORCE:
                print()
                if time1 > time2:
                    print(f'Brute force faster by {time1-time2}s')
                else:
                    print(f'Approximation force faster by {time2-time1}s')


if __name__ == '__main__':
    main()
