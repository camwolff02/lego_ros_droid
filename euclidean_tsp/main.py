"""
Loads GNSS Waypoints from filename command line argument. Constructs a
weighted graph from waypoints, with weights corresponding to their euclidean
distance. Uses various lgorithms to find the shortest Hamiltonian path in the
graph, or an approximation. Benchmarks all algorithms and prints results
"""
from collections import OrderedDict
import math
import sys
import time
from typing import NamedTuple

from graph.graph import Graph
from tsp import (
    brute_force_tsp,
    parallel_naive_tsp,
    approx_min_path
)

ENABLE_ALGOS: tuple[bool, ...] = (True, True, True)

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

    edges: list[tuple[Vertex, Vertex, float]] = []

    for vertex_u in vertices:
        for vertex_v in vertices:
            if vertex_u != vertex_v:
                weight = math.dist((vertex_u.lat, vertex_u.lon),
                                   (vertex_v.lat, vertex_v.lon))
                edges.append((vertex_u, vertex_v, weight))

    return Graph(edges, weighted=True)


def main() -> None:
    """Loads graph, runs and benchmarks all algorithms"""
    filename: str = sys.argv[1] if len(sys.argv) > 1 else 'in1.txt'
    graph: Graph = construct_graph(filename)

    for vertex in graph.vertices:
        if vertex.name == 'START':
            source = vertex

            algorithms = OrderedDict([
                ('Brute Force', brute_force_tsp),
                ('Approximation', approx_min_path),
                ('Parallelization', parallel_naive_tsp)
            ])

            times: list[float] = [0]*len(algorithms)
            min_name: str = 'n/a'
            min_time: float = math.inf

            # Benchmarking
            for i, (enable, name) in enumerate(zip(ENABLE_ALGOS, algorithms)):
                if enable:
                    print(f'{name} algorithm')
                    start = time.perf_counter()
                    length, path = brute_force_tsp(graph, source)
                    end = time.perf_counter()

                    print(f'path: {length/PRECISION * DEG_TO_M:.4f}m')
                    for vertex in path[:-1]:
                        print(f'{vertex}, ', end='')
                    print(path[-1])
                    print(f'Elapsed time: {(time_elapsed := end-start)}s\n')
                    times[i] = time_elapsed

                    if time_elapsed < min_time:
                        min_name = name
                        min_time = time_elapsed

            print(f'{min_name} fastest at {min_time}s')


if __name__ == '__main__':
    main()
