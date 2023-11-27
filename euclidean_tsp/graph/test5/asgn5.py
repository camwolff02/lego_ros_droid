"""
Loads an edge list from an input file, generates a Graph G from the edge
list, then finds all the bridges in G.
"""
import sys
from typing import Optional

sys.path.insert(1, '/Users/camwolff/Development/local/euclidean_tsp/graph')
from graph import Graph
# from scripts.graphviz import visualize
from tsp import k_cores


def load_graph_from_edges(filename: str,
                          directed=False,
                          weighted=False) -> Optional[Graph]:
    """Create graph from filename console argument"""
    try:
        with open(filename, 'r') as file:
            edges = set()
            while True:
                line = file.readline()
                if line == '':
                    break
                edges.add(tuple(int(x) for x in line.split(', ')))

            return Graph(edges, directed=directed, weighted=weighted)

    except OSError as err:
        print(err)
        return None


def main() -> None:
    """Main function to test code"""
    filename = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] != '' \
        else "in1.txt"
    graph = load_graph_from_edges(filename, directed=False, weighted=False)

    if graph is not None:
        sol: list[set] = k_cores(graph)
        for idx, set_ in enumerate(sol):
            if idx == 0:
                continue
            vertices = list(set_)
            vertices.sort()
            print(f'Vertices in {idx}-cores:')
            for vertex in vertices[:-1]:
                print(f'{vertex}, ', end='')
            print(vertices[-1])


if __name__ == '__main__':
    main()
