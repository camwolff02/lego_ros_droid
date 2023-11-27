"""
Loads an edge list from an input file, generates a Graph G from the edge
list, then finds all the bridges in G.
"""
import sys
from typing import Optional

sys.path.insert(1, '/Users/camwolff/Development/local/euclidean_tsp/graph')
from tsp import find_eulerian_path, find_s_and_t
from graph import Graph
# from scripts.graphviz import visualize


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
    graph = load_graph_from_edges(filename, directed=True, weighted=False)

    if graph is not None:
        candidates: dict[str, list] = find_s_and_t(graph)
        s_list: list = candidates['s']
        t_list: list = candidates['t']

        if len(s_list) == 1 and len(t_list) == 1:
            ans = find_eulerian_path(graph, s_list[0], t_list[0])

            if ans[0] == ans[-1]:
                print('Directed Eulerian cycle:')
            else:
                print('Directed Eulerian path:')

            for vertex in ans[:-1]:
                print(f'{vertex}, ', end='')
            print(ans[-1])

        else:
            print('No directed Eulerian paths or cycles.')


if __name__ == '__main__':
    main()
