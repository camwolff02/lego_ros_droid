"""
Loads an edge list from an input file, generates a Graph G from the edge
list, then finds all the bridges in G.
"""
import sys
# from graphviz import visualize
from typing import Optional

sys.path.insert(1, '/Users/camwolff/Development/workspaces/test_ws/src/lego_ros_droid/euclidean_tsp/graph/')
# sys.path.insert(1, '/home/cam/Development/ros-workspaces/ros2_ws/src/lego_ros_droid/euclidean_tsp/graph/')
from algos import FINDBRIDGES
from graph import Graph



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


def main():
    """Main function to test code"""
    filename = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] != '' \
        else "in1.txt"
    graph = load_graph_from_edges(filename, directed=False, weighted=False)

    if graph is not None:
        bridges = FINDBRIDGES(graph)

        if len(bridges) == 0:
            print('Contains no bridges.')
        else:
            print(f'Contains {len(bridges)} bridge(s):')
            bridges = [(min(bridge), max(bridge)) for bridge in bridges]
            bridges.sort()
            for bridge in bridges:
                print(f'{bridge[0]}, {bridge[1]}')

        # visualize(graph)


if __name__ == '__main__':
    main()
