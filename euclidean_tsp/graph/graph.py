"""
Definition for a class that represents the abstract concept of a graph G
according to graph theory
"""
import copy
from typing import Collection, Generic, Hashable, Optional, TypeVar

V = TypeVar('V', bound=Hashable)


class Graph(Generic[V]):
    """Implementation of a class that represents a graph G. Class accepts as
    "input" a set of edges which represent the graph. Graph  class allows
    for representations of G as an adjacency list of G, with optional
    weights and directionality.

    :param edges: Set of all edges of the graph; an edge is defined by the
                  two vertices it connects
    :param vertices: Optional set of all vertices, useful if graph has vertices
                     that are not connected by an edge
    :param directed: True if graph is directed, false otherwise.
    :param weighted: True if graph is weighted, false otherwise. An unweighted
                     graph has 1 set for all weights

    :attr vertices: Set of all vertices in the graph
    :attr edges: Dictionary mapping every edge in the graph to its weight

    NOTE: Instantiation is O(|V|+|E|). All other operations are O(1), unless
    otherwise specified.
    """
    _adj_list: dict[V, dict[V, float]]  # maps vertices adj. vertices w/ weight
    _vertices: set[V]  # set of all vertices
    _dir_edges: dict[tuple[V, V], float]  # map directed edges to weight
    _und_edges: dict[frozenset[V], float]  # map undirected edges to weight

    _graph_has_changed: bool  # true if we need to update string graph
    _in_degrees: dict[V, int]  # map vertices to their in-degrees

    directed: bool
    weighted: bool
    str_graph: str

    def __init__(self,
                 edges: Collection[tuple[V, V] | tuple[V, V, float]],
                 vertices: Optional[Collection[V]] = None,
                 directed: bool = False,
                 weighted: bool = False):

        # Define attributes
        self._adj_list = {}
        self._in_degrees = {}

        self._vertices = set() if vertices is None else set(vertices)
        self._dir_edges = {}
        self._und_edges = {}

        self._graph_has_changed = True

        self.directed = directed
        self.weighted = weighted

        # fill adjacency list with edges
        for edge in edges:
            self.add_edge(edge)
            self._vertices.add(edge[0])
            self._vertices.add(edge[1])

        # add vertices with no edges to adjacency list
        for vertex in self._vertices:
            if vertex not in self._adj_list:
                self._adj_list[vertex] = {}

    def __len__(self) -> int:
        """Returns the cardinality of V."""
        return len(self.vertices)

    def __str__(self) -> str:
        """Returns visual representation of graph.
        O(1) if graph has not changed, O(|V|) otherwise.
        """
        if self._graph_has_changed:
            # Generate new string representation
            str_list: list[str] = []

            for vertex, adj_vertices in self._adj_list.items():
                str_list.append(f'{vertex} -> ')
                for child_vertex, weight in adj_vertices.items():
                    if self.weighted:
                        str_list.append(f'{child_vertex}[{weight}], ')
                    else:
                        str_list.append(f'{child_vertex}, ')

                str_list[-1] = str_list[-1][:-2]
                str_list.append('\n')

            self.str_graph = ''.join(str_list)
            self._graph_has_changed = False

        return self.str_graph

    @property
    def vertices(self) -> set[V]:
        """Returns the set of all vertices in the graph."""
        return self._vertices

    @property
    def edges(self) -> dict[tuple[V, V], float] | dict[frozenset[V], float]:
        """Returns a dictionary of all edges mapped to their weights.
        Dictionary keys are tuples if directed, and frozenset if undirected."""
        if self.directed:
            return self._dir_edges
        else:
            return self._und_edges

    def add_edge(self, edge: frozenset[V] | tuple[V, V] | tuple[V, V, float]
                 ) -> None:
        """Adds the given edge to the graph. An edge is a tuple with length 2
        (unweighted), or a tuple with length 3 (weighted)."""
        self._graph_has_changed = True
        vertex_u: V = (edge_ := tuple(edge))[0]  # type: ignore
        vertex_v: V = edge_[1]  # type: ignore
        weight = float(edge_[2]) if len(edge_) > 2 else 1.0  # type: ignore

        # initialize degrees and increase the in degree of v
        if vertex_u not in self._in_degrees:
            self._in_degrees[vertex_u] = 0
        if vertex_v not in self._in_degrees:
            self._in_degrees[vertex_v] = 0

        self._in_degrees[vertex_v] += 1

        # add edge to set of edge weights
        self._dir_edges[(vertex_u, vertex_v)] = weight
        self._und_edges[frozenset((vertex_u, vertex_v))] = weight

        # add vertices to set of all vertices
        self._vertices.add(vertex_u)
        self._vertices.add(vertex_v)

        # add v to the list of vertices that are adjacent to u
        if vertex_u in self._adj_list:
            self._adj_list[vertex_u][vertex_v] = weight
        else:
            self._adj_list[vertex_u] = {vertex_v: weight}

        # add u to the list of vertices that are adjacent to v
        if not self.directed:
            if vertex_v in self._adj_list:
                self._adj_list[vertex_v][vertex_u] = weight
            else:
                self._adj_list[vertex_v] = {vertex_u: weight}

    def remove_edge(self, edge: tuple[V, V] | frozenset[V]) -> None:
        """Removes the given edge from the graph."""
        self._graph_has_changed = True
        vertex_u, vertex_v = edge

        # remove edge from set of edges
        if edge in self._dir_edges:
            del self._dir_edges[(vertex_u, vertex_v)]
        if edge in self._und_edges:
            del self._und_edges[frozenset(edge)]

        # remove edge from adjacency list
        if self.adjacent(vertex_u, vertex_v):
            del self._adj_list[vertex_u][vertex_v]
        if not self.directed and self.adjacent(vertex_v, vertex_u):
            del self._adj_list[vertex_v][vertex_u]

        # update in-degrees
        self._in_degrees[vertex_v] -= 1

    def remove_vertex(self, vertex_u: V) -> None:
        """Removes the given vertex from the graph. Worst time O(|V|)."""
        # Delete all edges from this vertex
        for vertex_v in copy.deepcopy(self.neighbors(vertex_u)):
            self.remove_edge((vertex_u, vertex_v))

        # delete this vertex
        self._vertices.remove(vertex_u)
        del self._adj_list[vertex_u]

    def adjacent(self, vertex_x: V, vertex_y: V) -> bool:
        """Tests whether there is an edge from the vertex x to the vertex y."""
        return vertex_y in self._adj_list[vertex_x]

    def neighbors(self, vertex_x: V) -> dict[V, float]:
        """Lists all vertices y such that there is an edge from the vertex x
            to the vertex y.
        """
        return self._adj_list[vertex_x]

    def successors(self, vertex_x: V) -> dict[V, float]:
        """Wrapper function for neighbors using directed graph terminology."""
        return self.neighbors(vertex_x)

    def deg(self, vertex: V) -> int:
        """Returns the number of edges connected to this vertex."""
        if self.directed:
            return self.in_deg(vertex) + self.out_deg(vertex)
        else:
            return self.out_deg(vertex)

    def in_deg(self, vertex: V) -> int:
        """Returns the number of edges running into this vertex."""
        return self._in_degrees[vertex]

    def out_deg(self, vertex: V) -> int:
        """Returns the number of edges running out of this vertex."""
        return len(self._adj_list[vertex])

    def weight(self, edge: tuple[V, V]) -> float:
        return self._adj_list[edge[0]][edge[1]]
