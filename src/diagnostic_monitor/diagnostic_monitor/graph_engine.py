"""Dynamic ROS 2 graph discovery and dependency graph engine.

Discovers the live ROS 2 computational graph using official ROS 2 graph APIs.
Constructs directed dependency graph G = (V, E) without any hardcoded topology.
Strictly excludes fault injection and evaluation topics to maintain isolation.
"""

from collections import deque
from typing import Dict, List, Optional, Sequence, Set, Tuple


class DependencyGraph:
    """Directed dependency graph of ROS 2 nodes and their communication links."""

    def __init__(self):
        self.nodes: Set[str] = set()
        self.adj: Dict[str, Set[str]] = {}       # u -> set of v where u publishes and v subscribes
        self.rev_adj: Dict[str, Set[str]] = {}   # v -> set of u
        self.edge_topics: Dict[Tuple[str, str], Set[str]] = {}

    def add_node(self, node: str):
        if node not in self.nodes:
            self.nodes.add(node)
            self.adj[node] = set()
            self.rev_adj[node] = set()

    def add_edge(self, u: str, v: str, topic: str):
        self.add_node(u)
        self.add_node(v)
        self.adj[u].add(v)
        self.rev_adj[v].add(u)
        key = (u, v)
        if key not in self.edge_topics:
            self.edge_topics[key] = set()
        self.edge_topics[key].add(topic)

    def remove_node(self, node: str):
        if node in self.nodes:
            self.nodes.remove(node)
            # Remove outgoing edges
            for v in self.adj.pop(node, set()):
                self.rev_adj.get(v, set()).discard(node)
                self.edge_topics.pop((node, v), None)
            # Remove incoming edges
            for u in self.rev_adj.pop(node, set()):
                self.adj.get(u, set()).discard(node)
                self.edge_topics.pop((u, node), None)

    def successors(self, node: str) -> Set[str]:
        return set(self.adj.get(node, set()))

    def predecessors(self, node: str) -> Set[str]:
        return set(self.rev_adj.get(node, set()))

    def descendants(self, node: str) -> Set[str]:
        """Returns all downstream nodes reachable from `node` via directed edges."""
        visited = set()
        queue = deque([node])
        while queue:
            curr = queue.popleft()
            for succ in self.adj.get(curr, set()):
                if succ not in visited:
                    visited.add(succ)
                    queue.append(succ)
        return visited

    def ancestors(self, node: str) -> Set[str]:
        """Returns all upstream nodes from which `node` can be reached."""
        visited = set()
        queue = deque([node])
        while queue:
            curr = queue.popleft()
            for pred in self.rev_adj.get(curr, set()):
                if pred not in visited:
                    visited.add(pred)
                    queue.append(pred)
        return visited

    def find_all_paths(self, source: str, target: str, max_depth: int = 10) -> List[List[str]]:
        """Finds all simple directed paths from source to target."""
        if source not in self.nodes or target not in self.nodes:
            return []
        paths = []
        queue = deque([[source]])
        while queue:
            path = queue.popleft()
            curr = path[-1]
            if curr == target:
                paths.append(path)
                continue
            if len(path) > max_depth:
                continue
            for succ in self.adj.get(curr, set()):
                if succ not in path:  # Prevent cycles
                    queue.append(path + [succ])
        return paths

    def to_dict(self) -> dict:
        return {
            'nodes': sorted(list(self.nodes)),
            'edges': sorted(
                [
                    {'from': u, 'to': v, 'topics': sorted(list(topics))}
                    for (u, v), topics in self.edge_topics.items()
                ],
                key=lambda e: (e['from'], e['to']),
            ),
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'DependencyGraph':
        g = cls()
        for n in d.get('nodes', []):
            g.add_node(n)
        for e in d.get('edges', []):
            for t in e.get('topics', []) or ['']:
                g.add_edge(e['from'], e['to'], t)
        return g

    def retain_edges_of(self, other: 'DependencyGraph', nodes: Set[str]):
        """Copies edges touching `nodes` from `other` into this graph.

        Used to keep the last-known dependency structure of a node that has
        vanished (crashed) so that its downstream symptoms can still be
        attributed to it during the analysis window.
        """
        for (u, v), topics in other.edge_topics.items():
            if u in nodes or v in nodes:
                for t in topics:
                    self.add_edge(u, v, t)

    def __eq__(self, other) -> bool:
        return isinstance(other, DependencyGraph) and self.to_dict() == other.to_dict()


class RosGraphDiscoverer:
    """Discovers ROS 2 dependency graph dynamically using rclpy Graph APIs."""

    # Topics to ignore to ensure strict ground truth isolation and filter out ROS 2 internals
    EXCLUDED_TOPIC_PREFIXES = (
        '/rosout',
        '/parameter_events',
        '/rca/fault_',      # Ground truth fault injection isolation
        '/rca/eval_',       # Evaluation isolation
        '/rca/diagnosis',   # Monitor output isolation
        '/rca/dashboard',   # Monitor output isolation (live TUI state)
        '/experiment_',     # Experiment controller isolation
    )

    EXCLUDED_NODE_NAMES = (
        'diagnostic_monitor',
        'fault_injector',
        'experiment_controller',
    )

    # Deployment-specific additions, supplied at launch time (NOT topology).
    # Used for raw simulator-transport topics that are not part of the monitored
    # application data flow, e.g. the Gazebo/ros_gz bridge topics under '/gz/',
    # which carry simulator clock stamps rather than ROS wall-clock stamps.
    # This never names an application node or an application data-flow topic, so
    # the monitored dependency graph is still discovered, never declared.
    EXTRA_TOPIC_PREFIXES: Tuple[str, ...] = ()
    EXTRA_NODE_NAMES: Tuple[str, ...] = ()

    @classmethod
    def configure(cls, topic_prefixes=(), node_names=()):
        """Registers deployment-specific exclusions (idempotent, process-wide)."""
        cls.EXTRA_TOPIC_PREFIXES = tuple(sorted({p for p in topic_prefixes if p}))
        cls.EXTRA_NODE_NAMES = tuple(sorted({n for n in node_names if n}))
        return cls.EXTRA_TOPIC_PREFIXES, cls.EXTRA_NODE_NAMES

    @classmethod
    def should_ignore_topic(cls, topic_name: str) -> bool:
        return any(topic_name.startswith(prefix)
                   for prefix in cls.EXCLUDED_TOPIC_PREFIXES + cls.EXTRA_TOPIC_PREFIXES)

    @classmethod
    def should_ignore_node(cls, node_name: str) -> bool:
        if node_name.startswith('_ros2cli_'):
            return True
        if node_name == '_NODE_NAME_UNKNOWN_':      # rmw sentinel: endpoint without a resolvable node
            return True
        return node_name in cls.EXCLUDED_NODE_NAMES + cls.EXTRA_NODE_NAMES

    @classmethod
    def discover(cls, ros_node) -> DependencyGraph:
        """Queries ROS 2 Graph APIs on `ros_node` to build the live DependencyGraph.

        A node belongs to the dependency graph iff it is a publisher or a
        subscriber of at least one application topic (i.e. a topic that is not
        excluded). Pure observers such as the diagnostic monitor itself, CLI
        tools or the dashboard TUI only touch excluded ``/rca/*`` topics (or no
        topics at all) and therefore never appear as graph nodes.
        """
        graph = DependencyGraph()

        # 1. Discover all active nodes
        try:
            node_names_and_ns = ros_node.get_node_names_and_namespaces()
        except Exception as e:
            ros_node.get_logger().error(f'Failed to get node names: {e}')
            return graph

        active_app_nodes = set()
        for name, _ in node_names_and_ns:
            if not cls.should_ignore_node(name):
                active_app_nodes.add(name)

        # 2. Discover all topics and publisher/subscriber endpoint info
        try:
            topic_names_and_types = ros_node.get_topic_names_and_types()
        except Exception as e:
            ros_node.get_logger().error(f'Failed to get topic names: {e}')
            return graph

        endpoint_nodes = set()
        edges = []
        for topic_name, _ in topic_names_and_types:
            if cls.should_ignore_topic(topic_name):
                continue
            try:
                pubs = ros_node.get_publishers_info_by_topic(topic_name)
            except Exception:
                pubs = []
            try:
                subs = ros_node.get_subscriptions_info_by_topic(topic_name)
            except Exception:
                subs = []

            pub_nodes = [p.node_name for p in pubs if not cls.should_ignore_node(p.node_name)]
            sub_nodes = [s.node_name for s in subs if not cls.should_ignore_node(s.node_name)]
            endpoint_nodes.update(pub_nodes)
            endpoint_nodes.update(sub_nodes)
            for pub_node in pub_nodes:
                for sub_node in sub_nodes:
                    if pub_node != sub_node:
                        edges.append((pub_node, sub_node, topic_name))

        # 3. Graph = application data-flow participants only
        for name in sorted(active_app_nodes & endpoint_nodes):
            graph.add_node(name)
        for u, v, topic in edges:
            graph.add_edge(u, v, topic)

        return graph
