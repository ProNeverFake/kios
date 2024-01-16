import networkx as nx

# Create a new directed graph
G = nx.DiGraph()

# Add nodes
G.add_node("table")
G.add_node("ball")
G.add_node("cabinet")

# Add property to cabinet
G.nodes["cabinet"]["open"] = True

# Add relation (edge) from ball to table
G.add_edge("ball", "table", relation="on_something")


# Function to check if specific relation exists
def check_relation(graph, source, target, relation):
    if graph.has_edge(source, target):
        return graph[source][target].get("relation") == relation
    return False


# Check if 'ball' is 'on_something' 'table'
relation_exists = check_relation(G, "ball", "table", "on_something")
print("Relation exists:", relation_exists)  # Output: True


# Function to check if a node has a specific property
def check_property(graph, node, property_key):
    return graph.nodes[node].get(property_key) is not None


# Check if 'cabinet' has property 'open'
property_exists = check_property(G, "cabinet", "open")
print("Cabinet has 'open' property:", property_exists)  # Output: True
