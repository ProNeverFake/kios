from neo4j import GraphDatabase

from typing import List, Dict, Any, Set
from kios_world.data_types import WorldNode, Relationship

"""
currently this module is just for visulization.
# ! DEPRECATED
"""


class Neo4jInterface:
    """
    Interface for the Neo4j database
    """

    def __init__(
        self,
        uri: str = "neo4j://localhost:7687",
        user: str = "neo4j",
        password: str = "12345678",
    ):
        self.uri = uri
        self.user = user
        self.password = password
        self.driver = GraphDatabase.driver(uri, auth=(user, password))

    def _clear_database(self, tx):
        tx.run("MATCH (n) DETACH DELETE n")

    def clear_database(self):
        with self.driver.session() as session:
            session.write_transaction(self._clear_database)

    def open_driver(self):
        self.driver = GraphDatabase.driver(self.uri, auth=(self.user, self.password))

    def close_driver(self):
        self.driver.close()

    def _create_objects(self, tx, object_names: list):
        for obj in object_names:
            tx.run(f"CREATE (o:Object {{name: '{obj}'}})")

    def create_objects(self, object_names: list):
        with self.driver.session() as session:
            session.write_transaction(self._create_objects, object_names)

    def _set_properties(self, tx, objects_and_properties: Dict[str, Set[str]]):
        for obj, properties in objects_and_properties.items():
            for prop in properties:
                tx.run(f"MATCH (o:Object {{name: '{obj}'}}) SET o.{prop} = ''")
                # tx.run(f"MATCH (o:Object {{name: '{obj}'}}) SET o.{prop} = ''")

    def set_properties(self, world_node_dict: Dict[str, WorldNode]):
        with self.driver.session() as session:
            objects_and_properties = {}
            for obj, world_node in world_node_dict.items():
                objects_and_properties[obj] = world_node.properties
            session.write_transaction(self._set_properties, objects_and_properties)

    def _create_relations(self, tx, relations: Dict[str, List[str]]):
        """
        create relations between objects, if objects do not exist, create them.
        """
        for relation, objects in relations.items():
            for obj in objects:
                result_from = tx.run(f"MATCH (o:Object {{name: '{obj[0]}'}}) RETURN o")
                result_to = tx.run(f"MATCH (o:Object {{name: '{obj[1]}'}}) RETURN o")
                if result_from.single() and result_to.single():
                    tx.run(
                        f"MATCH (from:Object {{name: '{obj[0]}'}}), (to:Object {{name: '{obj[1]}'}}) "
                        f"CREATE (from)-[:{relation.upper()}]->(to)"
                    )
                else:
                    # raise ValueError(
                    #     f"Object {obj} does not exist in the database!"
                    # )
                    if not result_from.single():
                        self.create_objects([obj[0]])
                    if not result_to.single():
                        self.create_objects([obj[1]])
                    tx.run(
                        f"MATCH (from:Object {{name: '{obj[0]}'}}), (to:Object {{name: '{obj[1]}'}}) "
                        f"CREATE (from)-[:{relation.upper()}]->(to)"
                    )

    def create_relations(self, relation_set: Set[Relationship]):
        with self.driver.session() as session:
            relations = {}
            for relation in relation_set:
                if relation.relation_name not in relations:
                    relations[relation.relation_name] = []
                relations[relation.relation_name].append(relation.objects)
            session.write_transaction(self._create_relations, relations)

    def fetch_all(self):
        with self.driver.session() as session:
            result = session.run("MATCH (from)-[r]->(to) RETURN from.name, r, to.name")
            return [
                (record["from.name"], record["r"].type, record["to.name"])
                for record in result
            ]

    def fetch_nodes(self):
        with self.driver.session() as session:
            result = session.run("MATCH (o:Object) RETURN o.name")
            return [record["o.name"] for record in result]

    def fetch_relations(self):
        with self.driver.session() as session:
            result = session.run("MATCH (from)-[r]->(to) RETURN from.name, r, to.name")
            return [
                (record["from.name"], record["r"].type, record["to.name"])
                for record in result
            ]

    def fetch_specific(self, node_names: List[str]):
        """
        fetch specific nodes and relations from the database
        """
        with self.driver.session() as session:
            result = session.run(
                f"MATCH (from)-[r]->(to) WHERE from.name IN {node_names} AND to.name IN {node_names} RETURN from.name, r, to.name"
            )
            return [
                (record["from.name"], record["r"].type, record["to.name"])
                for record in result
            ]

    def remove_nodes(self, node_names: List[str]):
        """
        remove specific nodes from the database
        """
        with self.driver.session() as session:
            for node_name in node_names:
                session.run(f"MATCH (o:Object {{name: '{node_name}'}}) DETACH DELETE o")

    def remove_relations(self, relations: Dict[str, List[str]]):
        """
        remove specific relations from the database
        """
        with self.driver.session() as session:
            for relation, objects in relations.items():
                for obj in objects:
                    session.run(
                        f"MATCH (from:Object {{name: '{obj[0]}'}})-[r:{relation.upper()}]->(to:Object {{name: '{obj[1]}'}}) DELETE r"
                    )

    def print_all(self):
        with self.driver.session() as session:
            result = session.run("MATCH (from)-[r]->(to) RETURN from, r, to")
            for record in result:
                print(
                    f"{record['from']['name']} - {record['r'].type} - {record['to']['name']}"
                )
