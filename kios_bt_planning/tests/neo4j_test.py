from neo4j import GraphDatabase

from typing import List, Dict, Any


class Neo4jInterface:
    """
    Interface for the Neo4j database
    """

    def __init__(self, uri: str, user: str, password: str):
        self.driver = GraphDatabase.driver(uri, auth=(user, password))

    def _clear_database(self, tx):
        tx.run("MATCH (n) DETACH DELETE n")

    def clear_database(self):
        with self.driver.session() as session:
            session.write_transaction(self._clear_database)

    def close_driver(self):
        self.driver.close()

    def _create_objects(self, tx, object_names: list):
        for obj in object_names:
            tx.run(f"CREATE (o:Object {{name: '{obj}'}})")

    def create_objects(self, object_names: list):
        with self.driver.session() as session:
            session.write_transaction(self._create_objects, object_names)

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

    def create_relations(self, relations: Dict[str, List[str]]):
        with self.driver.session() as session:
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


def test_neo4j():
    neo4j_interface = Neo4jInterface(
        uri="neo4j://localhost:7687", user="neo4j", password="14637982"
    )
    neo4j_interface.clear_database()
    neo4j_interface.create_objects(["a", "b", "c", "d", "e", "f"])
    relations = {
        "on": [("a", "b"), ("b", "c"), ("c", "d"), ("d", "e"), ("e", "f")],
        "next_to": [("a", "c"), ("c", "e"), ("b", "d"), ("d", "f")],
    }
    neo4j_interface.create_relations(relations)
    neo4j_interface.print_all()
    neo4j_interface.close_driver()


if __name__ == "__main__":
    test_neo4j()
