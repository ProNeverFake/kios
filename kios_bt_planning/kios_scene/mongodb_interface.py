from pymongo import MongoClient, database, collection
from typing import Dict, List
import logging

from kios_robot.data_types import MiosObject
from kios_utils.bblab_utils import setup_logger

mongodb_logger = setup_logger("MongoDB_interface")


class MongoDBInterface:
    client: MongoClient = None
    mios_database: database.Database = None
    mios_environment_collection: collection.Collection = None
    kios_database: database.Database = None
    mios_collections: Dict[str, collection.Collection] = {}
    kios_collections: Dict[str, collection.Collection] = {}

    def __init__(self, address: str = "127.0.0.1", port: int = 27017):
        self.client = MongoClient(address, port)

        self.mios_database = self.client["miosL"]
        self.mios_environment_collection = self.mios_database["environment"]
        self.mios_collections["environment"] = self.mios_environment_collection
        self.kios_database = self.client["kios"]
        # ! BBTEST
        # self.setup_defaults()

    # # ! DISCARDED
    # def setup_defaults(self):
    #     # if tools collection does not exist, create it. add the default tools.
    #     if "tools" not in self.kios_database.list_collection_names():
    #         self.kios_database.create_collection("tools")
    #         self.kios_database["tools"].insert_one(
    #             {
    #                 "name": "parallel_box1",
    #                 "EE_T_TCP": [
    #                     [1, 0, 0, 0.0],
    #                     [0, 1, 0, 0.0],
    #                     [0, 0, 1, 0.1],
    #                     [0, 0, 0, 1],
    #                 ],
    #             }
    #         )
    #         self.kios_database["tools"].insert_one(
    #             {
    #                 "name": "parallel_box2",
    #                 "EE_T_TCP": [
    #                     [1, 0, 0, 0.0],
    #                     [0, 1, 0, 0.0],
    #                     [0, 0, 1, 0.1],
    #                     [0, 0, 0, 1],
    #                 ],
    #             }
    #         )
    #         self.kios_database["tools"].insert_one(
    #             {
    #                 "name": "inward_claw",
    #                 "EE_T_TCP": [
    #                     [1, 0, 0, 0.0],
    #                     [0, 1, 0, 0.0],
    #                     [0, 0, 1, 0.1],
    #                     [0, 0, 0, 1],
    #                 ],
    #             }
    #         )
    #         self.kios_database["tools"].insert_one(
    #             {
    #                 "name": "outward_claw",
    #                 "EE_T_TCP": [
    #                     [1, 0, 0, 0.0],
    #                     [0, 1, 0, 0.0],
    #                     [0, 0, 1, 0.1],
    #                     [0, 0, 0, 1],
    #                 ],
    #             }
    #         )

    def query_mios_object(self, object_name: str) -> MiosObject:
        """
        be careful. the name can be duplicated.
        """
        query = {"name": object_name}
        result = self.mios_environment_collection.find(query)
        count = self.mios_environment_collection.count_documents(query)
        if count > 1:
            raise Exception("Duplicated object name!")
        elif count == 0:
            mongodb_logger.warn(f'object "{object_name}" not found in the database')
            # ! HACK
            return MiosObject.generate_dummy(object_name=object_name)
        for document in result:
            return MiosObject.from_json(document)

    def fetch_all_mios_objects(self) -> List[Dict]:
        target_collection = self.mios_collections.get("environment")
        documents = list(target_collection.find())
        return documents

    def erase_all_mios_objects(self) -> None:
        print(
            "\033[91mDangerous operation. Type 'yes' to continue. Type 'c' to cancel.\033[0m"
        )
        user_input = input()
        if user_input == "yes":
            # Erase all the objects in the collection
            target_collection = self.mios_collections.get("environment")
            target_collection.delete_many({})
            print("\033[92mAll objects erased.\033[0m")
        elif user_input == "c":
            # Quit the program
            print("\033[92mOperation canceled.\033[0m")
        else:
            # Invalid input, handle accordingly
            print("\033[93mInvalid input. Operation canceled.\033[0m")

    def inject_mios_objects(self, documents: List[Dict]):
        target_collection = self.mios_collections.get("environment")
        target_collection.insert_many(documents)

    def update_mios_objects(self, documents: List[Dict]):
        target_collection = self.mios_collections.get("environment")
        for document in documents:
            query = {"name": document["name"]}
            target_collection.replace_one(query, document)

    def query(self, query):
        return self.collection.find(query)

    # def query_from_collection(self, key: str, collection_name: str):
    #     target_collection = self.mios_collections.get(collection_name)
    #     if target_collection is None:
    #         raise Exception(f"Collection {collection_name} not found")
    #     query = {key: collection_name}
    #     return target_collection.find(query)

    def save(self, document):
        return self.collection.insert_one(document)

    def fetch_all_from_mios_collection(self, collection_name: str) -> List[Dict]:
        target_collection = self.mios_collections.get(collection_name)
        if target_collection is None:
            raise Exception(f"Collection {collection_name} not found")
        documents = list(target_collection.find())
        return documents


def test_query():
    interface = MongoDBInterface()
    result = interface.query_mios_object("EndEffector")
    print(result)


if __name__ == "__main__":
    test_query()
