from pymongo import MongoClient, database, collection
from typing import Dict, List

from kios_robot.data_types import MiosObject


class MongoDBInterface:
    client: MongoClient = None
    mios_database: database.Database = None
    mios_environment_collection: collection.Collection = None
    kios_database: database.Database = None

    def __init__(self, address: str = "127.0.0.1", port: int = 27017):
        self.client = MongoClient(address, port)

        self.mios_database = self.client["miosL"]
        self.mios_environment_collection = self.mios_database["environment"]
        # self.database_dict["miosR"] = self.client["miosR"]
        self.kios_database = self.client["kios"]

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
            raise Exception("Object not found!")
        for document in result:
            return MiosObject.from_json(document)

    def query(self, query):
        return self.collection.find(query)

    def save(self, document):
        return self.collection.insert_one(document)


def test_query():
    interface = MongoDBInterface()
    result = interface.query_mios_object("EndEffector")
    print(result)


test_query()
