from mongodb_client import MongoDBClient
from socketserver import ThreadingMixIn

import random
import logging
import numpy as np
from xmlrpc.server import SimpleXMLRPCServer

logger = logging.getLogger("ml_service")


class DatabaseServer(ThreadingMixIn, SimpleXMLRPCServer):
    pass


class Database():
    def __init__(self, port, mongo_port=27017):
        # Database names:
        self.task_knowledge_db_name = "global_knowledge"  # knowledge of single tasks
        self.results_db_name = "global_ml_results"  # raw result data

        self.rpc_server = None
        self.db_client = MongoDBClient(port=mongo_port)
        self.port = port

        self.stop = False

    def start_server(self):
        """makes all functions available over rpc"""
        self.rpc_server = DatabaseServer(
            ("0.0.0.0", self.port), allow_none=True, logRequests=False)
        self.rpc_server.register_introspection_functions()
        self.rpc_server.register_function(self.store_result, "store_result")

        self.rpc_server.register_function(self.stop_server, "stop_server")

        self.rpc_server.register_function(self.get_result, "get_result")

        self.stop = False
        while not self.stop:
            self.rpc_server.handle_request()
        logger.debug("databse: Global database rpc server has stopped")
        return True

    def store_result(self, result: dict):
        """takes whole ml data of task and saves it to database"""
        logger.debug("Database.store_result")
        if isinstance(result, dict):
            logger.debug("Database.store_result: store task result")
            skill_class = result["meta"]["skill_class"]
            task_id = self.db_client.write(
                self.results_db_name, skill_class, result)
        else:
            logger.error(
                "Database.store_result: Received result is not of type dict or list! " + str(type(result)))
            return False
        # task_identity = {"skill_class": result["meta"]["skill_class"],
        #                 "tags": result["meta"]["tags"],
        #                 "optimum_weights": result["meta"]["cost_function"]["optimum_weights"],
        #                 "geometry_factor": result["meta"]["cost_function"]["geometry_factor"]}
        return task_id

    def get_result(self, db, collection, filter):
        logger.debug("Database.get_result")
        results = self.db_client.read(db, collection, filter)
        if len(results) == 0:
            return False
        if len(results) > 1:
            return False
        return results[0]

    def stop_server(self):
        logger.debug("database.stop_server")
        self.stop = True
