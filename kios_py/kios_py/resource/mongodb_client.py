import pymongo
from bson import objectid
from pymongo import MongoClient
import logging
import time

logger = logging.getLogger("ml_service")


class MongoDBClient():
    """Simple Client for MongoDB interaction"""

    def __init__(self, host: str='localhost', port: int=27017, max_retry: int=3):
        self.client = MongoClient(host, port)
        self.max_retry = max_retry
        logger.info("MongoDB is initialized at " + host + ":" + str(port))

    def read(self, db: str, collection: str, search_param: dict) -> list:
        db_connection = self.client[db]
        col = db_connection[collection]
        findings = []
        #if search params are in list, search for all the contend (not the list itself...)
        for key in search_param:
            value = search_param[key]
            if key == "tags" or key == "meta.tags":
                if isinstance(value, list):
                    search_param[key] = {"$all": value}
            if not value:  # check if key exists if no value is given
                search_param[key] = {"$exists": True} 
            if key == "_id":  # if _id is given as string  ->  ObjectId
                if isinstance(search_param[key],str):
                    search_param[key] = objectid.ObjectId(search_param[key])
        retry_count = 0
        #while not findings:
        for f in col.find(filter=search_param):
            f["_id"] = str(f["_id"])
            findings.append(f)
        #if retry_count >= self.max_retry:
        #    break
        #else:
        #    retry_count += 1
        #    time.sleep(0.5)
        return findings

    def write(self, db: str, collection: str, document: dict or list) -> objectid.ObjectId or list:
        db_connection = self.client[db]
        col = db_connection[collection]
        if isinstance(document, list):
            # document_id = col.insert_many(document).inserted_ids
            document_ids = []
            for doc in document:
                single_id = self._write_single(db, collection, doc)
                document_ids.append(single_id)
        elif isinstance(document, dict):
            document_ids = self._write_single(db, collection, document)
        else:
            logger.error("Document type is not dict or list, but " + str(type(document)))
            logger.info("Cannot insert document into Database.")
            return False
        return document_ids

    def _write_single(self, db: str, collection: str, document: dict) -> objectid.ObjectId:
        db_connection = self.client[db]
        col = db_connection[collection]
        if isinstance(document, dict):
            if "_id" in document.keys():  # if _id is given as string  ->  ObjectId
                if isinstance(document["_id"],str):
                    document["_id"] = objectid.ObjectId(document["_id"])
            return str(col.insert_one(document).inserted_id)
        else:
            logger.error("Document type is not dict, but " + str(type(document)))
            logger.info("Cannot insert document into Database.")
            return False

    def remove(self, db: str, collection: str, search_param: dict) -> bool:
        db_connection = self.client[db]
        col = db_connection[collection]
        if "_id" in search_param.keys():  # if _id is given as string  ->  ObjectId
                if isinstance(search_param["_id"], str):
                    search_param["_id"] = objectid.ObjectId(search_param["_id"])
        for key in search_param:
            value = search_param[key]
            if key == "tags" or "meta.tags":
                if isinstance(value, list):
                    search_param[key] = {"$all": value}
        result = col.delete_many(search_param, collation=None, session=None)
        if result.deleted_count > 0:
            return True
        else:
            return False

    def update(self, db: str, collection: str, search_param: dict, new_param: dict) -> bool:
        db_connection = self.client[db]
        col = db_connection[collection]
        if isinstance(search_param, dict):
            if "_id" in search_param.keys():  # if _id is given as string  ->  ObjectId
                if isinstance(search_param["_id"], str):
                    search_param["_id"] = objectid.ObjectId(search_param["_id"])
            if isinstance(new_param, dict):
                if new_param.get("_id", False):
                    new_param.pop("_id")
                new_param_set = {"$set": new_param}
                col.update_one(search_param, new_param_set)
            else:
                logger.error("new parameter for update are not dict, but " + str(type(new_param)))
                return False
        else:
            logger.error("search parameter for update are not dict, but " + str(type(new_param)))
            return False
        return True

    def delete_collection(self, db: str, collection: str):
        db_connection = self.client[db]
        col = db_connection[collection]
        col.drop()

    def get_collections(self, db:str) -> list:
        db_connection = self.client[db]
        return db_connection.list_collection_names(filter={})
