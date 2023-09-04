#include "kios_communication/mongodb_client.hpp"
#include "spdlog/spdlog.h"

#include <bsoncxx/json.hpp>
#include <bsoncxx/document/view_or_value.hpp>
#include <bsoncxx/document/view.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <mongocxx/uri.hpp>

#include <mongocxx/exception/bulk_write_exception.hpp>
#include <mongocxx/exception/logic_error.hpp>
#include <mongocxx/exception/query_exception.hpp>
#include <mongocxx/exception/operation_exception.hpp>
#include <bsoncxx/exception/exception.hpp>

#include "kios_utils/parameters.hpp"

#include <thread>
#include <iostream>

namespace kios
{

    MongodbClient::MongodbClient(const std::string &database, unsigned port)
    {
        std::cout << "TEST CREATE CLIENT PRE" << std::endl;
        spdlog::trace("MongodbClient::MongodbClient");
        std::scoped_lock<std::mutex> lock(m_mutex_db_access);
        spdlog::debug("Connecting to database " + database + " on localhost:" + std::to_string(port));
        std::cout << port << std::endl;
        std::cout << "mongodb://localhost:" + std::to_string(port) << std::endl;
        mongocxx::uri uri("mongodb://localhost:" + std::to_string(port));
        std::cout << "TEST CREATE CLIENT POST" << std::endl;

        m_client = mongocxx::client(uri);

        bool found_database = false;
        bool message_displayed = false;
        while (!found_database)
        {
            try
            {
                std::vector<std::string> db_names = m_client.list_database_names();
                m_mongodb = m_client.database(database);
                m_collections.clear();
                m_collections.insert(std::pair<const char *, mongocxx::collection>("frames", m_mongodb["frames"]));
                m_collections.insert(std::pair<const char *, mongocxx::collection>("environment", m_mongodb["environment"]));
                m_collections.insert(std::pair<const char *, mongocxx::collection>("parameters", m_mongodb["parameters"]));
                found_database = true;
                spdlog::debug("Mongodb client initialized.");
            }
            catch (const mongocxx::exception &e)
            {
                if (!message_displayed)
                {
                    spdlog::debug(e.what());
                    spdlog::warn("Database is not reachable or faulty, I will try again...");
                    message_displayed = true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    bool MongodbClient::read_documents(const std::string &collection, std::set<nlohmann::json> &docs)
    {
        spdlog::trace("MongodbClient::read_documents(string,set<json>)");
        try
        {
            if (!m_mongodb.has_collection(collection))
            {
                spdlog::error("Database has no " + collection + " collection");
                return false;
            }
            if (m_collections.find(collection) == m_collections.end())
            {
                spdlog::error("Database has no " + collection + " collection");
                return false;
            }
            spdlog::trace("[MONGODBCLIENT]: READ_DOCUMENT.PRE_FIND");
            for (const auto &d : m_collections[collection].find({}))
            {
                std::string description = bsoncxx::to_json(d);
                docs.insert(nlohmann::json::parse(description));
            }
            return true;
        }
        catch (const mongocxx::logic_error &e)
        {
            spdlog::error("Reading of documents in collection " + collection + " has failed.");
            spdlog::debug(e.what());
            return false;
        }
        catch (const mongocxx::operation_exception &e)
        {
            spdlog::error("Reading of documents in collection " + collection + " has failed.");
            spdlog::debug(e.what());
            return false;
        }
        catch (const mongocxx::exception &e)
        {
            spdlog::error("Reading of documents in collection " + collection + " has failed.");
            spdlog::debug(e.what());
            return false;
        }
        catch (const bsoncxx::exception &e)
        {
            spdlog::error("Reading of documents in collection " + collection + " has failed.");
            spdlog::debug(e.what());
            return false;
        }
        catch (const nlohmann::detail::parse_error &e)
        {
            spdlog::error("Reading of documents in collection " + collection + " has failed.");
            spdlog::debug(e.what());
            return false;
        }
    }

    bool MongodbClient::read_document(const std::string &name, const std::string &collection, nlohmann::json &descr)
    {
        spdlog::trace("MongodbClient::read_document(string,string,json)");
        std::scoped_lock<std::mutex> lock(m_mutex_db_access);
        try
        {
            if (!m_mongodb.has_collection(collection))
            {
                spdlog::error("Database has no " + collection + " collection");
                return false;
            }
            if (m_collections.find(collection) == m_collections.end())
            {
                spdlog::error("Database has no " + collection + " collection");
                return false;
            }
            spdlog::trace("[MONGODBCLIENT]: READ_DOCUMENT.PRE_COUNT");
            unsigned n_doc = m_collections[collection].count_documents({bsoncxx::builder::stream::document{} << "name" << name << bsoncxx::builder::stream::finalize});
            if (n_doc == 0)
            {
                spdlog::error("No document with name " + name + " of type " + collection + " present in database.");
                descr = nlohmann::json();
                return false;
            }
            if (n_doc > 1)
            {
                spdlog::error("Multiple documents with name " + name + " of type " + collection + " present in database.");
                descr = nlohmann::json();
                return false;
            }
            spdlog::trace("[MONGODBCLIENT]: READ_DOCUMENT.PRE_FIND");
            bsoncxx::stdx::optional<bsoncxx::document::value> doc = m_collections[collection].find_one({bsoncxx::builder::stream::document{} << "name" << name << bsoncxx::builder::stream::finalize});
            std::string descr_str = bsoncxx::to_json(*doc);
            descr = nlohmann::json::parse(descr_str);
            spdlog::trace("[MONGODBCLIENT]: READ_DOCUMENT.POST_FIND");
            return true;
        }
        catch (const mongocxx::logic_error &e)
        {
            spdlog::error("Reading of document with name " + name + " of type " + collection + " has failed.");
            spdlog::debug(e.what());
            return false;
        }
        catch (const mongocxx::operation_exception &e)
        {
            spdlog::error("Reading of document with name " + name + " of type " + collection + " has failed.");
            spdlog::debug(e.what());
            return false;
        }
        catch (const mongocxx::exception &e)
        {
            spdlog::error("Reading of document with name " + name + " of type " + collection + " has failed.");
            spdlog::debug(e.what());
            return false;
        }
        catch (const bsoncxx::exception &e)
        {
            spdlog::error("Reading of document with name " + name + " of type " + collection + " has failed.");
            spdlog::debug(e.what());
            return false;
        }
        catch (const nlohmann::detail::parse_error &e)
        {
            spdlog::error("Reading of document with name " + name + " of type " + collection + " has failed.");
            spdlog::debug(e.what());
            return false;
        }
        return false;
    }

    bool MongodbClient::write_documents(const std::string &collection, const std::set<nlohmann::json> &docs, bool overwrite)
    {
        spdlog::trace("MongodbClient::write_documents");
        for (const auto &d : docs)
        {
            if (d.find("name") == d.end())
            {
                spdlog::error("Cannot upload document to database since it has no field <name>.");
                return false;
            }
            if (!write_document(d["name"], collection, d, overwrite))
            {
                return false;
            }
        }
        return true;
    }

    bool MongodbClient::write_document(const std::string &name, const std::string &collection, const nlohmann::json &descr, bool overwrite)
    {
        spdlog::trace("MongodbClient::write_document");
        std::scoped_lock<std::mutex> lock(m_mutex_db_access);
        try
        {
            if (!m_mongodb.has_collection(collection))
            {
                spdlog::error("Database has no collection " + collection + ".");
                return false;
            }
            nlohmann::json descr_in = descr;
            descr_in["name"] = name;
            bsoncxx::document::view_or_value doc = bsoncxx::from_json(descr_in.dump());
            int n_docs = m_collections[collection].count_documents({bsoncxx::builder::stream::document{} << "name" << name << bsoncxx::builder::stream::finalize});
            if (n_docs > 1)
            {
                spdlog::error("Multiple documents with name " + name + " present in collection " + collection + ".");
                return false;
            }
            else if (n_docs == 1 && overwrite)
            {
                m_mongodb[collection].replace_one(bsoncxx::builder::stream::document{} << "name" << name << bsoncxx::builder::stream::finalize, doc);
            }
            else if (n_docs == 1 && !overwrite)
            {
                spdlog::error("Document with name " + name + " already exists in collection " + collection + " and overwrite was not set.");
            }
            else
            {
                m_mongodb[collection].insert_one(doc);
            }
        }
        catch (const mongocxx::query_exception &e)
        {
            spdlog::error("Writing of document with name " + name + " of type " + collection + " has failed.");
            spdlog::debug(e.what());
            return false;
        }
        catch (const mongocxx::operation_exception &e)
        {
            spdlog::error("Writing of document with name " + name + " of type " + collection + " has failed.");
            spdlog::debug(e.what());
            return false;
        }
        catch (const mongocxx::logic_error &e)
        {
            spdlog::error("Writing of document with name " + name + " of type " + collection + " has failed.");
            spdlog::debug(e.what());
            return false;
        }
        catch (const bsoncxx::exception &e)
        {
            spdlog::error("Writing of document with name " + name + " of type " + collection + " has failed.");
            spdlog::debug(e.what());
            return false;
        }
        catch (const nlohmann::detail::type_error &e)
        {
            spdlog::error("Writing of document with name " + name + " of type " + collection + " has failed.");
            spdlog::debug(e.what());
            return false;
        }
        return true;
    }

    bool MongodbClient::make_document_consistent(const std::string &name, std::string collection, const nlohmann::json &template_doc)
    {
        spdlog::trace("MongodbClient::make_document_consistent");
        try
        {
            bsoncxx::document::view_or_value doc = bsoncxx::from_json(template_doc.dump());
            if (!m_mongodb.has_collection(collection))
            {
                m_mongodb[collection].insert_one(doc);
            }
            else
            {
                if (m_mongodb[collection].count_documents({bsoncxx::builder::stream::document{} << "name" << name << bsoncxx::builder::stream::finalize}) == 0)
                {
                    m_mongodb[collection].insert_one(doc);
                }
                else if (m_mongodb[collection].count_documents({bsoncxx::builder::stream::document{} << "name" << name << bsoncxx::builder::stream::finalize}) > 1)
                {
                    m_mongodb[collection].delete_many({bsoncxx::builder::stream::document{} << "name" << name << bsoncxx::builder::stream::finalize});
                    m_mongodb[collection].insert_one(doc);
                }
                else
                {
                    nlohmann::json doc_in_database;
                    read_document(name, collection, doc_in_database);
                    for (const auto &el : template_doc.items())
                    { // for every element in template
                        if (el.key() == "_id")
                        {
                            continue;
                        }
                        if (doc_in_database.find(el.key()) != doc_in_database.end())
                        { // if element is found in doc
                            if (el.value().type() != doc_in_database[el.key()].type())
                            {                                                       // if type of element is not equal to type of element with same key in doc
                                doc_in_database[el.key()] = template_doc[el.key()]; // replace with template
                            }
                            else
                            {
                            }
                        }
                        else
                        {
                            doc_in_database[el.key()] = template_doc[el.key()]; // assign with template
                        }
                    }
                    for (const auto &el : doc_in_database.items())
                    { // for every element in document from database
                        if (el.key() == "_id")
                        {
                            continue;
                        }
                        if (template_doc.find(el.key()) == template_doc.end())
                        { // if element is not found in template
                            doc_in_database.erase(el.key());
                        }
                        if (el.value().is_array())
                        {
                            if (!template_doc[el.key()].is_array())
                            {
                                doc_in_database[el.key()] = template_doc[el.key()];
                            }
                            else if (el.value().size() != template_doc[el.key()].size())
                            {
                                doc_in_database[el.key()] = template_doc[el.key()];
                            }
                        }
                    }
                    bsoncxx::document::view_or_value doc_replacement = bsoncxx::from_json(doc_in_database.dump());
                    m_mongodb[collection].replace_one({bsoncxx::builder::stream::document{} << "name" << name << bsoncxx::builder::stream::finalize}, doc_replacement);
                }
            }
        }
        catch (const bsoncxx::exception &e)
        {
            spdlog::debug(e.what());
            spdlog::error("Could not make document with name " + name + " in collection " + collection + " consistent.");
            return false;
        }
        catch (const mongocxx::bulk_write_exception &e)
        {
            spdlog::debug(e.what());
            spdlog::error("Could not make document with name " + name + " in collection " + collection + " consistent.");
            return false;
        }
        catch (const mongocxx::logic_error &e)
        {
            spdlog::debug(e.what());
            spdlog::error("Could not make document with name " + name + " in collection " + collection + " consistent.");
            return false;
        }
        catch (const mongocxx::query_exception &e)
        {
            spdlog::debug(e.what());
            spdlog::error("Could not make document with name " + name + " in collection " + collection + " consistent.");
            return false;
        }
        catch (const nlohmann::detail::type_error &e)
        {
            spdlog::debug(e.what());
            spdlog::error("Could not make document with name " + name + " in collection " + collection + " consistent.");
            return false;
        }
        catch (const mongocxx::operation_exception &e)
        {
            spdlog::debug(e.what());
            return false;
        }

        return true;
    }

    bool MongodbClient::health_check() const
    {
        spdlog::trace("MongodbClient::health_check");
        try
        {
            unsigned n_doc_parameters = 6;
            if (!m_mongodb.has_collection("parameters"))
            {
                spdlog::error("Database has no parameters collection.");
                return false;
            }
            if (!m_mongodb.has_collection("environment"))
            {
                spdlog::error("Database has no environment collection.");
                return false;
            }
            unsigned cnt_doc = m_mongodb.collection("parameters").count_documents({});
            if (cnt_doc > n_doc_parameters)
            {
                spdlog::error("The parameters collection of the database has more than " + std::to_string(n_doc_parameters) + " documents.");
                return false;
            }
        }
        catch (const mongocxx::query_exception &e)
        {
            spdlog::debug(e.what());
            return false;
        }
        catch (const mongocxx::operation_exception &e)
        {
            spdlog::debug(e.what());
            return false;
        }
        return true;
    }

} // namespace kios
