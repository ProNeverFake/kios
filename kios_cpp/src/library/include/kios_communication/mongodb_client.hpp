#pragma once

#include "mongocxx/client.hpp"
#include "mongocxx/instance.hpp"
#include "mongocxx/database.hpp"
#include "mongocxx/collection.hpp"

#include "nlohmann/json.hpp"

#include <mutex>
#include <set>
#include <string>
#include <map>

namespace kios
{

    class MongodbClient
    {
    public:
        MongodbClient(const std::string &database, unsigned port = 27017);

        bool read_document(const std::string &name, const std::string &collection, nlohmann::json &descr);
        bool read_documents(const std::string &collection, std::set<nlohmann::json> &docs);
        bool write_document(const std::string &name, const std::string &collection, const nlohmann::json &descr, bool overwrite);
        bool write_documents(const std::string &collection, const std::set<nlohmann::json> &docs, bool overwrite);
        bool make_document_consistent(const std::string &name, std::string collection, const nlohmann::json &template_json);
        bool health_check() const;

    private:
        mongocxx::instance m_instance;
        mongocxx::client m_client;
        mongocxx::database m_mongodb;
        std::map<std::string, mongocxx::collection> m_collections;

        std::mutex m_mutex_db_access;
    };

} // namespace kios
