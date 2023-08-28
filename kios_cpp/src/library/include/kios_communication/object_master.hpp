/**
 * @file lt_memory.hpp
 * @author your name (you@domain.com)
 * @brief long time memory.
 * * store and read data from mongo DB for long time utilization.
 * @version 0.1
 * @date 2023-06-11
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once

#include "kios_communication/mongodb_client.hpp"
#include "kios_utils/object.hpp"
#include "kios_utils/parameters.hpp"

#include "nlohmann/json.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include <string>
#include <memory>
#include <unordered_map>

namespace kios
{

    class ObjectMaster
    {
    public:
        ObjectMaster(std::string robot_arm, unsigned database_port = 27017);
        bool is_ok() const;
        bool initialize(unsigned robot_configuration);
        bool load_default_parameters(nlohmann::json &parameters);

        bool load_environment();
        bool upload_environment_element(const Object &element);

        bool update_database();

        unsigned m_database_port;

        std::shared_ptr<std::unordered_map<std::string, Object>> get_object_dictionary();

    private:
        bool make_database_consistent();
        bool make_default_environment_consistent();

        MongodbClient m_mongodb_client;

        std::shared_ptr<std::unordered_map<std::string, Object>> object_dictionary_ptr_;
    };

} // namespace kios
