#include "kios_utils/context_manager.hpp"

namespace kios
{

    ContextManager::ContextManager()
        : action_ground_dictionary_(),
          default_file_name("context_archive.json"),
          dump_file_name("dump_context_archive.json"),
          file_name(default_file_name)
    {
        if (!read_archive())
        {
            std::cout << "in order to protect the possibly existing file, switch to the dump file name!" << std::endl;
            file_name = dump_file_name;
        }
    }

    /**
     * @brief archive the action node and its context
     *
     * @param action_achive 1: group,  2: id,  3: description,  4. context
     * @return bool
     */
    bool ContextManager::archive_action(const std::tuple<int, int, std::string, nlohmann::json> &action_achive)
    {
        try
        {
            // structrual bind
            const auto &[action_group, action_id, description, context] = action_achive;

            // find group
            auto &group_dictionary = action_ground_dictionary_[action_group];

            // find id
            if (group_dictionary.find(action_id) == group_dictionary.end())
            {
                // insert new
                auto &context_pair = group_dictionary[action_id];
                // insert context pair
                context_pair = std::make_pair(description, context);
            }
            else
            {
                // already exists. this is allowed. do nothing.
                std::cout << "action group: " << action_group
                          << ", action_id: " << action_id
                          << ", description: " << description
                          << ", this action already exists. do nothing..." << std::endl;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        return true;
    }

    /**
     * @brief store the archive to json file
     *
     * @return true
     * @return false
     */
    bool ContextManager::store_archive()
    {
        try
        {
            nlohmann::json j;
            for (const auto &[action_group, group_dictionary] : action_ground_dictionary_)
            {
                for (const auto &[action_id, context_pair] : group_dictionary)
                {
                    auto &[description, context] = context_pair;
                    j[std::to_string(action_group)][std::to_string(action_id)] = {{"description", description}, {"context", context}};
                }
            }

            std::ofstream o(file_name);
            o << j.dump(4); // dump with indentation of 4
            o.close();
        }
        catch (...)
        {
            std::cerr << "FAILED WHEN WRITING INTO FILE " << file_name << "!" << std::endl;
            return false;
        }
        return true;
    }

    /**
     * @brief read
     *
     * @return true
     * @return false
     */
    bool ContextManager::read_archive()
    {
        nlohmann::json j;
        std::ifstream i(file_name);

        if (i.fail())
        {
            // file open failed.
            std::cerr << "FAILED WHEN OPENING THE JSON FILE " << file_name << "!" << std::endl;
            return false;
        }

        try
        {
            i >> j;
            i.close();

            for (const auto &[action_group_str, group_dictionary_json] : j.items())
            {
                int action_group = std::stoi(action_group_str);
                for (const auto &[action_id_str, value_json] : group_dictionary_json.items())
                {
                    int action_id = std::stoi(action_id_str);
                    std::string description = value_json["description"];
                    nlohmann::json context = value_json["context"];

                    action_ground_dictionary_[action_group][action_id] = std::make_pair(description, context);
                }
            }
        }
        catch (...)
        {
            return false;
        }
        return true;
    }

} // namespace kios
