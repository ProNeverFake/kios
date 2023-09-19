#include "kios_utils/context_manager.hpp"

namespace kios
{

    ContextManager::ContextManager()
        : action_ground_dictionary_(),
          default_file_name("context_archive.json"),
          dump_file_name("dump_context_archive.json"),
          file_name(default_file_name)
    //   default_context_dictionary_()
    {
        // ! BBTEST
        auto package_path = ament_index_cpp::get_package_share_directory("kios_cpp");
        std::cout << package_path << std::endl;

        if (!read_archive())
        {
            std::cout << "in order to protect the possibly existing file, switch to the dump file name!" << std::endl;
            file_name = dump_file_name;
        }
    }

    /**
     * @brief archive the action node and its context
     *
     * @param action_achive 1: group,  2: id,  3: description,  4. action_phase
     * @return bool
     */
    bool ContextManager::archive_action(const NodeArchive &action_achive) // ! TESTING
    {
        // try
        // {
        //     // structrual bind
        //     const auto &[action_group, action_id, description, action_phase] = action_achive;

        //     // find group
        //     auto &group_dictionary = action_ground_dictionary_[action_group];
        //     // * fetch the default context with the ap
        //     auto context = default_context_dictionary_.get_default_context(action_phase);
        //     if (!context.has_value())
        //     {
        //         std::cerr << "Failed when archiving new action node " << action_phase_to_str(action_phase).value() << ": default context of this action phase is not defined!" << std::endl;
        //         return false;
        //     }

        //     // find id
        //     if (group_dictionary.find(action_id) == group_dictionary.end())
        //     {
        //         // insert new
        //         auto &context_pair = group_dictionary[action_id];
        //         // insert pair
        //         context_pair = std::make_pair(description, context.value());
        //     }
        //     else
        //     {
        //         // already exists. this is allowed. do nothing.
        //         // ! here check the action phase. if inconsistent, the action is deemed unconductable.

        //         // ! BBTODO
        //         auto &ap_default = context.value()["skill"]["action_context"]["action_phase"]; // exception handled
        //         if (action_phase == static_cast<ActionPhase>(ap_default))
        //         {
        //             std::cout << "action group: " << action_group
        //                       << ", action_id: " << action_id
        //                       << ", description: " << description
        //                       << ", this action already exists. Skip archiving it..." << std::endl;
        //         }
        //         else
        //         {
        //             std::cerr << "action group: " << action_group
        //                       << ", action_id: " << action_id
        //                       << ", description: " << description
        //                       << ", this action already exists but is defined as a different ActionPhase."
        //                       << " The execution of this action is deemed impossible. Please check the initialization process of the action node!" << std::endl;
        //             return false;
        //         }
        //     }
        // }
        // catch (const std::exception &e)
        // {
        //     std::cerr << "THIS" << std::endl;
        //     std::cerr << e.what() << std::endl;
        //     return false;
        // }
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
            std::cerr << "THIS" << std::endl;
            return false;
        }
        return true;
    }

} // namespace kios
