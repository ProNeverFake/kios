#pragma once

// #include "kios_utils/kios_utils.hpp"
#include "kios_utils/data_type.hpp"
#include <fstream>
#include <memory>
#include <filesystem>

namespace kios
{
    // /**
    //  * @brief the class to manage the parameters of the actions in a map.
    //  *
    //  */
    // class ContextClerk
    // {
    // public:
    //     ContextClerk();

    //     void initialize();

    //     bool archive_action(const NodeArchive &action_archive);

    //     bool store_archive();
    //     bool read_archive();

    // private:
    //     std::string file_name;
    //     std::string default_file_name;
    //     std::string dump_file_name;

    //     std::unique_ptr<DefaultActionContext> default_context_dictionary_ptr_;

    //     std::unordered_map<int, std::unordered_map<int, std::pair<std::string, nlohmann::json>>> action_ground_dictionary_;
    //     // ! ground method
    // };

    class ContextClerk
    {
    public:
        ContextClerk();

        void initialize();

        bool archive_action(const NodeArchive &action_archive);

        bool store_archive();
        bool read_archive();

        nlohmann::json get_context(const NodeArchive &archive) const;

    private:
        std::string file_name;
        std::string default_file_name;
        std::string dump_file_name;

        std::unique_ptr<DefaultActionContext> default_context_dictionary_ptr_;

        std::unordered_map<int, std::unordered_map<int, std::pair<std::string, nlohmann::json>>> action_ground_dictionary_;
        // ! ground method
    };
} // namespace kios
