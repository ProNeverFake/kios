// Copyright (c) 2020 - present, Lars Johannsmeier
// All rights reserved.
// contact: lars.johannsmeier@gmail.com

#pragma once

#include <iostream>
#include <array>
#include <optional>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "nlohmann/json.hpp"
#include "spdlog/spdlog.h"

namespace mirmi_utils
{

    /**
     * Appends the given Eigen::Matrix value to the indicated json array.
     * @param[in] paramJ Json value.
     * @param[in] param Eigen::Matrix value that is appended to the json value.
     * @return True if operation is successful, false otherwise.
     */
    template <typename T, int S1, int S2>
    bool append_json_array(nlohmann::json &paramJ, const Eigen::Matrix<T, S1, S2> &param)
    {
        try
        {
            for (unsigned i = 0; i < param.cols(); i++)
            {
                for (unsigned j = 0; j < param.rows(); j++)
                {
                    paramJ.push_back(param(j, i));
                }
            }
            return true;
        }
        catch (const nlohmann::detail::type_error &e)
        {
            spdlog::error(e.what());
            return false;
        }
    }

    /**
     * Appends the given std::array value to the indicated json array.
     * @param[in] paramJ Json value.
     * @param[in] param std::array value that is appended to the json value.
     * @return True if operation is successful, false otherwise.
     */
    template <typename T, std::size_t S>
    bool append_json_array(nlohmann::json &paramJ, const std::array<T, S> &param)
    {
        try
        {
            for (unsigned i = 0; i < param.size(); i++)
            {
                paramJ.push_back(param[i]);
            }
            return true;
        }
        catch (const nlohmann::detail::type_error &e)
        {
            spdlog::error(e.what());
            return false;
        }
    }

    /**
     * Converts the given Eigen::Matrix value to a json array.
     * @param[in] paramJ Json value.
     * @param[in] param Eigen::Matrix value that is converted to a json value.
     * @return True if operation is successful, false otherwise.
     */
    template <typename T, int S1, int S2>
    bool write_json_array(nlohmann::json &paramJ, const Eigen::Matrix<T, S1, S2> &param)
    {
        paramJ.clear();
        try
        {
            for (unsigned i = 0; i < param.cols(); i++)
            {
                for (unsigned j = 0; j < param.rows(); j++)
                {
                    paramJ.push_back(param(j, i));
                }
            }
        }
        catch (const nlohmann::detail::type_error &e)
        {
            spdlog::error(e.what());
            return false;
        }
        return true;
    }

    /**
     * Converts the given std::array value to a json array.
     * @param[in] paramJ Json value.
     * @param[in] param std::array value that is converted to a json value.
     * @return True if operation is successful, false otherwise.
     */
    template <typename T, std::size_t S>
    bool write_json_array(nlohmann::json &paramJ, const std::array<T, S> &param)
    {
        paramJ.clear();
        try
        {
            for (unsigned i = 0; i < param.size(); i++)
            {
                paramJ.push_back(param[i]);
            }
        }
        catch (const nlohmann::detail::type_error &e)
        {
            spdlog::error(e.what());
            return false;
        }
        return true;
    }

    /**
     * Converts the given std::vector value to a json array.
     * @param[in] paramJ Json value.
     * @param[in] param std::vector value that is converted to a json value.
     * @return True if operation is successful, false otherwise.
     */
    template <typename T>
    bool write_json_array(nlohmann::json &paramJ, const std::vector<T> &param)
    {
        paramJ.clear();
        try
        {
            for (unsigned i = 0; i < param.size(); i++)
            {
                paramJ.push_back(param[i]);
            }
        }
        catch (const nlohmann::detail::type_error &e)
        {
            spdlog::error(e.what());
            return false;
        }
        return true;
    }

    /**
     * Converts the indicated json value into an Eigen::Matrix.
     * @param paramJ Json value to read from.
     * @param param Target Eigen::Matrix type. Has to be of matching size with the json value.
     * @return True if operation is successful, false otherwise.
     */
    template <typename T>
    bool read_json_param(const nlohmann::json &paramJ, const char *key, T &param)
    {
        try
        {
            if (!paramJ.contains(key))
            {
                return false;
            }
            if (paramJ[key].is_null())
            {
                return false;
            }
            paramJ[key].get_to(param);
        }
        catch (const nlohmann::detail::parse_error &e)
        {
            spdlog::error(e.what());
            return false;
        }
        catch (const nlohmann::detail::exception &e)
        {
            spdlog::error(e.what());
            return false;
        }
        return true;
    }

    template <typename T>
    std::optional<T> from_json(const nlohmann::json &paramJ, const char *key)
    {
        try
        {
            if (!paramJ.contains(key))
            {
                return {};
            }
            if (paramJ[key].is_null())
            {
                return {};
            }
            T param;
            paramJ[key].get_to(param);
            return param;
        }
        catch (const nlohmann::detail::parse_error &e)
        {
            spdlog::error(e.what());
            return {};
        }
        catch (const nlohmann::detail::exception &e)
        {
            spdlog::error(e.what());
            return {};
        }
    }

    /**
     * Converts the indicated json value into an Eigen::Matrix.
     * @param paramJ Json value to read from.
     * @param param Target Eigen::Matrix type. Has to be of matching size with the json value.
     * @return True if operation is successful, false otherwise.
     */
    template <typename T, int S1, int S2>
    bool read_json_param(const nlohmann::json &paramJ, Eigen::Matrix<T, S1, S2> &param)
    {
        try
        {
            if (!paramJ.is_array())
            {
                return false;
            }
            if (paramJ.size() != S1 * S2)
            {
                return false;
            }
            for (unsigned i = 0; i < param.cols(); i++)
            {
                for (unsigned j = 0; j < param.rows(); j++)
                {
                    paramJ[i * static_cast<unsigned>(param.rows()) + j].get_to(param(j, i));
                }
            }
        }
        catch (const nlohmann::detail::parse_error &e)
        {
            spdlog::error(e.what());
            return false;
        }
        catch (const nlohmann::detail::exception &e)
        {
            spdlog::error(e.what());
            return false;
        }
        return true;
    }

    /**
     * Converts the indicated json value into an Eigen::Matrix.
     * @param paramJ Json value to read from.
     * @param param Target Eigen::Matrix type. Has to be of matching size with the json value.
     * @return True if operation is successful, false otherwise.
     */
    template <typename T, int S1, int S2>
    bool read_json_param(const nlohmann::json &paramJ, const char *key, Eigen::Matrix<T, S1, S2> &param)
    {
        try
        {
            if (!paramJ.contains(key))
            {
                return false;
            }
            if (paramJ[key].is_null() || paramJ[key].empty())
            {
                return false;
            }
            if (paramJ[key].size() != param.rows() * param.cols())
            {
                spdlog::error("Can not copy json parameter, expected size (" + std::to_string(param.rows() * param.cols()) + ") is different from actual one (" + std::to_string(paramJ.size()) + ").");
                return false;
            }
            for (unsigned i = 0; i < param.cols(); i++)
            {
                for (unsigned j = 0; j < param.rows(); j++)
                {
                    paramJ[key][i * static_cast<unsigned>(param.rows()) + j].get_to(param(j, i));
                }
            }
        }
        catch (const nlohmann::detail::parse_error &e)
        {
            spdlog::error(e.what());
            return false;
        }
        catch (const nlohmann::detail::exception &e)
        {
            spdlog::error(e.what());
            return false;
        }
        return true;
    }

    /**
     * Converts the indicated json value into an std::array.
     * @param paramJ Json value to read from.
     * @param param Target std::array type. Has to be of matching size with the json value.
     * @return True if operation is successful, false otherwise.
     */
    template <typename T>
    bool read_json_param(const nlohmann::json &paramJ, const char *key, std::vector<T> &param)
    {
        try
        {
            if (!paramJ.contains(key))
            {
                return false;
            }
            if (paramJ[key].is_null() || paramJ[key].empty())
            {
                return false;
            }
            param.resize(paramJ[key].size());
            for (unsigned i = 0; i < paramJ[key].size(); i++)
            {
                paramJ[key][i].get_to(param[i]);
            }
        }
        catch (const nlohmann::detail::parse_error &e)
        {
            spdlog::error(e.what());
            return false;
        }
        catch (const nlohmann::detail::exception &e)
        {
            spdlog::error(e.what());
            return false;
        }
        return true;
    }

    template <typename T, int S1, int S2>
    nlohmann::json from_eigen(const Eigen::Matrix<T, S1, S2> eigen_object, bool column_major = true)
    {
        nlohmann::json json_array;
        if (column_major)
        {
            for (unsigned i = 0; i < S2; i++)
            {
                for (unsigned j = 0; j < S1; j++)
                {
                    json_array.emplace_back(eigen_object(j, i));
                }
            }
        }
        else
        {
            for (unsigned i = 0; i < S1; i++)
            {
                for (unsigned j = 0; j < S2; j++)
                {
                    json_array.emplace_back(eigen_object(i, j));
                }
            }
        }
        return json_array;
    }

    /**
     * Searches for a specific key in a given nlohmann::json object. Note that sub elements of the json object are not searched.
     * @param json Json value in which to search.
     * @param key Key to search for.
     * @return Returns true if key is found, false otherwise.
     */
    [[deprecated("Use the built-in find method of json objects instead.")]] bool find_json_value(const nlohmann::json &json, const char *key);

    bool overwrite_valid_json(const nlohmann::json &source, nlohmann::json &sink);

} // namespace mirmi_utils
