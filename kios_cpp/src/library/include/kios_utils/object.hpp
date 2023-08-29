#pragma once

#include "nlohmann/json.hpp"
#include "Eigen/Core"

#include <optional>

namespace kios
{

    /**
     * The object class is the internal representation of an object description. Object descriptions are saved in the mongodb databse.
     */
    class Object
    {
    public:
        /**
         * The constructor sets default values for the object properties.
         */
        Object(const std::string name_in);

        /**
         * Transforms the internal object representation into json format.
         * @return Object representation in json format.
         */
        nlohmann::json to_json() const;

        /**
         * Reads an object description from json format.
         * @param p Object description in json format.
         */
        static Object from_json(const nlohmann::json &p);

        void update(const nlohmann::json &p);
        void set_pose(std::optional<double> x, std::optional<double> y, std::optional<double> z, std::optional<Eigen::Matrix<double, 3, 3> > R);

        /**
         * The object id in both internal representation as well as the mongodb database.
         */
        const std::string name;

        /**
         * The object pose in joint space.
         */
        Eigen::Matrix<double, 7, 1> q;

        /**
         * The Cartesian object pose in origin frame.
         */
        Eigen::Matrix<double, 4, 4> O_T_OB;

        /**
         * Transformation matrix from EE frame to object frame.
         */
        Eigen::Matrix<double, 4, 4> OB_T_gp;
        Eigen::Matrix<double, 4, 4> OB_T_TCP;

        /**
         * The object's intertial tensor in object frame.
         */
        Eigen::Matrix<double, 3, 3> OB_I;

        /**
         * The object's mass.
         */
        double mass;

        /**
         * Expected finger width when grasping the object.
         */
        double grasp_width;
        double grasp_force;

        /**
         * The object's geometry description. It can have arbitrary properties that can be represented as scalars and arrays.
         */
        nlohmann::json geometry;
    };

} // namespace kios
