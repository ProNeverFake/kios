// Copyright (c) 2020 - present, Lars Johannsmeier
// All rights reserved.
// contact: lars.johannsmeier@gmail.com

#pragma once

#include <iostream>
#include <array>
#include <math.h>

#include "Eigen/Core"

namespace mirmi_utils
{

    Eigen::Matrix<double, 6, 1> rotate_vector(const Eigen::Matrix<double, 6, 1> &v_in, const Eigen::Matrix<double, 3, 3> &M);

    Eigen::Matrix<double, 4, 4> rotate_matrix(const Eigen::Matrix<double, 4, 4> &M_in, const Eigen::Matrix<double, 3, 3> &M_rot);

    [[deprecated("Uneccessary, will be removed in the future. Instead just call M.transpose() provided by the eigen library.")]] Eigen::Matrix<double, 3, 3> invert_matrix(const Eigen::Matrix<double, 3, 3> &M);
    /**
     * Inverts a given 4x4 transformation matrix.
     * @param M 4x4 homogeneous transformation matrix.
     * @return Inverted 4x4 matrix.
     */
    Eigen::Matrix<double, 4, 4> invert_transformation_matrix(const Eigen::Matrix<double, 4, 4> &M);

    /**
     * Builds a 4x4 transformation matrix from a 3x3 rotation matrix and a 3x1 position vector.
     * @param R 3x3 rotation matrix.
     * @param v 3x1 position vector.
     * @return 4x4 homogeneous transformation matrix.
     */
    Eigen::Matrix<double, 4, 4> concatenate_matrix(const Eigen::Matrix<double, 3, 3> R, const Eigen::Matrix<double, 3, 1> v);

    /**
     * Creates a 3x3 rotation matrix from three angles according to the RPY-euler convention.
     * @param alpha Angle around x-axis.
     * @param beta Angle around y-axis.
     * @param gamma Angle around z-axis.
     * @return 3x3 rotation matrix.
     */
    Eigen::Matrix<double, 3, 3> eulerRPY_to_mat(double alpha, double beta, double gamma);

    /**
     * Checks whether the given 3x3 matrix is orthonormal.
     * @param M 3x3 matrix.
     * @return True if M is orthonormal, false otherwise.
     */
    bool is_orthonormal(Eigen::Matrix<double, 3, 3> M);

    /**
     * Calculates the 2-norm from a given vector.
     * @param v
     * @return
     */
    template <int S>
    double norm_2(const Eigen::Matrix<double, S, 1> v)
    {
        double n = 0;
        for (unsigned i = 0; i < v.rows(); i++)
        {
            n += pow(v(i), 2);
        }
        return sqrt(n);
    }

    /**
     * @brief Calculates the linear distance between two transformation matrices.
     * @param T_1 First matrix
     * @param T_2 Second matrix
     * @return A distance value in meters.
     */
    double get_linear_distance(const Eigen::Matrix<double, 4, 4> &T_1, const Eigen::Matrix<double, 4, 4> &T_2);

    /**
     * @brief Calculates the angular distance between two transformation matrices.
     * @param T_1 First matrix
     * @param T_2 Second matrix
     * @return The difference angle in radians.
     */
    double get_angular_distance(const Eigen::Matrix<double, 4, 4> &T_1, const Eigen::Matrix<double, 4, 4> &T_2);

    /**
     * Gives a random number within an inclusive interval according to the uniform distribution.
     * @param fMin Maximum value of interval.
     * @param fMax Minimum value of interval.
     * @return A scalar value randomly sampled from the given interval.
     */
    double fRand(double fMin, double fMax);

    /**
     * Retuns the sign of a given value.
     * @param[in] A scalar value.
     * @return +1 if sign is positive, -1 if negative, 0 if val is 0.
     */
    template <typename T>
    int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    Eigen::Matrix<double, 3, 3> build_rotation_matrix(const Eigen::Matrix<double, 3, 1> &v1, const Eigen::Matrix<double, 3, 1> &v2);

} // namespace mirmi_utils
