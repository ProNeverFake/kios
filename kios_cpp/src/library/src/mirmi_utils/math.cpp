#include "mirmi_utils/math.hpp"
#include "Eigen/Dense"

namespace mirmi_utils
{

    Eigen::Matrix<double, 6, 1> rotate_vector(const Eigen::Matrix<double, 6, 1> &v_in, const Eigen::Matrix<double, 3, 3> &M)
    {
        Eigen::Vector3d v_in_t = {v_in[0], v_in[1], v_in[2]};
        Eigen::Vector3d v_in_r = {v_in[3], v_in[4], v_in[5]};
        Eigen::Vector3d v_out_t = M * v_in_t;
        Eigen::Vector3d v_out_r = M * v_in_r;
        Eigen::VectorXd v_out(6);
        v_out << v_out_t[0], v_out_t[1], v_out_t[2], v_out_r[0], v_out_r[1], v_out_r[2];
        return v_out;
    }

    Eigen::Matrix<double, 4, 4> rotate_matrix(const Eigen::Matrix<double, 4, 4> &M_in, const Eigen::Matrix<double, 3, 3> &M_rot)
    {
        Eigen::Matrix<double, 4, 3> T_rot_1;
        Eigen::Matrix<double, 4, 4> T_rot_2;
        Eigen::Matrix<double, 1, 3> last_row;
        last_row << 0, 0, 0;
        T_rot_1 << M_rot,
            last_row;
        Eigen::Matrix<double, 4, 1> pos;
        pos << 0, 0, 0, 1;
        T_rot_2 << T_rot_1, pos;
        return T_rot_2 * M_in;
    }

    Eigen::Matrix<double, 3, 3> invert_matrix(const Eigen::Matrix<double, 3, 3> &M)
    {
        return M.transpose();
    }

    Eigen::Matrix<double, 4, 4> invert_transformation_matrix(const Eigen::Matrix<double, 4, 4> &M)
    {
        Eigen::Matrix<double, 3, 3> R_inv = M.block<3, 3>(0, 0).transpose();
        Eigen::Matrix<double, 3, 1> t_inv = -R_inv * M.block<3, 1>(0, 3);
        Eigen::Matrix<double, 3, 4> T_inv_tmp;
        T_inv_tmp << R_inv, t_inv;
        Eigen::Matrix<double, 1, 4> last_row;
        last_row << 0, 0, 0, 1;
        Eigen::Matrix<double, 4, 4> T_inv;
        T_inv << T_inv_tmp,
            last_row;
        return T_inv;
    }

    Eigen::Matrix<double, 4, 4> concatenate_matrix(const Eigen::Matrix<double, 3, 3> R, const Eigen::Matrix<double, 3, 1> v)
    {
        Eigen::Matrix<double, 1, 4> h;
        h << 0, 0, 0, 1;
        Eigen::Matrix<double, 3, 4> T_tmp;
        Eigen::Matrix<double, 4, 4> T;
        T_tmp << R, v;
        T << T_tmp, h;
        return T;
    }

    Eigen::Matrix<double, 3, 3> eulerRPY_to_mat(double alpha, double beta, double gamma)
    {
        gamma *= M_PI / 180.0;
        beta *= M_PI / 180.0;
        alpha *= M_PI / 180.0;

        Eigen::Matrix<double, 3, 3> R_z;
        R_z << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        Eigen::Matrix<double, 3, 3> R_y;
        R_y << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        Eigen::Matrix<double, 3, 3> R_x;
        R_x << 1, 0, 0, 0, 1, 0, 0, 0, 1;

        if (gamma != 0)
        {
            R_z << cos(gamma), sin(gamma), 0,
                -sin(gamma), cos(gamma), 0,
                0, 0, 1;
        }
        if (beta != 0)
        {
            R_y << cos(beta), 0, -sin(beta),
                0, 1, 0,
                sin(beta), 0, cos(beta);
        }
        if (alpha != 0)
        {
            R_x << 1, 0, 0,
                0, cos(alpha), sin(alpha),
                0, -sin(alpha), cos(alpha);
        }
        Eigen::Matrix<double, 3, 3> tmp = R_y * R_x;
        return R_z * tmp;
    }

    bool is_orthonormal(Eigen::Matrix<double, 3, 3> M)
    {
        for (unsigned i = 0; i < 3; i++)
        {
            if (fabs(sqrt(pow(M(0, i), 2) + pow(M(1, i), 2) + pow(M(2, i), 2)) - 1) > 1e-3)
            {
                return false;
            }
        }
        Eigen::Matrix<double, 3, 1> y_d = M.block<3, 1>(0, 2).cross(M.block<3, 1>(0, 0));
        for (unsigned i = 0; i < 3; i++)
        {
            if (fabs(y_d(i) - M(i, 1)) > 1e-3)
            {
                return false;
            }
        }
        return true;
    }

    double get_linear_distance(const Eigen::Matrix<double, 4, 4> &T_1, const Eigen::Matrix<double, 4, 4> &T_2)
    {
        return (T_1.block<3, 1>(0, 3) - T_2.block<3, 1>(0, 3)).norm();
    }

    double get_angular_distance(const Eigen::Matrix<double, 4, 4> &T_1, const Eigen::Matrix<double, 4, 4> &T_2)
    {
        return acos(((T_1.block<3, 3>(0, 0).transpose() * T_2.block<3, 3>(0, 0)).trace() - 1) / 2);
    }

    double fRand(double fMin, double fMax)
    {
        double f = static_cast<double>(rand()) / RAND_MAX;
        return fMin + f * (fMax - fMin);
    }

    Eigen::Matrix<double, 3, 3> build_rotation_matrix(const Eigen::Matrix<double, 3, 1> &v1, const Eigen::Matrix<double, 3, 1> &v2)
    {
        Eigen::Matrix<double, 3, 1> p = v1.cross(v2);
        double s = p.norm();
        double c = v1.dot(v2);

        Eigen::Matrix<double, 3, 3> v_x;
        v_x << 0, -p(2), p(1), p(2), 0, -p(0), -p(1), p(0), 0;
        Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Identity() + v_x + (v_x * v_x) * (1 - c) / pow(s, 2);
        R.block<3, 1>(0, 0).normalize();
        R.block<3, 1>(0, 1).normalize();
        R.block<3, 1>(0, 2).normalize();
        return R;
    }

} // namespace mirmi_utils
