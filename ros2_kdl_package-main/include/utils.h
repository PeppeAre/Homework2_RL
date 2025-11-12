#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>

// Eigen to KDL
inline KDL::Vector toKDL(const Eigen::Vector3d& v)
{
    return KDL::Vector(v(0), v(1), v(2));
}

inline KDL::Rotation toKDL(const Eigen::Matrix3d& m)
{
    return KDL::Rotation(m(0, 0), m(0, 1), m(0, 2),
                         m(1, 0), m(1, 1), m(1, 2),
                         m(2, 0), m(2, 1), m(2, 2));
}

// std::vector to KDL
inline KDL::Frame toKDL(const std::vector<double>& v)
{
    return KDL::Frame(KDL::Rotation::RPY(v[3], v[4], v[5]),
                      KDL::Vector(v[0], v[1], v[2]));
}

// Converte un messaggio ROS Pose in un KDL::Frame.
inline KDL::Frame toKDL(const geometry_msgs::msg::Pose& p)
{
    return KDL::Frame(
        KDL::Rotation::Quaternion(
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w
        ),
        KDL::Vector(
            p.position.x,
            p.position.y,
            p.position.z
        )
    );
}


// KDL to Eigen
inline Eigen::Vector3d toEigen(const KDL::Vector& v)
{
    return Eigen::Vector3d(v(0), v(1), v(2));
}

inline Eigen::Matrix3d toEigen(const KDL::Rotation& m)
{
    Eigen::Matrix3d e;
    e << m(0, 0), m(0, 1), m(0, 2),
         m(1, 0), m(1, 1), m(1, 2),
         m(2, 0), m(2, 1), m(2, 2);
    return e;
}

// std::vector to Eigen
inline Eigen::VectorXd toEigen(const std::vector<double>& v)
{
    Eigen::VectorXd e(v.size());
    for (size_t i = 0; i < v.size(); ++i)
    {
        e(i) = v[i];
    }
    return e;
}

// KDL to std::vector
inline std::vector<double> toStdVector(const KDL::JntArray& q)
{
    std::vector<double> v(q.rows());
    for (unsigned int i = 0; i < q.rows(); ++i)
    {
        v[i] = q(i);
    }
    return v;
}

// Eigen to std::vector
inline std::vector<double> toStdVector(const Eigen::VectorXd& e)
{
    std::vector<double> v(e.size());
    // --- CORREZIONE ERRORE (Warning e Logica) --- //
    for (int i = 0; i < e.size(); ++i) // size_t -> int (per warning)
    {
        v[i] = e(i);
    }
    return v; // Era 'return e;' -> ERRORE
    // --- FINE CORREZIONE --- //
}

// Compute linear error
inline Eigen::Vector3d computeLinearError(const Eigen::Vector3d& p_des, const Eigen::Vector3d& p_cur)
{
    return p_des - p_cur;
}

// Compute orientation error
inline Eigen::Vector3d computeOrientationError(const Eigen::Matrix3d& R_des, const Eigen::Matrix3d& R_cur)
{
    Eigen::Matrix3d R_err = R_des * R_cur.transpose();
    Eigen::Vector3d e_o = 0.5 * (R_err.col(1).cross(R_err.col(2)) +
                                 R_err.col(2).cross(R_err.col(0)) +
                                 R_err.col(0).cross(R_err.col(1)));
    return e_o;
}

// Skew symmetric matrix
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d S;
    S << 0, -v(2), v(1),
         v(2), 0, -v(0),
        -v(1), v(0), 0;
    return S;
}

// Pseudoinverse
inline Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& M, const double& tolerance = 1e-6)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd s = svd.singularValues();
    Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(s.size(), s.size());
    for (int i = 0; i < s.size(); ++i)
    {
        if (s(i) > tolerance)
        {
            S_inv(i, i) = 1.0 / s(i);
        }
    }
    return svd.matrixV() * S_inv * svd.matrixU().transpose();
}

// Gradient joint limits
inline Eigen::VectorXd gradientJointLimits(const Eigen::VectorXd& q, const Eigen::MatrixXd& limits, double& cost)
{
    double k = 0.1;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(q.size());
    cost = 0.0;

    for (int i = 0; i < q.size(); ++i)
    {
        double q_min = limits(i, 0);
        double q_max = limits(i, 1);
        double q_avg = (q_min + q_max) / 2.0;
        double err = q(i) - q_avg;
        double range = q_max - q_min;

        // Cost function
        cost += (k * err * err) / (range * range);

        // Gradient
        gradient(i) = (2 * k * err) / (range * range);
    }

    return gradient;
}

#endif
