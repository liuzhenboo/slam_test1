#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>

using namespace Eigen;
using namespace std;

struct ReprojectionError3D
{
    ReprojectionError3D(double observed_u, double observed_v, Eigen::Matrix4d TFpose)
        : observed_u(observed_u), observed_v(observed_v)
    {
        observed_TFpose_wv = TFpose;
    }

    template <typename T>
    bool operator()(const T *const vc_q, const T *const point, T *residuals) const
    {
        Eigen::Matrix<T, 3, 3> rotation;
        Eigen::Quaternion<T> q(vc_q[0], vc_q[1], vc_q[2], vc_q[3]);
        rotation = q.toRotationMatrix();

        Eigen::Matrix<T, 4, 4> T_vc = Eigen::Matrix<T, 4, 4>::Zero();
        // T_vc.template block<3, 3>(0, 0) = rotation;
        //T_vc.template block<3, 1>(0, 3) = (T)external_t;
        T_vc(0, 0) = rotation(0, 0);
        T_vc(0, 1) = rotation(0, 1);
        T_vc(0, 2) = rotation(0, 2);
        T_vc(1, 0) = rotation(1, 0);
        T_vc(1, 1) = rotation(1, 1);
        T_vc(1, 2) = rotation(1, 2);
        T_vc(2, 0) = rotation(2, 0);
        T_vc(2, 1) = rotation(2, 1);
        T_vc(2, 2) = rotation(2, 2);

        T_vc(0, 3) = (T)external_t(0, 0);
        T_vc(1, 3) = (T)external_t(1, 0);
        T_vc(2, 3) = (T)external_t(2, 0);

        T_vc(3, 3) = (T)1.0;

        Eigen::Matrix<T, 4, 1> Point;
        Point(0, 0) = point[0];
        Point(1, 0) = point[1];
        Point(2, 0) = point[2];
        Point(3, 0) = (T)1.0;

        Eigen::Matrix<T, 4, 1> res;
        res = T_vc.inverse() * observed_TFpose_wv.inverse() * Point;

        T xp = res[0] / res[2];
        T yp = res[1] / res[2];

        residuals[0] = xp - T(observed_u);
        residuals[1] = yp - T(observed_v);
        return true;
    }

    static ceres::CostFunction *Create(const double observed_x,
                                       const double observed_y, const Eigen::Matrix4d TFpose)
    {
        return (new ceres::AutoDiffCostFunction<
                ReprojectionError3D, 2, 4, 3>(
            new ReprojectionError3D(observed_x, observed_y, TFpose)));
    }

    double observed_u;
    double observed_v;
    Eigen::Matrix4d observed_TFpose_wv;
    Eigen::Vector3d external_t{1.6, -0.8, -0.5};
};