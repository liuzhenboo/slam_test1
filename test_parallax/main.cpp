#include <iostream>
#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "ceres_error.h"
#include <cstdlib>
using namespace std;
const double DEG_TO_ARC = 0.0174532925199433;
int main()
{
	//外参
	// 右前 yaw-152.5, pitch0.72, roll-87.8
	double yaw = 132.5;
	double pitch = 0.72;
	double roll = -87.8;
	double yaw1 = 139.5;
	double pitch1 = 10.72;
	double roll1 = -98.8;
	Eigen::Matrix3d rotation_vc;
	Eigen::Vector3d t_vc;
	rotation_vc = Eigen::AngleAxisd(yaw * DEG_TO_ARC, Eigen::Vector3d ::UnitZ()) *
				  Eigen::AngleAxisd(pitch * DEG_TO_ARC, Eigen::Vector3d::UnitY()) *
				  Eigen::AngleAxisd(roll * DEG_TO_ARC, Eigen::Vector3d::UnitX());
	t_vc << 1.6, -0.8, -0.5;
	Eigen::Matrix4d Tvc = Eigen::Matrix4d::Identity();
	Tvc.block<3, 3>(0, 0) = rotation_vc;
	Tvc.block<3, 1>(0, 3) = t_vc;

	Eigen::Matrix3d rotation_state;
	rotation_state = Eigen::AngleAxisd(yaw1 * DEG_TO_ARC, Eigen::Vector3d ::UnitZ()) *
					 Eigen::AngleAxisd(pitch1 * DEG_TO_ARC, Eigen::Vector3d::UnitY()) *
					 Eigen::AngleAxisd(roll1 * DEG_TO_ARC, Eigen::Vector3d::UnitX());
	Eigen::Quaterniond q(rotation_state);
	double q_state[4] = {0.0};
	q_state[0] = q.w();
	q_state[1] = q.x();
	q_state[2] = q.y();
	q_state[3] = q.z();

	// pose
	vector<Eigen::Matrix4d> T;
	vector<Eigen::Matrix4d> T_M;

	vector<double> angles_z(3);
	double r = 11;
	angles_z.push_back(0);
	angles_z.push_back(10);
	// angles_z.push_back(2);
	// angles_z.push_back(3);
	// angles_z.push_back(4);
	// angles_z.push_back(5);
	// angles_z.push_back(6);
	// angles_z.push_back(7);
	// angles_z.push_back(8);
	// angles_z.push_back(9);

	for (unsigned int i = 0; i < angles_z.size(); i++)
	{
		Eigen::Matrix3d rotation;
		Eigen::Vector3d t;
		rotation = Eigen::AngleAxisd(-1.0 * angles_z[i] * DEG_TO_ARC, Eigen::Vector3d::UnitZ()) *
				   Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
				   Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
		// if (i == 0)
		// {
		// 	t << 0, r, 0;
		// }
		// else if (i == 1)
		// {
		// 	t << 0.5, r, 0;
		// }
		// else
		// {
		// 	t << 1.6, r + 0.0, 0;
		// }
		t << r * sin(angles_z[i]), r * cos(angles_z[i]), 0;

		Eigen::Matrix4d T_temp = Eigen::Matrix4d::Identity();
		T_temp.block<3, 3>(0, 0) = rotation;
		T_temp.block<3, 1>(0, 3) = t;
		T_temp(3, 3) = 1.0;
		T.push_back(T_temp); // Twv;

		Eigen::Matrix3d rotationm;
		Eigen::Vector3d tm;
		double ey = 0.0; // ((rand() % 10) / 10000000.0);

		double ep = 0.0; //((rand() % 10) / 100000000.0); // yaw
		double er = 0.0;
		//((rand() % 10) / 1000000000.0);

		rotationm = Eigen::AngleAxisd(-1.0 * (angles_z[i] + ey) * DEG_TO_ARC, Eigen::Vector3d::UnitZ()) *
					Eigen::AngleAxisd(ep, Eigen::Vector3d::UnitY()) *
					Eigen::AngleAxisd(er, Eigen::Vector3d::UnitX());
		tm = t;
		Eigen::Matrix4d T_tempm = Eigen::Matrix4d::Identity();
		T_tempm.block<3, 3>(0, 0) = rotationm;
		T_tempm.block<3, 1>(0, 3) = tm;
		T_tempm(3, 3) = 1.0;
		T_M.push_back(T_tempm);
	}

	//points
	vector<Eigen::Vector3d> points;
	points.push_back({25, -10, 30});
	points.push_back({46, 0, 2});
	points.push_back({18, -25, 5});
	points.push_back({30, -3, 8});
	points.push_back({39, -7, 12});
	points.push_back({10, -3, 10});
	points.push_back({91, -34, 0});
	points.push_back({33, -1, 25});
	points.push_back({20, -3, 8});
	points.push_back({10, -1, 12});

	std::vector<std::vector<double>> points_state;
	for (auto it : points)
	{
		std::vector<double> s_p;
		s_p.push_back(it(0, 0) + 0.5);
		s_p.push_back(it(1, 0) + 0.5);
		s_p.push_back(it(2, 0) - 0.5);
		points_state.push_back(s_p);
	}

	ceres::QuaternionParameterization *quaternionParameterization = new ceres::QuaternionParameterization();
	ceres::Problem problem;
	problem.AddParameterBlock(q_state, 4, quaternionParameterization);
	// 构造图像观测
	std::vector<std::vector<Eigen::Vector2d>> measures(10); //代表某个地图点在第几帧观测的uv
	for (unsigned int i = 0; i < 10; i++)
	{
		for (unsigned int j = 0; j < angles_z.size(); j++)
		{
			// if (j == 2)
			// 	continue;
			Eigen::Vector3d uv;
			uv = (Tvc.inverse() * T[j].inverse() * points[i].homogeneous()).block<3, 1>(0, 0);
			uv = uv / uv(2, 0);
			uv(0, 0) += ((rand() % 10) / 500.0);
			uv(1, 0) += ((rand() % 10) / 500.0);
			measures[i].push_back(uv.block<2, 1>(0, 0));

			ceres::CostFunction *cost_function;
			cost_function = ReprojectionError3D::Create(uv(0, 0), uv(1, 0), T_M[j]);

			//ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
			problem.AddResidualBlock(cost_function, nullptr, q_state, points_state[i].data());
		}
	}
	ceres::Solver::Options options;
	ceres::Solver::Summary summary;
	options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
	options.minimizer_progress_to_stdout = true;
	options.max_num_iterations = 1000;
	ceres::Solve(options, &problem, &summary);
	//std::cout << summary.FullReport() << "\n";

	Eigen::Quaterniond q_f(q_state[0], q_state[1], q_state[2], q_state[3]);
	Eigen::Vector3d eulerAngle = q_f.matrix().eulerAngles(2, 1, 0);
	eulerAngle *= (180.0 / 3.14159);
	std::cout << eulerAngle[0] << ", " << eulerAngle[1] << ", " << eulerAngle[2] << std::endl;
	return 0;
}