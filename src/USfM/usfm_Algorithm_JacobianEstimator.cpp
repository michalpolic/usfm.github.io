// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Algorithm_JacobianEstimator.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/


#include "USfM/usfm_Algorithm_JacobianEstimator.hpp"
#include "USfM/usfm_InputCovarinace_Factory.hpp"

namespace usfm {

	void Algorithm_JacobianEstimator::compute(Scene& scene, Statistic& statistic) {
		// initialize the projections, camera and image models and sort parameters
		init(scene);
		//std::cout.precision(17);

		// test of the reprojection error for I_DIVISON2 model
		double res[2];
		scene._projections[0]->computeResidual(res); 
		scene._projections[10]->computeResidual(res);
		scene._projections[50]->computeResidual(res);

		// find radial distortion parameters
		if (scene._settings._run_opt_radial)
			optimizeRadialParams(scene, statistic);

		// jacobian of the projection function
		computeJacobian(scene, statistic);
		//std::cout << scene._jacobian << "\n\n\n";
		//std::cout << "J: " << isNan(scene._jacobian) << "\n";

		// the nullspace
		DM &H = scene._H;		// the nullspace of jacobian of projection function
		if (scene._settings._return_nullspace)
		  computeJacobianNullspace(scene, H);

		// close and clear
		finish();
	}


	// Compute the unknown radial parameters
	void Algorithm_JacobianEstimator::optimizeRadialParams(Scene& scene, Statistic& statistic) {
		// convert Scene to ceres::Problem
		ceres::Problem problem;
		composeCeresProblem(scene, statistic, &problem);

		// fix all the non-radial parameters
		fixNonRadialParameters(scene, statistic, &problem);

		// setup the order of Jacobian blocks
		std::vector<double*> parameter_blocks;
		setupJacobianBlocksOrder(scene, parameter_blocks);

		// ceres options for computing the Jacobian
		double cost = 0.0;
		ceres::Problem::EvaluateOptions evalOpt;
		evalOpt.parameter_blocks = parameter_blocks;
		evalOpt.apply_loss_function = false;

		// optimize - we assume the problem close to optimal solution
		ceres::Solver::Options options = defaultOprimizationOptions();
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		//std::cout << summary.FullReport() << "\n";
	}

	void Algorithm_JacobianEstimator::fixNonRadialParameters(Scene& scene, Statistic& statistic, ceres::Problem* problem) {
		for (auto &p3D : scene._points3D)
			problem->SetParameterBlockConstant(p3D.second._X);
		for (auto &i : scene._images)
			problem->SetParameterBlockConstant(i.second._parameters.data());
		for (auto &c : scene._cameras){
			if (c.second.getModel() == eSimplePinhole || c.second.getModel() == ePinhole){
				problem->SetParameterBlockConstant(c.second._parameters.data());
			}else {
				problem->SetParameterization(c.second._parameters.data(), 
					new ceres::SubsetParameterization(c.second.numParams(), getNotRadialParamsOffset(c.second)));
			}
		}
	}

	std::vector<int> Algorithm_JacobianEstimator::getNotRadialParamsOffset(Camera& camera) {
		std::vector<int> offsets;
		std::vector<ECameraParameter> rd_params{ e_f, e_fx, e_fy, e_cx, e_cy, e_img_width, e_img_height };
		for (int i = 0; i < rd_params.size(); ++i) {
			if (camera._offset_in_parameters.find(rd_params[i]) != camera._offset_in_parameters.end() &&
				camera._offset_in_parameters.at(rd_params[i]) < camera.numParams())
				offsets.push_back(camera._offset_in_parameters.at(rd_params[i]));
		}
		return offsets;
	}

}