// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_InputCovarinace_Squared_Residuals.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/


#include "USfM/usfm_InputCovarinace_Squared_Residuals.hpp"

namespace usfm {

	void InputCovariance_SquaredResiduals::compute(Scene& scene, Statistic& statistic) {
		int num_obs = scene._projections.size();
		scene._in_cov = SM(2 * num_obs, 2 * num_obs);
		scene._in_precision_mat = SM(2 * num_obs, 2 * num_obs);

		// compute the sum of squred residuals
		std::vector<double> reproj_errors;
		reprojectionErrors(scene, reproj_errors);

		// set identity matrix from triplets
		std::vector<Tri> cov_triplets;
		std::vector<Tri> precision_triplets;
		cov_triplets.reserve(2 * num_obs);
		precision_triplets.reserve(2 * num_obs);
		for (int i = 0; i < 2 * num_obs; ++i){
			cov_triplets.push_back(Tri(i, i, reproj_errors[i] * reproj_errors[i]));
			precision_triplets.push_back(Tri(i, i, 1/(reproj_errors[i] * reproj_errors[i])));
		}
		scene._in_cov.setFromTriplets(cov_triplets.begin(), cov_triplets.end());
		scene._in_precision_mat.setFromTriplets(precision_triplets.begin(), precision_triplets.end());
	}

}