// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_InputCovarinace_Variance_Factor.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/


#include "USfM/usfm_InputCovarinace_Variance_Factor.hpp"

namespace usfm {

	void InputCovariance_VarianceFactor::compute(Scene& scene, Statistic& statistic) {
		int num_obs = scene._projections.size();
		scene._in_cov = SM(2 * num_obs, 2 * num_obs);
		scene._in_precision_mat = SM(2 * num_obs, 2 * num_obs);

		// compute the sum of squred residuals
		std::vector<double> reproj_errors;
		reprojectionErrors(scene, reproj_errors);
		double sum_sqr_reproj_err = 0;
		for (double err : reproj_errors)
			sum_sqr_reproj_err += err * err;

		// variance factor
		double vf = sum_sqr_reproj_err / (2 * num_obs - scene.numParams());
		statistic.addItem("variance_factor", vf);

		// set identity matrix from triplets
		std::vector<Tri> cov_triplets;
		std::vector<Tri> precision_triplets;
		cov_triplets.reserve(2 * num_obs);
		precision_triplets.reserve(2 * num_obs);
		for (int i = 0; i < 2 * num_obs; ++i){
			cov_triplets.push_back(Tri(i, i, vf));
			precision_triplets.push_back(Tri(i, i, 1/vf));
		}
		scene._in_cov.setFromTriplets(cov_triplets.begin(), cov_triplets.end());
		scene._in_precision_mat.setFromTriplets(precision_triplets.begin(), precision_triplets.end());
	}

}