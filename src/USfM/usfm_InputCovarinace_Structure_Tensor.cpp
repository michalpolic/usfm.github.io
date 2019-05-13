// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_InputCovarinace_Structure_Tensor.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/


#include "USfM/usfm_InputCovarinace_Structure_Tensor.hpp"

namespace usfm {

	void InputCovariance_StructureTensor::compute(Scene& scene, Statistic& statistic) {
		int num_obs = scene._projections.size();
		scene._in_cov = SM(2 * num_obs, 2 * num_obs);
		scene._in_precision_mat = SM(2 * num_obs, 2 * num_obs);

		// we assume the variance was loaded or computed before (i.e., Matlab must send values, COLMAP must read SIFT properties and work with images)
		// if the Point2D covariances are not loaded the function assume unity covariance matrix 

		// set Point2D covariances in the order of projections
		std::vector<Tri> cov_triplets;
		std::vector<Tri> precision_triplets;
		cov_triplets.reserve(2 * num_obs);
		precision_triplets.reserve(2 * num_obs);
		for (int i = 0; i < num_obs; ++i){
			Point2D *p2D = scene._projections[i]->_obs;
			const double *val = p2D->_xy_cov;
			cov_triplets.push_back(Tri(2*i + 0, 2*i + 0, val[0]));
			cov_triplets.push_back(Tri(2*i + 1, 2*i + 1, val[3]));
			cov_triplets.push_back(Tri(2*i + 1, 2*i + 0, val[1]));
			cov_triplets.push_back(Tri(2*i + 0, 2*i + 1, val[2]));

			const double d = val[0] * val[3] - val[1] * val[2];
			precision_triplets.push_back(Tri(2 * i + 0, 2 * i + 0, val[3] / d));
			precision_triplets.push_back(Tri(2 * i + 0, 2 * i + 1, -val[1] / d));
			precision_triplets.push_back(Tri(2 * i + 1, 2 * i + 0, -val[2] / d));
			precision_triplets.push_back(Tri(2 * i + 1, 2 * i + 1, val[0] / d));
		}
		scene._in_cov.setFromTriplets(cov_triplets.begin(), cov_triplets.end());
		scene._in_precision_mat.setFromTriplets(precision_triplets.begin(), precision_triplets.end());
	}

}