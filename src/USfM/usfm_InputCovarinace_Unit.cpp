// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_InputCovarinace_Unit.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/


#include "USfM/usfm_InputCovarinace_Unit.hpp"

namespace usfm {

	void InputCovariance_Unit::compute(Scene& scene, Statistic& statistic) {
		int num_obs = scene._projections.size();
		scene._in_cov = SM(2*num_obs, 2*num_obs);
		scene._in_precision_mat = SM(2 * num_obs, 2 * num_obs);

		// set identity matrix from triplets
		std::vector<Tri> triplets;
		triplets.reserve(2 * num_obs);
		for (int i = 0; i < 2 * num_obs; ++i)
			triplets.push_back(Tri(i, i, 1));
		scene._in_cov.setFromTriplets(triplets.begin(), triplets.end());
		scene._in_precision_mat.setFromTriplets(triplets.begin(), triplets.end());
	}

}