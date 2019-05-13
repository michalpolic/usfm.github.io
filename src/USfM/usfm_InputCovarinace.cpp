// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_InputCovarinace.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/


#include "USfM/usfm_InputCovarinace.hpp"

namespace usfm {

	void InputCovariance::reprojectionErrors(Scene& scene, std::vector<double> &reproj_errors) {
		int N = scene._projections.size();
		reproj_errors.resize(2 * N);
		for (int i = 0; i < N; ++i) 
			scene._projections[i]->computeResidual(reproj_errors.data() + 2 * i);
	}

} 