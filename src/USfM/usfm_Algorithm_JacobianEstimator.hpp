// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Algorithm_JacobianEstimator.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_ALGORITHM_JACOBIANESTIMATOR_H
#define USFM_ALGORITHM_JACOBIANESTIMATOR_H

#include "USfM/usfm_Algorithm.hpp"

namespace usfm {

	class Algorithm_JacobianEstimator: public Algorithm {
	public:
		void compute(Scene& scene, Statistic& statistic);
		void optimizeRadialParams(Scene& scene, Statistic& statistic);
		void fixNonRadialParameters(Scene & scene, Statistic & statistic, ceres::Problem * problem);
		std::vector<int> getNotRadialParamsOffset(Camera & camera);
	};
}

#endif