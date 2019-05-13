// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_InputCovarinace.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_INPUT_COVARIANCE_H
#define USFM_INPUT_COVARIANCE_H

#include "USfM/usfm_data_Scene.hpp"
#include "USfM/usfm_Statistics.hpp"

namespace usfm {
	
	class InputCovariance {
	public:
		virtual void compute(Scene &scene, Statistic &statistic) = 0;

	protected:
		void reprojectionErrors(Scene& scene, std::vector<double> &reproj_errors);
	};
}

#endif