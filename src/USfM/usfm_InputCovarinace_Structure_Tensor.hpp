// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_InputCovarinace_Structure_Tensor.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_INPUT_COVARIANCE_STRUCTURE_TENSOR_H
#define USFM_INPUT_COVARIANCE_STRUCTURE_TENSOR_H

#include "USfM/usfm_InputCovarinace.hpp"

namespace usfm {

	class InputCovariance_StructureTensor : public InputCovariance {
	public:
		void compute(Scene& scene, Statistic& statistic);
	};
}

#endif