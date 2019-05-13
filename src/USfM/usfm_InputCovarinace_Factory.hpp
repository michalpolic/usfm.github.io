// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_InputCovarinace_Factory.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_INPUT_COVARIANCE_FACTORY_H
#define USFM_INPUT_COVARIANCE_FACTORY_H

#include "USfM/usfm_InputCovarinace.hpp"
#include "USfM/usfm_InputCovarinace_Unit.hpp"
#include "USfM/usfm_InputCovarinace_Variance_Factor.hpp"
#include "USfM/usfm_InputCovarinace_Squared_Residuals.hpp"
#include "USfM/usfm_InputCovarinace_Structure_Tensor.hpp"

namespace usfm {
	
	class InputCovariance_Factory {
	public:
		static std::shared_ptr<InputCovariance> initInputCovariance(const EPoint2DCovariance covariance);
	};
}

#endif