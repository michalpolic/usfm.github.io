// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_InputCovarinace_Factory.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_InputCovarinace_Factory.hpp"

namespace usfm {


	// retun the instance of input covariance estimator
	std::shared_ptr<InputCovariance> InputCovariance_Factory::initInputCovariance(const EPoint2DCovariance covariance) {
		if (covariance == eUnit)
			return std::make_shared<InputCovariance_Unit>();

		if (covariance == eVarianceFactor)
			return std::make_shared<InputCovariance_VarianceFactor>();

		if (covariance == eSquaredResiduals)
			return std::make_shared<InputCovariance_SquaredResiduals>();

		if (covariance == eStructureTensor)
			return std::make_shared<InputCovariance_StructureTensor>();
		 
		throw std::runtime_error(std::string("Observations covariance estimator wasn't inicialized."));
	}




}