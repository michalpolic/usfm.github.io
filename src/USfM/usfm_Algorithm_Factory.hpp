// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Algorithm_Factory.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_ALGORITHM_FACTORY_H
#define USFM_ALGORITHM_FACTORY_H

#include <string>
#include "USfM/usfm_Algorithm.hpp"
#include "USfM/usfm_Algorithm_SVD_QR.hpp"
#include "USfM/usfm_Algorithm_SVD_DQ.hpp"
#include "USfM/usfm_Algorithm_Lhuillier.hpp"
#include "USfM/usfm_Algorithm_TE_Inversion.hpp"
#include "USfM/usfm_Algorithm_NBUP.hpp"
#include "USfM/usfm_Algorithm_JacobianEstimator.hpp"

namespace usfm {

	
	inline std::string EAlgorithm_enumToString(EAlgorithm alg);
	inline EAlgorithm EAlgorithm_stringToEnum(const std::string& algorithm);


	// Generate the io object for selected input file format
	class Algorithm_Factory {
	public:
		static std::shared_ptr<Algorithm> initAlgorithm(const std::string type);
		static std::shared_ptr<Algorithm> initAlgorithm(const EAlgorithm input_algorithm_enum);
	};

}

#endif