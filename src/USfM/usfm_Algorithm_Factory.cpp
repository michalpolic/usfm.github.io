// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Algorithm_Factory.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_Algorithm_Factory.hpp"

namespace usfm {

	inline std::string EAlgorithm_enumToString(EAlgorithm alg) {
		switch (alg) {
		case eSvdQrIteration: return "SVD_QR_ITERATION";
		case eSvdDeviceAndconquer: return "SVD_DEVIDE_AND_CONQUER";
		case eLhuilier: return "LHUILLIER";
		case eTaylorExpansion: return "TE-INVERSION";
		case eNullspaceBounding: return "NBUP";
		case eJacobianEstimator: return "JACOBIAN_ESTIMATOR";
		}
		throw std::runtime_error("Error: E_enumToString(EAlgorithm alg) - unrecognized algorithm!");
	}

	inline EAlgorithm EAlgorithm_stringToEnum(const std::string& algorithm) {
		if (algorithm == "SVD_QR_ITERATION")
			return eSvdQrIteration;
		if (algorithm == "SVD_DEVIDE_AND_CONQUER")
			return eSvdDeviceAndconquer;
		if (algorithm == "LHUILLIER")
			return eLhuilier;
		if (algorithm == "TE-INVERSION")
			return eTaylorExpansion;
		if (algorithm == "NBUP")
			return eNullspaceBounding;
		if (algorithm == "JACOBIAN_ESTIMATOR")
			return eJacobianEstimator;
		throw std::runtime_error(std::string("Unrecognized algorithm: ") + algorithm);
	}

	// Create the algorithm object
	std::shared_ptr<Algorithm> Algorithm_Factory::initAlgorithm(const EAlgorithm input_algorithm_enum) {
		if (input_algorithm_enum == eSvdQrIteration)
			throw std::runtime_error("Error: the algorithm SVD_QR is not implemented yet!");
			//return std::make_shared<Algorithm_SVD_QR>();

		if (input_algorithm_enum == eSvdDeviceAndconquer)
			throw std::runtime_error("Error: the algorithm SVD_DQ is not implemented yet!");
			//return std::make_shared<Algorithm_SVD_DQ>();

		if (input_algorithm_enum == eLhuilier)
			throw std::runtime_error("Error: the algorithm Lhuilier is not implemented yet!");
			//return std::make_shared<Algorithm_Lhuillier>();

		if (input_algorithm_enum == eTaylorExpansion)
			throw std::runtime_error("Error: the algorithm TaylorExpansion is not implemented yet!");
			//return std::make_shared<Algorithm_TE_Inversion>();

		if (input_algorithm_enum == eNullspaceBounding)
			return std::make_shared<Algorithm_NBUP>();

		if (input_algorithm_enum == eJacobianEstimator)
			return std::make_shared<Algorithm_JacobianEstimator>();

		throw std::runtime_error(std::string("Scene algorithm wasn't inicialized."));
	}

	// Create the algorithm object
	std::shared_ptr<Algorithm> Algorithm_Factory::initAlgorithm(const std::string input_algorithm){
		return initAlgorithm(EAlgorithm_stringToEnum(input_algorithm));
		
	};
}