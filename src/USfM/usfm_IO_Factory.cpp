// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_IO_Factory.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_IO_Factory.hpp"

namespace usfm {

	inline std::string EInputFormat_toString(EInputFormat input_format) {
		switch (input_format) {
			case eCOLMAP: return "COLMAP";
			case eAliceVision: return "AliceVision";
		}
		throw std::runtime_error(std::string("EInputFormat_toString: unrecognized input format."));
	}

	inline EInputFormat EInputFormat_toEnum(const std::string& input_format){
		if (input_format == "COLMAP")
			return eCOLMAP;
		if (input_format == "AliceVision")
			return eAliceVision;
		throw std::runtime_error(std::string("EInputFormat_toEnum: unrecognized input format: ") + input_format);
	}

	std::shared_ptr<IO> IO_Factory::createIO(const std::string input_format) {
		EInputFormat e_input_format = EInputFormat_toEnum(input_format);
		switch (e_input_format) {
		case eCOLMAP:
			return std::make_shared<IO_Colmap>();
		case eMATLAB:
			#ifdef USE_MATLAB
				return std::make_shared<IO_Matlab>();
			#else
				throw std::runtime_error(std::string("Matlab is disabled. Recompile the project with: USfM_BUILD_MEX = ON"));
			#endif
		case eAliceVision:
			throw std::runtime_error(std::string("AliceVision input class is not implemented."));
		default:
			throw std::runtime_error(std::string("Unknown scene reader. Scene reader wasn't inicialized."));
		}
	}

}