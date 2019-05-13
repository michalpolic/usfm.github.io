// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Scene.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_IO_FACTORY_H
#define USFM_IO_FACTORY_H

#include <string.h>
#include <memory>
#include "USfM/usfm_IO.hpp"
#include "USfM/usfm_IO_Colmap.hpp"
#include "USfM/usfm_IO_Matlab.hpp"

namespace usfm {

	enum EInputFormat {
		eCOLMAP = 0,
		eMATLAB = 1,
		eAliceVision = 2
	};

	inline std::string EInputFormat_toString(EInputFormat input_format);
	inline EInputFormat EInputFormat_toEnum(const std::string& input_format);

	// Generate the io object for selected input file format
	class IO_Factory {
	public:
		static std::shared_ptr<IO> createIO(const std::string type);
	};

}

#endif