// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_IO.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_IO_H
#define USFM_IO_H

#include <string>
#include <iostream>
#include <fstream>

#include "mex.h"
#include "USfM/usfm_data_Scene.hpp"
#include "USfM/usfm_Statistics.hpp"

namespace usfm {

	class IO {
	public:
		virtual void read(const std::string& input, Scene& scene) = 0;

		void writeCov2File(const std::string& filepath, Scene& scene, Statistic& statistic);
	};
}

#endif 