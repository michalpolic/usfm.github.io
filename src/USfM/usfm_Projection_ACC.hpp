// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Projection_ACC.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_PROJECTION_ACC_H 
#define USFM_PROJECTION_ACC_H

#include "USfM/usfm_Projection.hpp"


namespace usfm {

	class ProjectionACC : public Projection {
	public:
		static const int N_IMG_PARAMS = 6;

		// number of images parameters
		int numImgParams();

		// init the offsets of variables in the vector _parameters
		void initOffsets(Image* img);

		// reorder the parameters
		void reorderParams(Image* img);

	};

}

#endif	// USFM_PROJECTION_ACC_H