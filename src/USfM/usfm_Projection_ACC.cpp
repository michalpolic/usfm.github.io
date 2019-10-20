// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Projection_ACC.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_Projection_ACC.hpp"

namespace usfm {

	// init pointers to images
	void ProjectionACC::initOffsets(Image* img) {
		img->setOffset(std::vector<IP>{IP(0, e_aa), IP(3, e_C), IP(6, e_q), IP(10, e_t), IP(13, e_R)});
	}

	// sort the parameters array
	void ProjectionACC::reorderParams(Image* img) {
		if (!img)
			throw std::runtime_error("The input image is not initialized.");
		img->reorderParams(eAAC);
	}

	// number of used image parameters for specified image model
	int ProjectionACC::numImgParams() {
		return ProjectionACC::N_IMG_PARAMS;
	}

}