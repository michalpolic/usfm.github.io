// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_data_Scene_Options.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_ALGORITHM_OPTIONS_H
#define USFM_ALGORITHM_OPTIONS_H

#include <vector>

namespace usfm {

	struct Options {
	public:
		double _epsilon, _lambda = -1;
		int _numCams, _camParams, _numPoints, _numObs, _svdRemoveN = -1, _maxIterTE = -1;
		std::vector<int> _pts2fix;
		bool _debug = false;
		bool _computePtsCov = false;

		Options();
		Options(int numCams, int camParams, int numPoints, int numObs);
		Options(double eps_or_lamb, int numCams, int camParams, int numPoints, int numObs);
	};

}

#endif	// USFM_ALGORITHM_OPTIONS_H
