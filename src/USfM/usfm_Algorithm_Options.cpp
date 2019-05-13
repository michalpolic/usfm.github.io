// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_data_Scene_Options.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_Algorithm_Options.hpp"

namespace usfm {

	Options::Options() : _lambda(-1), _svdRemoveN(-1), _maxIterTE(-1) {}

	Options::Options(int numCams, int camParams, int numPoints, int numObs) :
		_epsilon(1e-10), _lambda(-1), _numCams(numCams), _camParams(camParams),
		_numPoints(numPoints), _numObs(numObs), _svdRemoveN(-1), _maxIterTE(-1) {}

	Options::Options(double eps_or_lamb, int numCams, int camParams, int numPoints, int numObs) :
		_epsilon(eps_or_lamb), _lambda(eps_or_lamb), _numCams(numCams), _camParams(camParams),
		_numPoints(numPoints), _numObs(numObs), _svdRemoveN(-1), _maxIterTE(-1) {}

}