// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   main_uncertainty.cpp
* Author: Michal Polic
*/

#include <string>

#include "USfM/usfm_Statistics.hpp"
#include "USfM/usfm_IO.hpp"
#include "USfM/usfm_IO_Factory.hpp"
#include "USfM/usfm_Algorithm_Factory.hpp"
#include "USfM/usfm_data_Scene.hpp"

#include "mex.h"

using namespace usfm;

/*
Matlab interface
covariances = usfm_mex( scene )
*/
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	if (nrhs != 4)		// input is: algorithm, obs_covariance, cameras, images, points3D
		exit(1);
	Statistic statistic = Statistic();
	Scene scene = Scene();
	
	// read input data
	std::shared_ptr<IO_Matlab> io;
	io->read(prhs, scene);

	// init algorithm
	std::shared_ptr<Algorithm> alg = Algorithm_Factory::initAlgorithm(scene._settings._alg);

	// run algorithm
	alg->compute(scene, statistic);

	// save the results
	io->writeToMatlab(nlhs, plhs, scene, statistic);
}
