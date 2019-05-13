// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_main.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_Statistics.hpp"
#include "USfM/usfm_IO.hpp"
#include "USfM/usfm_IO_Factory.hpp"
#include "USfM/usfm_Algorithm_Factory.hpp"
#include "USfM/usfm_data_Scene.hpp"

#include <gflags/gflags.h>

#ifdef _WIN32
	#define EXECUTABLE_FILE "uncertainty.exe"
#elif __linux__ 
	#define EXECUTABLE_FILE "uncertainty"
#endif

#ifndef LOAD_CMD_IO_FLAGS
    #define LOAD_CMD_IO_FLAGS
    DEFINE_string(alg, "NBUP",
            "algorithm for inversion of Schur complement matrix [SVD_QR_ITERATION, SVD_DEVIDE_AND_CONQUER, LHUILLIER, TE-INVERSION, NBUP]");
	
	DEFINE_string(in_cov, "UNIT",
		"the covariance matrix of observations [UNIT, VARIANCE_FACTOR, SQUARED_RESIDUALS, STRUCTURE_TENSOR]");

    DEFINE_string(in, ".",
            "path to input scene files (e.g. directory which contains cameras.txt, images.txt, points3D.txt for COLMAP)");

    DEFINE_string(in_form, "COLMAP",
			"the format of input data [COLMAP]");	// TODO: AliceVision input

    DEFINE_string(out, ".",
            "path to output covariance file");

    DEFINE_bool(debug, false,
            "print all the used matrices to txt files.\n");


#endif

using namespace usfm;

// Main function called from command line
int main(int argc, char* argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    Statistic statistic = Statistic();
    Scene scene = Scene();

	// read input data
	std::shared_ptr<IO> io = IO_Factory::createIO(FLAGS_in_form);
	io->read(FLAGS_in, scene);
    scene.setInputCovarianceEstimator(FLAGS_in_cov); 

	// init algorithm
  	std::shared_ptr<Algorithm> alg = Algorithm_Factory::initAlgorithm(FLAGS_alg);
	
	// run algorithm
 	alg->compute(scene, statistic);
        
	// save the results
	io->writeCov2File(FLAGS_out, scene, statistic);
}