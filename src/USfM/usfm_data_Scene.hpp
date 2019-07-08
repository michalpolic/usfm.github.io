// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_data_Scene.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_DATA_SCENE_H
#define USFM_DATA_SCENE_H

#include <map>
#include <ostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "USfM/usfm_data_Camera.hpp"
#include "USfM/usfm_data_Image.hpp"
#include "USfM/usfm_data_Point3D.hpp"
#include "USfM/usfm_Algorithm_Options.hpp"
#include "USfM/usfm_Projection.hpp"

typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SM;
typedef Eigen::SparseMatrix<double, Eigen::RowMajor> SM_row;
typedef Eigen::MatrixXd DM;
typedef Eigen::Triplet<double> Tri;


namespace usfm {

	enum EAlgorithm {
		eSvdQrIteration = 0,
		eSvdDeviceAndconquer = 1,
		eTaylorExpansion = 2,
		eNullspaceBounding = 3,
		eLhuilier = 4,
		eJacobianEstimator = 5
	};

	struct NumSceneParams {
		int _numCamPar = 0;
		int _numImgPar = 0;
		int _numPtsPar = 0;
	};

	enum EPoint2DCovariance {
		eUnit = 0,
		eVarianceFactor = 1,
		eSquaredResiduals = 2,
		eStructureTensor = 3
	};

	EPoint2DCovariance str_to_EPoint2DCovariance(const std::string type);

	// settings of the algoritmes
	struct AlgSettings {
		EAlgorithm _alg = eNullspaceBounding;
		EPoint2DCovariance _e_in_cov = eVarianceFactor;
		bool _run_opt = true;			// run the CERES optimization
		bool _run_opt_radial = false;   // run the CERES optimization on radial coeff. only
		bool _robust_lost = false;
    bool _return_nullspace = false;
	};


	class Scene {
	public:
		Scene();
		AlgSettings _settings;

		// input parameters
		std::map<int, Camera> _cameras;
		std::map<int, Image> _images;
		std::map<int, Point3D> _points3D;
		SM _in_cov;
		SM _in_precision_mat;
		
		// projection functions
		std::vector<std::shared_ptr<Projection>> _projections;

		// internal parameters
		SM _Sa;			// applied scale of the _jacobian from right
		SM_row _jacobian;
		DM _H;			// the nullspace of jacobian (used in NBUP alg.)
		Options _options = Options();

		// output parameters 
		DM _iZ;			// covariance matrix of camera parameters
		std::vector<Eigen::Matrix3f> _cov_pts;

		// the numbers of parameters
		const NumSceneParams getNumSceneParams() const;
		int numParams();

		void setInputCovarianceEstimator(const std::string type);
	};

	std::ostream& operator<< (std::ostream& out, const Scene& s);
}

#endif