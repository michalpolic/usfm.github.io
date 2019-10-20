// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Projection_Radial2Division2.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_Projection_Radial2Division2.hpp"

namespace usfm {

	ProjectionRadial2Division2::ProjectionRadial2Division2() {}
	ProjectionRadial2Division2::ProjectionRadial2Division2(Image* image) {
		_image = image;
	};
	ProjectionRadial2Division2::ProjectionRadial2Division2(Camera* camera) {
		_camera = camera;
	};
	ProjectionRadial2Division2::ProjectionRadial2Division2(Camera* camera, Image* image) {
		_camera = camera;
		_image = image;
	}
	ProjectionRadial2Division2::ProjectionRadial2Division2(Camera* camera, Image* image, Point3D* X, Point2D* obs) {
		_camera = camera;
		_image = image;
		_X = X;
		_obs = obs;
	}

	// assume correct order of parameters array, assign the pointers
	void ProjectionRadial2Division2::initOffsets(Camera* cam) {
		cam->setOffset(std::vector<ECameraParameter>{e_img_width, e_img_height, e_f, e_cx, e_cy, e_k1, e_k2, e_k3, e_k4});
	}

	// sort the parameters array (allows the change of projection function)
	void ProjectionRadial2Division2::reorderParams(Camera* cam) {
		if (!cam)
			throw std::runtime_error("The input camera is not initialized.");
		exchangeParameters(cam, std::vector<ECameraParameter>{e_f, e_cx, e_cy, e_k1, e_k2, e_k3, e_k4});
	}

	// ceres function for deriving the derivation
	ceres::CostFunction* ProjectionRadial2Division2::getCostFunction() {
		return (new ceres::AutoDiffCostFunction<ProjectionRadial2Division2,
			2,												// output residuals
			3,												// input 3D point	
			ProjectionRadial2Division2::N_CAM_PARAMS,		// camera parameters
			ProjectionRadial2Division2::N_IMG_PARAMS>(new ProjectionRadial2Division2(_camera, _image, _X, _obs)));	// image parameters
	}

	// compute the residual of related 3D point projection and measured observation
	void ProjectionRadial2Division2::computeResidual(double * res) {
		if (_camera == NULL || _image == NULL || _obs == NULL || _X == NULL)
			throw std::runtime_error("To compute residual function, the pointers _camera, _image, _obs, _X must be known.");
		if (!(*this)(_X->_X, _camera->_parameters.data(), _image->_parameters.data(), res))
			throw std::runtime_error("The computation of the residual error failed.");
	}

	// number of used camera parameters for specified image model
	int ProjectionRadial2Division2::numCamParams() {
		return ProjectionRadial2Division2::N_CAM_PARAMS;
	}


}