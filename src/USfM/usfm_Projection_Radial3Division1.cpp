// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Projection_Radial3Division1.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_Projection_Radial3Division1.hpp"

namespace usfm {

	ProjectionRadial3Division1::ProjectionRadial3Division1() {}
	ProjectionRadial3Division1::ProjectionRadial3Division1(Image* image) {
		_image = image;
	};
	ProjectionRadial3Division1::ProjectionRadial3Division1(Camera* camera) {
		_camera = camera;
	};
	ProjectionRadial3Division1::ProjectionRadial3Division1(Camera* camera, Image* image) {
		_camera = camera;
		_image = image;
	}
	ProjectionRadial3Division1::ProjectionRadial3Division1(Camera* camera, Image* image, Point3D* X, Point2D* obs) {
		_camera = camera;
		_image = image;
		_X = X;
		_obs = obs;
	}

	// assume correct order of parameters array, assign the pointers
	void ProjectionRadial3Division1::initOffsets(Camera* cam) {
		cam->setOffset(std::vector<ECameraParameter>{e_img_width, e_img_height, e_f, e_cx, e_cy, e_k1, e_k2, e_k3, e_k4});
	}

	// sort the parameters array (allows the change of projection function)
	void ProjectionRadial3Division1::reorderParams(Camera* cam) {
		if (!cam)
			throw std::runtime_error("The input camera is not initialized.");
		exchangeParameters(cam, std::vector<ECameraParameter>{e_f, e_cx, e_cy, e_k1, e_k2, e_k3, e_k4});
	}

	// ceres function for deriving the derivation
	ceres::CostFunction* ProjectionRadial3Division1::getCostFunction() {
		return (new ceres::AutoDiffCostFunction<ProjectionRadial3Division1,
			2,												// output residuals
			3,												// input 3D point	
			ProjectionRadial3Division1::N_CAM_PARAMS,		// camera parameters
			ProjectionRadial3Division1::N_IMG_PARAMS>(new ProjectionRadial3Division1(_camera, _image, _X, _obs)));	// image parameters
	}

	// compute the residual of related 3D point projection and measured observation
	void ProjectionRadial3Division1::computeResidual(double * res) {
		if (_camera == NULL || _image == NULL || _obs == NULL || _X == NULL)
			throw std::runtime_error("To compute residual function, the pointers _camera, _image, _obs, _X must be known.");
		if (!(*this)(_X->_X, _camera->_parameters.data(), _image->_parameters.data(), res))
			throw std::runtime_error("The computation of the residual error failed.");
	}

	// number of used camera parameters for specified image model
	int ProjectionRadial3Division1::numCamParams() {
		return ProjectionRadial3Division1::N_CAM_PARAMS;
	}


}