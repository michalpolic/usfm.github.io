// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Projection_InverseDivision1.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_Projection_InverseDivision1.hpp"

namespace usfm {

	ProjectionInverseDivision1::ProjectionInverseDivision1() {}
	ProjectionInverseDivision1::ProjectionInverseDivision1(Image* image) {
		_image = image;
	};
	ProjectionInverseDivision1::ProjectionInverseDivision1(Camera* camera) {
		_camera = camera;
	};
	ProjectionInverseDivision1::ProjectionInverseDivision1(Camera* camera, Image* image) {
		_camera = camera;
		_image = image;
	}
	ProjectionInverseDivision1::ProjectionInverseDivision1(Camera* camera, Image* image, Point3D* X, Point2D* obs) {
		_camera = camera;
		_image = image;
		_X = X;
		_obs = obs;
	}


	// assume the correct order of parameters array 
	// assign the pointers (allows the change of projection function)
	void ProjectionInverseDivision1::initOffsets(Camera* cam) {
		cam->_offset_in_parameters[e_img_width] = 0;
		cam->_offset_in_parameters[e_img_height] = 1;
		cam->_offset_in_parameters[e_f] = 2;
		cam->_offset_in_parameters[e_cx] = 3;
		cam->_offset_in_parameters[e_cy] = 4;
		cam->_offset_in_parameters[e_k1] = 5;
	}

	// sort the parameters array (allows the change of projection function)
	void ProjectionInverseDivision1::reorderParams(Camera* cam) {
		if (!cam)
			throw std::runtime_error("The input camera is not initialized.");

		exchangeParameters(cam, 0, e_f);
		exchangeParameters(cam, 1, e_cx);
		exchangeParameters(cam, 2, e_cy);
		exchangeParameters(cam, 3, e_k1);
	}

	// init pointers to images
	void ProjectionInverseDivision1::initOffsets(Image* img) {
		img->_offset_in_parameters[e_aa] = 0;
		img->_offset_in_parameters[e_C] = 3;
		img->_offset_in_parameters[e_q] = 6;
		img->_offset_in_parameters[e_t] = 10;
		img->_offset_in_parameters[e_R] = 13;
	}

	// sort the parameters array
	void ProjectionInverseDivision1::reorderParams(Image* img) {
		std::vector<double> sorted_params;
		sorted_params.resize(22);
		for (int i = 0; i < 3; ++i)
			sorted_params[i + 0] = img->value(e_aa)[i];
		for (int i = 0; i < 3; ++i)
			sorted_params[i + 3] = img->value(e_C)[i];
		for (int i = 0; i < 4; ++i)
			sorted_params[i + 6] = img->value(e_q)[i];
		for (int i = 0; i < 3; ++i)
			sorted_params[i + 10] = img->value(e_t)[i];
		for (int i = 0; i < 9; ++i)
			sorted_params[i + 13] = img->value(e_R)[i];
		memcpy((void*)img->_parameters.data(), (void*)sorted_params.data(), 22 * sizeof(double));
		img->_offset_in_parameters[e_aa] = 0;
		img->_offset_in_parameters[e_C] = 3;
		img->_offset_in_parameters[e_q] = 6;
		img->_offset_in_parameters[e_t] = 10;
		img->_offset_in_parameters[e_R] = 13;
	}

	// ceres function for deriving the derivation
	ceres::CostFunction* ProjectionInverseDivision1::getCostFunction() {
		return (new ceres::AutoDiffCostFunction<ProjectionInverseDivision1,
			2,												// output residuals
			3,												// input 3D point	
			ProjectionInverseDivision1::N_CAM_PARAMS,			// camera parameters
			ProjectionInverseDivision1::N_IMG_PARAMS>(new ProjectionInverseDivision1(_camera, _image, _X, _obs)));	// image parameters
	}

	// compute the residual of related 3D point projection and measured observation
	void ProjectionInverseDivision1::computeResidual(double * res) {
		if (_camera == NULL || _image == NULL || _obs == NULL || _X == NULL)
			throw std::runtime_error("To compute residual function, the pointers _camera, _image, _obs, _X must be known.");
		if (!(*this)(_X->_X, _camera->_parameters.data(), _image->_parameters.data(), res))
			throw std::runtime_error("The computation of the residual error failed.");
	}

	// number of used camera parameters for specified image model
	int ProjectionInverseDivision1::numCamParams() {
		return ProjectionInverseDivision1::N_CAM_PARAMS;
	}

	// number of used image parameters for specified image model
	int ProjectionInverseDivision1::numImgParams() {
		return ProjectionInverseDivision1::N_IMG_PARAMS;
	}
}