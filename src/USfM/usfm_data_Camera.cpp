// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_data_Camera.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_data_Camera.hpp"
#include "USfM/usfm_Projection_Simple_Pinhole.hpp"
#include "USfM/usfm_Projection_Simple_Pinhole_RS.hpp"
#include "USfM/usfm_Projection_Pinhole.hpp"
#include "USfM/usfm_Projection_Radial1.hpp"
#include "USfM/usfm_Projection_Radial2.hpp"
#include "USfM/usfm_Projection_Radial3.hpp"
#include "USfM/usfm_Projection_Radial4.hpp"
#include "USfM/usfm_Projection_Simple_Radial_Fisheye.hpp"
#include "USfM/usfm_Projection_Simple_Radial_RS.hpp"
#include "USfM/usfm_Projection_Radial_Fisheye.hpp"
#include "USfM/usfm_Projection_Radial_RS.hpp"
#include "USfM/usfm_Projection_OpenCV.hpp"
#include "USfM/usfm_Projection_OpenCV_Fisheye.hpp"
#include "USfM/usfm_Projection_OpenCV_Full.hpp"
#include "USfM/usfm_Projection_FOV.hpp"
#include "USfM/usfm_Projection_Thin_Prism_Fisheye.hpp"
#include "USfM/usfm_Projection_Factory.hpp"

namespace usfm {

	// constructor
	Camera::Camera() {}
	Camera::Camera(const int cam_id, const ECameraModel camModel, std::vector<double> params) {
		_id = cam_id;
		_camModel = camModel;
		_parameters.resize(params.size());
		memcpy((void*)_parameters.data(), (void*)params.data(), params.size() * sizeof(double));
		initOffsets();
	}

	// init the pointers to the vector of parameters
	void Camera::initOffsets() {
		std::shared_ptr<Projection> proj = Projection_Factory::createProjection(this);
		proj->initOffsets(this);
		_num_params = proj->numCamParams();
	}

	// reorder the parameters
	void Camera::setModel(const ECameraModel camModel) {
		_camModel = camModel;
		std::shared_ptr<Projection> proj = Projection_Factory::createProjection(this);
		proj->reorderParams(this);
		_num_params = proj->numCamParams();
	}

	// get camera model
	ECameraModel Camera::getModel() const {
		return _camModel;
	}

	const int Camera::numParams() const {
		return _num_params;
	}

	// set the offsets
	void Camera::setOffset(const std::vector<ECameraParameter> params) {
		for (int i = 0; i < params.size(); ++i)
			_offset_in_parameters[params[i]] = i;
	}

	// provide the offset in "_parameters" to the variable "var" if exist
	int Camera::offset(const ECameraParameter var) const {
		if (_offset_in_parameters.find(var) == _offset_in_parameters.end())
			throw std::runtime_error(std::string("The parameter ") + std::to_string(var) + std::string(" is not initialized."));
		return _offset_in_parameters.at(var);
	}

	// provide the variable "var" if exist
	double Camera::value(const ECameraParameter var) const {
		return _parameters[offset(var)];
	}

	// set the variable "var" if exist
	void Camera::set(const ECameraParameter var, const double value) {
		_parameters[offset(var)] = value;
	}

	// print
	std::ostream& operator<< (std::ostream& out, const Camera& c) {
		out << "> Camera " << c._id << " [focal:" << c.value(e_f)
			<< ", width:" << c.value(e_img_width) << ", height:" << c.value(e_img_height) << "... ]\n";
		return out;
	};
}
