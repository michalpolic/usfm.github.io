// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Projection.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_Projection.hpp"

namespace usfm {

	void Projection::exchangeParameters(double* arr, const int desired_offset,const int currnet_offset) {
		if ( desired_offset < 0 || currnet_offset < 0)
			throw std::runtime_error("At least one input offset/parameter is not initialized.");
		
		double tmp = arr[desired_offset];
		arr[desired_offset] = arr[currnet_offset];
		arr[currnet_offset] = tmp;
	}

	void Projection::exchangeParameters(Camera *cam, const int desired_offset, const ECameraParameter param) {
		if (!cam || desired_offset < 0 || param < 0)
			throw std::runtime_error("At least one input offset/parameter is not initialized.");

		// get offset of parameter to be moved
		int current_offset = cam->offset(param);

		// find if something points on desired offset
		ECameraParameter tmp_param;
		bool tmp_param_assigned = false;
		for (auto pair : cam->_offset_in_parameters) {
			if (pair.second == desired_offset) {
				tmp_param = pair.first;
				tmp_param_assigned = true;
				break;
			}
		}

		// change values 
		double tmp = cam->_parameters[desired_offset];
		cam->_parameters[desired_offset] = cam->_parameters[current_offset];
		cam->_parameters[current_offset] = tmp;

		// change the map of values 
		cam->_offset_in_parameters[param] = desired_offset;
		if (tmp_param_assigned)
			cam->_offset_in_parameters[tmp_param] = current_offset;
	}

	void Projection::exchangeParameters(Camera *cam, const std::vector<ECameraParameter> params) {
		for (int i = 0; i < params.size(); ++i)
			exchangeParameters(cam, i, params[i]);
	}

}