// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_data_Camera.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_DATA_CAMERA_H
#define USFM_DATA_CAMERA_H

#include <vector>
#include <iostream>
#include <map>

namespace usfm {

	enum ECameraModel {
		eSimplePinhole = 0,
		ePinhole = 1,
		eRadial1 = 2,
		eRadial2 = 3,
		eRadial3 = 4,
		eRadial4 = 5,
		eRadial1Fisheye = 6,
		eRadial2Fisheye = 7,
		eOpenCv = 8,
		eOpenCvFisheye = 9,
		eOpenCFull = 10,
		eFOV = 11,
		eThinPrismFisheye = 12,
		eDivision1 = 13,
		eDivision2 = 14,
		eDivision3 = 15,
		eDivision4 = 16,
    eInverseDivision1 = 17,
    eInverseDivision2 = 18
	};

	enum ECameraParameter {
		e_f = 0,
		e_fx = 1,
		e_fy = 2,
		e_cx = 3,
		e_cy = 4,
		e_k1 = 5,
		e_k2 = 6,
		e_k3 = 7,
		e_k4 = 8,
		e_k5 = 9,
		e_k6 = 10,
		e_p1 = 11,
		e_p2 = 12,
		e_img_width = 13,
		e_img_height = 14
	};

	class Camera {
	public:
		int _id;
		std::vector<double> _parameters;		// vector of all parameters	
		std::map<ECameraParameter, int> _offset_in_parameters;	// map of variables to offset in parameters array
		int _num_params = 0;

		Camera();
		Camera(const int cam_id, const ECameraModel camModel, std::vector<double> params);

		void initOffsets();
		void setModel(const ECameraModel camModel);
		ECameraModel getModel() const;

		// get number of used parameters by the currently used model of camera
		const int numParams() const;

		// provide the offset in "_parameters" to the variable "var" if exist
		int offset(const ECameraParameter var) const;
		
		// provide the variable "var" if exist
		double value (const ECameraParameter var) const;

		// set the variable "var" if exist
		void set(const ECameraParameter var, const double value);

	private:
		ECameraModel _camModel;			// camera model defines the order of parameters 
	};

	std::ostream& operator<< (std::ostream& out, const Camera& s);
}

#endif