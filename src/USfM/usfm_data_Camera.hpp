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
		eSimplePinhole,
		ePinhole,
		eRadial1,
		eRadial2,
		eRadial3,
		eRadial4,
		eRadial1Fisheye,
		eRadial2Fisheye,
		eOpenCv,
		eOpenCvFisheye,
		eOpenCFull,
		eFOV,
		eThinPrismFisheye,
		eDivision1,
		eDivision2,
		eDivision3,
		eDivision4,
		eInverseDivision1,
		eInverseDivision2,
		eRadial1Division1,
		eRadial2Division2,
		eRadial3Division1,
		eRadial3Division2,
		eRadial3Division3
	};

	enum ECameraParameter {
		e_f,
		e_fx,
		e_fy,
		e_cx,
		e_cy,
		e_k1,
		e_k2,
		e_k3,
		e_k4,
		e_k5,
		e_k6,
		e_p1,
		e_p2,
		e_img_width,
		e_img_height
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

		// set offset 
		void setOffset(const std::vector<ECameraParameter> params);

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