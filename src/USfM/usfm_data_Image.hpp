// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_data_Image.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_DATA_IMAGE_H
#define USFM_DATA_IMAGE_H

#include <vector>
#include <ostream>
#include <map>

#include "USfM/usfm_data_Point2D.hpp"

namespace usfm {

	enum EImgModel {
		eAAC = 0,				// AA - angle axis, C - camera center
		eAACRS = 1				// AA - angle axis, C - camera center, RS - rolling shutter parameters
	};

	enum EImgParameter {
		e_aa = 0,	// double aa[3] - angle axis (Euler vector)
		e_C = 1,	// double C[3] - camera center
		e_q = 2,	// double q[4] - quaternion (Eigen format)
		e_t = 3,	// oduble t[3] - translation 
		e_R = 4		// double R[9] - rotation matrix (Eigen format)
	};

	// class for saving pair: offset - EImgParameter
	class IP {
	public:
		int _offset;
		EImgParameter _parameter;
		IP(int offset, EImgParameter parameter);
	};


	class Image {
	public:
		int _id;
		int _cam_id;
		std::vector<double> _parameters;	// standard format is: aa, C
		std::map<EImgParameter, int> _offset_in_parameters;	// map of variables to offset in parameters array
		std::vector<Point2D> _point2D;
		int _num_params = 0;

		// construct image from q,t
		Image();
		Image(const int id, const int cam_id, const double q[4], const double t[3]);

		// reorder the parameters vector according the model
		void setModel(const EImgModel imgModel);

		// get the current model
		EImgModel getModel() const;

		// get number of used parameters by the currently used model of camera
		const int numParams() const;

		// set offset 
		void setOffset(const std::vector<IP> params);

		// provide the offset in "_parameters" to the variable "var" if exist
		int offset(const EImgParameter var) const;

		// order the parameters according selected image model
		void reorderParams(EImgModel model);

		// provide the variable "var" if exist
		double* value(const EImgParameter var) const;

		// set the variable "var" if exist
		void set(const EImgParameter var, const double* values, const int length);

		// compute quaternion from R
		static void q2R(double q[4], double* R);

		// compute R from quaternion
		static void R2q(double R[9], double* q);

		// compute q from angle axis (Euler vector)
		static void aa2q(double aa[3], double *q);

	private:
		EImgModel _imgModel;
		
		// compute camera center and angle axis from quaternion and translation 
		void qt2aaRC();		
	};

	std::ostream& operator<< (std::ostream& out, const Image& i);
}

#endif