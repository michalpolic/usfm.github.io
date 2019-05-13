// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_data_Image.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_data_Image.hpp"
#include "USfM/usfm_Projection.hpp"
#include "USfM/usfm_Projection_Factory.hpp"

namespace usfm {

	Image::Image() {}

	Image::Image(const int id, const int cam_id, const double q[4], const double t[3]) {
		_id = id;
		_cam_id = cam_id;

		// init parameters vector
		_parameters.resize(22);
		_offset_in_parameters[e_q] = 0;
		_offset_in_parameters[e_t] = 4;
		_offset_in_parameters[e_aa] = 7;
		_offset_in_parameters[e_C] = 10;
		_offset_in_parameters[e_R] = 13;
		
		// copy q,t into the parameters and compute aa,C
		set(e_q, q, 4);
		set(e_t, t, 3);
		qt2aaRC();

		// assume the angle-axis, camera center model
		setModel(eAAC);
	}

	// reorder the parameters
	void Image::setModel(const EImgModel imgModel) {
		_imgModel = imgModel;
		std::shared_ptr<Projection> proj = Projection_Factory::createProjection(this);
		proj->reorderParams(this);
		_num_params = proj->numImgParams();
	}

	// get image model
	EImgModel Image::getModel() const {
		return _imgModel;
	}

	const int Image::numParams() const {
		return _num_params;
	}

	// provide the offset in "_parameters" to the variable "var" if exist
	int Image::offset(const EImgParameter var) const {
		if (_offset_in_parameters.find(var) == _offset_in_parameters.end())
			throw std::runtime_error(std::string("The parameter ") + std::to_string(var) + std::string(" is not initialized."));
		return _offset_in_parameters.at(var);
	}

	// provide the variable "var" if exist
	double* Image::value(const EImgParameter var) const {
		return (double*) &(_parameters[offset(var)]);
	}

	// set the variable "var" if exist
	void Image::set(const EImgParameter var, const double* values, const int length) {
		for (int i = 0; i < length; ++i)
			_parameters[offset(var) + i] = values[i];
	}


	// compute the rotation, angle-axis and camera center
	// external parameters from quaternion and translation
	void Image::qt2aaRC() {
		double *R = value(e_R);
		double *q = value(e_q);
		double *t = value(e_t);
		double *aa = value(e_aa);
		double *C = value(e_C);

		// convert quatrnion to rotation matrix 
		q2R(q, R);

		// convert quatrnion to angle axis
		double sinth = sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		double angle = 2 * atan2(sinth, q[0]);
		if (sinth < std::numeric_limits<double>::epsilon() * 1e5) {
			aa[0] = 0;
			aa[1] = 0;
			aa[2] = 0;
		}
		else {
			aa[0] = angle * q[1] / sinth;
			aa[1] = angle * q[2] / sinth;
			aa[2] = angle * q[3] / sinth;
		}

		

		// convert translation to camera center
		C[0] = -(R[0] * t[0] + R[3] * t[1] + R[6] * t[2]);
		C[1] = -(R[1] * t[0] + R[4] * t[1] + R[7] * t[2]);
		C[2] = -(R[2] * t[0] + R[5] * t[1] + R[8] * t[2]);
	} 

	// compute R from quaternion
	void Image::q2R(double q[4], double* R) {
		R[0] = 1 - 2 * q[2] * q[2] - 2 * q[3] * q[3];
		R[1] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
		R[2] = 2 * q[3] * q[1] + 2 * q[0] * q[2];
		R[3] = 2 * q[1] * q[2] + 2 * q[0] * q[3];
		R[4] = 1 - 2 * q[1] * q[1] - 2 * q[3] * q[3];
		R[5] = 2 * q[2] * q[3] - 2 * q[0] * q[1];
		R[6] = 2 * q[3] * q[1] - 2 * q[0] * q[2];
		R[7] = 2 * q[2] * q[3] + 2 * q[0] * q[1];
		R[8] = 1 - 2 * q[1] * q[1] - 2 * q[2] * q[2];
	}

	// compute quaternion from R
	void Image::R2q(double R[9], double* q) {
		double c = R[0] + R[4] + R[8] + 1;
		if (std::abs(c) < 10000 * 1e-16) {	// 180 degrees
			Eigen::Matrix3d S;
			S << R[0] + 1, R[1], R[2],	R[3], R[4] + 1, R[5],	R[6], R[7], R[8] + 1;
			double a = S.block(0, 0, 3, 1).norm();
			double b = S.block(0, 1, 3, 1).norm();
			double c = S.block(0, 2, 3, 1).norm();
			Eigen::Vector3d s;
			if (a > b && a > c)
				s << S(0, 0)/a, S(1, 0)/a, S(2, 0)/a;
			if (b > a && b > c)
				s << S(0, 1)/b, S(1, 1)/b, S(2, 1)/b;
			if (c > a && c > b)
				s << S(0, 2)/c, S(1, 2)/c, S(2, 2)/c;
			q[0] = 0;
			q[1] = s(0);
			q[2] = s(1);
			q[3] = s(2);
		}
		else{
			q[0] = sqrt(c) / 2;
			q[1] = (R[5] - R[7]) / sqrt(c) / 2;
			q[2] = (R[6] - R[2]) / sqrt(c) / 2;
			q[3] = (R[1] - R[3]) / sqrt(c) / 2;
			double nq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
			q[0] = q[0] / nq;
			q[1] = q[1] / nq;
			q[2] = q[2] / nq;
			q[3] = q[3] / nq;
		}
	}

	// compute q from angle axis (Euler vector)
	void Image::aa2q(double aa[3], double *q) {
		const double theta2 = aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2];
		if (theta2 > std::numeric_limits<double>::epsilon()) {
			const double theta = sqrt(theta2);
			const double th = sin(0.5 * theta) * (1.0 / theta);
			q[0] = cos(0.5 *  theta);
			q[1] = th * aa[0];
			q[2] = th * aa[1];
			q[3] = th * aa[2];

		}else {
			q[0] = 0;
			q[1] = aa[0];
			q[2] = aa[1];
			q[3] = aa[2];
		}
	}

	// print
	std::ostream& operator<< (std::ostream& out, const Image& i) {
		out << "> Image [id:" << i._id << ", nobs:" << i._point2D.size()
			<< ", q:" << i.value(e_q)[0] << "," << i.value(e_q)[1] << "," << i.value(e_q)[2] << "," << i.value(e_q)[3]
			<< ", t:" << i.value(e_t)[0] << "," << i.value(e_t)[1] << "," << i.value(e_t)[2] << "]\n";

		out << ">> Observations: \n";
		int k = 0;
		for (Point2D const& p2D : i._point2D) {
			if (k < 10) {
				out << p2D;
				k++;
			}
			else break;
		}
		return out;
	};
}
