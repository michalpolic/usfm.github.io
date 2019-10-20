// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Projection.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_PROJECTION_H 
#define USFM_PROJECTION_H

#include <vector>

#include "USfM/usfm_data_Camera.hpp"
#include "USfM/usfm_data_Image.hpp"
#include "USfM/usfm_data_Point3D.hpp"

#include "ceres/ceres.h"



namespace usfm {

	class Projection {
	public:
		Camera* _camera = NULL;
		Image* _image = NULL;
		Point2D* _obs = NULL;
		Point3D* _X = NULL;

		virtual int numCamParams() = 0;
		virtual int numImgParams() = 0;
		virtual ceres::CostFunction* getCostFunction() = 0;

		// init the offsets of variables in the vector _parameters
		virtual void initOffsets(Camera* cam) = 0;
		//virtual void initOffsets(Image* img) = 0;

		// reorder the parameters
		virtual void reorderParams(Camera* cam) = 0;
		virtual void reorderParams(Image* img) = 0;

		// compute residual
		virtual void computeResidual(double* res) = 0;

	protected:
		// Exchange two doubles and return the pointer to the new position of v2
		void exchangeParameters(double* arr, const int desired_offset, const int currnet_offset);
		void exchangeParameters(Camera *cam, const int desired_offset, const ECameraParameter param);
		void exchangeParameters(Camera *cam, const std::vector<ECameraParameter> params);
	};


	// Rotation of point using angle-axis
	template<typename T> inline
		void AARotatePoint(const T aa[3], const T pt[3], T result[3]) {
		const T theta2 = aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2];
		if (theta2 > T(std::numeric_limits<double>::epsilon())) {
			const T theta = sqrt(theta2);
			const T costheta = cos(theta);
			const T sintheta = sin(theta);
			const T theta_inverse = 1.0 / theta;

			const T w[3] = { aa[0] * theta_inverse,
				aa[1] * theta_inverse,
				aa[2] * theta_inverse };

			// Cross product
			const T w_cross_pt[3] = { w[1] * pt[2] - w[2] * pt[1],
				w[2] * pt[0] - w[0] * pt[2],
				w[0] * pt[1] - w[1] * pt[0] };
			const T tmp = (w[0] * pt[0] + w[1] * pt[1] + w[2] * pt[2]) * (T(1.0) - costheta);

			result[0] = pt[0] * costheta + w_cross_pt[0] * sintheta + w[0] * tmp;
			result[1] = pt[1] * costheta + w_cross_pt[1] * sintheta + w[1] * tmp;
			result[2] = pt[2] * costheta + w_cross_pt[2] * sintheta + w[2] * tmp;
		}
		else {
			const T w_cross_pt[3] = { aa[1] * pt[2] - aa[2] * pt[1],
				aa[2] * pt[0] - aa[0] * pt[2],
				aa[0] * pt[1] - aa[1] * pt[0] };
			result[0] = pt[0] + w_cross_pt[0];
			result[1] = pt[1] + w_cross_pt[1];
			result[2] = pt[2] + w_cross_pt[2];
		}
	}
}

#endif	// USFM_PROJECTION_H