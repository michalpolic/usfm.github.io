// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Projection_InverseDivision1.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_PROJECTION_INVERSEDIVISION1_H
#define USFM_PROJECTION_INVERSEDIVISION1_H

#include "ceres/ceres.h"
#include "USfM/usfm_Projection.hpp"

namespace usfm {

	class ProjectionInverseDivision1 : public Projection {
	public:
		static const int N_CAM_PARAMS = 4;
		static const int N_IMG_PARAMS = 6;

		ProjectionInverseDivision1();
		ProjectionInverseDivision1(Camera* camera);
		ProjectionInverseDivision1(Image* camera);
		ProjectionInverseDivision1(Camera* camera, Image* image);
		ProjectionInverseDivision1(Camera* camera, Image* image, Point3D* X, Point2D* obs);

		// get number of parameters used by specified model
		int numCamParams();
		int numImgParams();

		// init the pointers to the vector of parameters
		void initOffsets(Camera* cam);
		void initOffsets(Image* img);

		// reorder the parameters
		void reorderParams(Camera* cam);
		void reorderParams(Image* img);

		// Create: ceres::CostFunction for an observation
		ceres::CostFunction* getCostFunction();

		// compute the residual of related 3D point projection and measured observation
		void computeResidual(double* res);

		// The projection function (used by Ceres to compose the Jacobian)
		template <typename T>
		bool operator()(const T* const X, const T* const cam, const T* const img, T* residuals) const 
		{
			// the internal/external parameters
			T f = cam[0];								// focal length
			T pp[2] = { cam[1], cam[2] };				// principal point
			T k = cam[3];								// radial distortion parameters
			T aa[3] = { img[0] ,img[1] ,img[2] };		// angle axis
			T C[3] = { img[3] ,img[4] ,img[5] };		// camera center

			// the computation of the residuals 
			T p[3], pC[3];								// auxiliary arrays
			pC[0] = X[0] - C[0];
			pC[1] = X[1] - C[1];
			pC[2] = X[2] - C[2];

			AARotatePoint(aa, pC, p);
			T x = p[0] / p[2];
			T y = p[1] / p[2];

			T s2 = x*x + y*y;
			T s = sqrt(s2);
			T D = sqrt(T(1) - T(4) * k * s2);
			T ks2 = T(2) * k * s;
			T c_1 = (T(1) + D) / ks2 / s;
			T c_2 = (T(1) - D) / ks2 / s;
			
			T c = T(1);
			if ((c_1 - T(1))*(c_1 - T(1)) < (c_2 - T(1))*(c_2 - T(1))) {
				c = c_1;
			}else{
				c = c_2;
			}
			T predicted_x = f * c * x + pp[0];
			T predicted_y = f *  c * y + pp[1];

			residuals[0] = predicted_x - T(_obs->_xy[0]);
			residuals[1] = predicted_y - T(_obs->_xy[1]);
			return true;
		}


		/*s = sqrt(sum(un. ^ 2));
		D = sqrt(1 - 4 * cp(4)*s. ^ 2);
		ks2 = 2 * cp(4)*s;
		c_1 = (1 + D) . / ks2 . / s;
		c_2 = (1 - D) . / ks2 . / s;
		[~, sel] = min([abs(c_1 - 1); abs(c_2 - 1)]);
		c = c_1 .* (sel == 1) + c_2 .* (sel == 2);*/


	};





}

#endif //USFM_PROJECTION_INVERSEDIVISION1_H