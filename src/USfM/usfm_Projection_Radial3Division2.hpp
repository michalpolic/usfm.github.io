// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Projection_Radial3Division2.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_PROJECTION_RADIAL3DIVISION2_H
#define USFM_PROJECTION_RADIAL3DIVISION2_H

#include "ceres/ceres.h"
#include "USfM/usfm_Projection_ACC.hpp"

namespace usfm {

	class ProjectionRadial3Division2 : public ProjectionACC {
	public:
		static const int N_CAM_PARAMS = 8;

		ProjectionRadial3Division2();
		ProjectionRadial3Division2(Camera* camera);
		ProjectionRadial3Division2(Image* camera);
		ProjectionRadial3Division2(Camera* camera, Image* image);
		ProjectionRadial3Division2(Camera* camera, Image* image, Point3D* X, Point2D* obs);

		// get number of parameters used by specified model
		int numCamParams();

		// init the pointers to the vector of parameters
		void initOffsets(Camera* cam);

		// reorder the parameters
		void reorderParams(Camera* cam);

		// Create: ceres::CostFunction for an observation
		ceres::CostFunction* getCostFunction();

		// compute the residual of related 3D point projection and measured observation
		void computeResidual(double* res);

		// The projection function (used by Ceres to compose the Jacobian)
		template <typename T>
		bool operator()(const T* const X, const T* const cam, const T* const img, T* residuals) const
		{
			// the internal/external parameters
			const T f = cam[0];								// focal length
			const T pp[2] = { cam[1], cam[2] };				// principal point
			const T k[5] = { cam[3], cam[4], cam[5], cam[6], cam[7] };					// radial distortion parameters
			const T aa[3] = { img[0] ,img[1] ,img[2] };		// angle axis
			const T C[3] = { img[3] ,img[4] ,img[5] };		// camera center

			// the computation of the residuals 
			const T pC[3] = { X[0] - C[0], X[1] - C[1], X[2] - C[2] };

			T p[3];
			AARotatePoint(aa, pC, p);
			const T x = p[0] / p[2];
			const T y = p[1] / p[2];

			const T r2 = x * x + y * y;
			const T r4 = r2 * r2;
			const T r6 = r4 * r2;
			const T distortion = (T(1.0) + k[0]*r2 + k[1]*r4 + k[2]*r6) / (T(1.0) + k[3]*r2 + k[4]*r4);
			const T predicted_x = f * x * distortion + pp[0];
			const T predicted_y = f * y * distortion + pp[1];

			residuals[0] = predicted_x - T(_obs->_xy[0]);
			residuals[1] = predicted_y - T(_obs->_xy[1]);
			return true;
		}
	};

}

#endif //USFM_PROJECTION_RADIAL3DIVISION2_H