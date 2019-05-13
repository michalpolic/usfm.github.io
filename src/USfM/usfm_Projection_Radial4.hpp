// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Projection_Radial4.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_PROJECTION_RADIAL4_H
#define USFM_PROJECTION_RADIAL4_H

#include "ceres/ceres.h"
#include "USfM/usfm_Projection.hpp"

namespace usfm {

	class ProjectionRadial4 : public Projection {
	public:
		static const int N_CAM_PARAMS = 7;
		static const int N_IMG_PARAMS = 6;

		ProjectionRadial4();
		ProjectionRadial4(Camera* camera);
		ProjectionRadial4(Image* camera);
		ProjectionRadial4(Camera* camera, Image* image);
		ProjectionRadial4(Camera* camera, Image* image, Point3D* X, Point2D* obs);

		// get number of parameters used by specified model
		int numCamParams();
		int numImgParams();

		// init the pointers to the vector of parameters
		void initOffsets(Camera * cam);
		void initOffsets(Image * img);

		// reorder the parameters
		void reorderParams(Camera * cam);
		void reorderParams(Image * img);

		// Create: ceres::CostFunction for an observation
		ceres::CostFunction * getCostFunction();

		// compute the residual of related 3D point projection and measured observation
		void computeResidual(double* res);

		// The projection function (used by Ceres to compose the Jacobian)
		template <typename T>
		bool operator()(const T* const X, const T* const cam, const T* const img, T* residuals) const
		{
			// the internal/external parameters
			T f = cam[0];									// focal length
			T pp[2] = { cam[1], cam[2] };					// principal point
			T k[4] = { cam[3], cam[4], cam[5], cam[6] };	// radial distortion parameters
			T aa[3] = { img[0] ,img[1] ,img[2] };			// angle axis
			T C[3] = { img[3] ,img[4] ,img[5] };			// camera center

			// the residual function
			T p[3], pC[3];									// auxiliary arrays
			pC[0] = X[0] - C[0];
			pC[1] = X[1] - C[1];
			pC[2] = X[2] - C[2];

			AARotatePoint(aa, pC, p);
			T x = p[0] / p[2];
			T y = p[1] / p[2];

			T dist = x*x + y*y;
			T distortion = T(1.0) + k[0]*dist + k[1]*dist*dist + k[2]*dist*dist*dist + k[3]*dist*dist*dist*dist;
			T predicted_x = f * x * distortion + pp[0];
			T predicted_y = f * y * distortion + pp[1];

			residuals[0] = predicted_x - T(_obs->_xy[0]);
			residuals[1] = predicted_y - T(_obs->_xy[1]);
			return true;
		}
	};

}

#endif //USFM_PROJECTION_RADIAL_H