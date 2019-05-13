// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Projection_Simple_Pinhole.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_PROJECTION_SIMPLE_PINHOLE_H
#define USFM_PROJECTION_SIMPLE_PINHOLE_H

#include "ceres/ceres.h"
#include "USfM/usfm_Projection.hpp"

namespace usfm {

	class ProjectionSimplePinhole : public Projection {
	public:
		static const int N_CAM_PARAMS = 3;
		static const int N_IMG_PARAMS = 6;

		ProjectionSimplePinhole();
		ProjectionSimplePinhole(Camera* camera);
		ProjectionSimplePinhole(Image* camera);
		ProjectionSimplePinhole(Camera* camera, Image* image);
		ProjectionSimplePinhole(Camera* camera, Image* image, Point3D* X, Point2D* obs);

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
			const T f = cam[0];								// focal length
			const T pp[2] = { cam[1], cam[2] };				// principal point
			const T aa[3] = { img[0] ,img[1] ,img[2] };		// angle axis
			const T C[3] = { img[3] ,img[4] ,img[5] };		// camera center

			// the computation of the residuals 
			T p[3], pC[3];								// auxiliary arrays
			pC[0] = X[0] - C[0];
			pC[1] = X[1] - C[1];
			pC[2] = X[2] - C[2];

			AARotatePoint(aa, pC, p);
			const T x = p[0] / p[2];
			const T y = p[1] / p[2];

			const T predicted_x = f * x + pp[0];
			const T predicted_y = f * y + pp[1];

			residuals[0] = predicted_x - T(_obs->_xy[0]);
			residuals[1] = predicted_y - T(_obs->_xy[1]);
			return true;
		}
	};

}

#endif //USFM_PROJECTION_SIMPLE_PINHOLE_H