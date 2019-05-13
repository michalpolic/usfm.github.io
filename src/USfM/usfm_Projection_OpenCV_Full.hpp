// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Projection_OpenCV_Full.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_PROJECTION_OPENCV_FULL_H
#define USFM_PROJECTION_OPENCV_FULL_H

#include "ceres/ceres.h"
#include "USfM/usfm_Projection.hpp"

namespace usfm {

	class ProjectionOpenCVFull : public Projection {
	public:
		static const int N_CAM_PARAMS = 12;
		static const int N_IMG_PARAMS = 6;

		ProjectionOpenCVFull();
		ProjectionOpenCVFull(Camera* camera);
		ProjectionOpenCVFull(Image* camera);
		ProjectionOpenCVFull(Camera* camera, Image* image);
		ProjectionOpenCVFull(Camera* camera, Image* image, Point3D* X, Point2D* obs);

		void initOffsets(Camera * cam);
		void reorderParams(Camera * cam);

		void initOffsets(Image * img);
		void reorderParams(Image * img);

		int numCamParams();
		int numImgParams();

		ceres::CostFunction * getCostFunction();

		// compute the residual of related 3D point projection and measured observation
		void computeResidual(double* res);

		// The projection function (used by Ceres to compose the Jacobian)
		template <typename T>
		bool operator()(const T* const X, const T* const cam, const T* const img, T* residuals) const
		{
			// the internal/external parameters			fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
			const T f[2] = { cam[0], cam[1] };				// focal length: fx, fy
			const T pp[2] = { cam[2], cam[3] };				// principal point: cx, cy
			const T k[6] = { cam[4], cam[5], cam[8], cam[9], cam[10], cam[11] };	// radial distortion parameters: k1, k2, k3, k4, k5, k6
			const T tk[2] = { cam[6], cam[7] };				// tangetial distortion parameters: p1, p2
			const T aa[3] = { img[0] ,img[1] ,img[2] };		// angle axis
			const T C[3] = { img[3] ,img[4] ,img[5] };		// camera center

			// the residual function
			T p[3], pC[3];								// auxiliary arrays
			pC[0] = X[0] - C[0];
			pC[1] = X[1] - C[1];
			pC[2] = X[2] - C[2];

			AARotatePoint(aa, pC, p);
			T x = p[0] / p[2];
			T y = p[1] / p[2];

			const T x2 = x*x;
			const T xy = x*y;
			const T y2 = y*y;
			const T r2 = x2 + y2;
			const T r4 = r2 * r2;
			const T r6 = r4 * r2;
			const T radial =	(T(1) + k[0] * r2 + k[1] * r4 + k[2] * r6) /
								(T(1) + k[3] * r2 + k[4] * r4 + k[5] * r6);
			x = x * radial + T(2) * tk[0] * xy + tk[1] * (r2 + T(2) * x2);
			y = y * radial + T(2) * tk[1] * xy + tk[0] * (r2 + T(2) * y2);

			const T predicted_x = f[0] * x + pp[0];
			const T predicted_y = f[1] * y + pp[1];

			residuals[0] = predicted_x - T(_obs->_xy[0]);
			residuals[1] = predicted_y - T(_obs->_xy[1]);
			return true;
		}
	};

}

#endif //USFM_PROJECTION_OPENCV_FISHEYE_H