// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Projection_InverseDivision2.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_PROJECTION_INVERSEDIVISION2_H
#define USFM_PROJECTION_INVERSEDIVISION2_H

#include "ceres/ceres.h"
#include "USfM/usfm_Projection.hpp"
#include "ceres/jet.h"

namespace usfm {

	class ProjectionInverseDivision2 : public Projection {
	public:
		static const int N_CAM_PARAMS = 5;
		static const int N_IMG_PARAMS = 6;

		ProjectionInverseDivision2();
		ProjectionInverseDivision2(Camera* camera);
		ProjectionInverseDivision2(Image* camera);
		ProjectionInverseDivision2(Camera* camera, Image* image);
		ProjectionInverseDivision2(Camera* camera, Image* image, Point3D* X, Point2D* obs);

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


		// The projection function - inverse division to division distance from principal point
		template <typename Q>
		bool iDiv2toDiv2(const Q k1, const Q  k2, const Q  s, Q* res) const {
			Q t2 = Q(2.519842099789746);
			Q t3 = k1 * k1;
			Q t4 = s * s;
			Q t5 = Q(1) / k2;
			Q t6 = (k2 * (Q(3) * sqrt(-((Q(3) * t4 * (k1 * (Q(144) * k2 - Q(4) * t3) 
				+ t4 * ((Q(256) * k2 - Q(128) * t3) * k2 + Q(16) * t3 * t3)) - Q(81) * k2) * t5)) + Q(27))
				- (Q(72) * k1 * k2 * t4) + (Q(2) * k1 * t3 * t4))*s;
			Q t7 = pow(t6, Q(0.33333333333333));     //t7 = exp(Q(1.0 / 3) * log(t6));
			t3 = pow(Q(4), Q(0.33333333333333)) * pow(t6,Q(0.666666666666666)) + (t2 * t3 * t4);
			t2 = k2 * t2;
			Q t8 = k1 * t7;
			t6 = Q(1) / pow(t6, Q(0.33333333333333));
			Q t9 = Q(1) / s;
			Q t10 = ((Q(12) * t2 * s - Q(4) * t8)*s + t3)*t5*t9*t6;
			Q t11 = sqrt(t10);
			t3 = (t11*t3);
			t7 = Q(2.449489742783178) * t7;
			t2 = (t2 * t11*t4);
			t4 = (Q(8) * t8*s*t11);
			t8 = Q(1) / sqrt(t10);
			Q t12 = sqrt(-Q(6) * (t3 + Q(12) * t7 + (Q(12) * t2) + t4) * t5*t9*t6*t8);
			Q t1 = Q(2.449489742783178) * t11;
			t2 = sqrt(Q(-6) * (t3 - Q(12) * t7 + (Q(12) * t2) + t4) * t5*t9*t6*t8);
      res[0] = (-Q(0.083333333333333) * (t12 + t1)) / s - Q(1);
      res[1] = (Q(0.083333333333333) * (t12 - t1)) / s - Q(1);
      res[2] = (-Q(0.083333333333333) * (t2 - t1)) / s - Q(1);
      res[3] = (Q(0.083333333333333) * (t2 + t1)) / s - Q(1);
      return res;
    }

		// The projection function (used by Ceres to compose the Jacobian)
		template <typename T>
		bool operator()(const T* const X, const T* const cam, const T* const img, T* residuals) const 
		{
			// the internal/external parameters
      const T f = cam[0];								// focal length
      const T pp[2] = { cam[1], cam[2] };				// principal point
      const T k[2] = { cam[3], cam[4] };					// radial distortion parameters
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

      const T s2 = x*x + y*y;
      const T s = sqrt(s2);
      T c0123[4];
      
      iDiv2toDiv2(k[0], k[1], s, c0123);
      const T c2_0123[4] = { c0123[0] * c0123[0], c0123[1] * c0123[1], c0123[2] * c0123[2], c0123[3] * c0123[3] };

      T c;
      if (c2_0123[0] < c2_0123[1] && c2_0123[0] < c2_0123[2] && c2_0123[0] < c2_0123[3])
        c = c0123[0];
      if (c2_0123[1] < c2_0123[0] && c2_0123[1] < c2_0123[2] && c2_0123[1] < c2_0123[3])
        c = c0123[1];
      if (c2_0123[2] < c2_0123[0] && c2_0123[2] < c2_0123[1] && c2_0123[2] < c2_0123[3])
        c = c0123[2];
      if (c2_0123[3] < c2_0123[0] && c2_0123[3] < c2_0123[1] && c2_0123[3] < c2_0123[2])
        c = c0123[3];


      const T predicted_x = f * (c + T(1)) * x + pp[0];
      const T predicted_y = f * (c + T(1)) * y + pp[1];

			residuals[0] = predicted_x - T(_obs->_xy[0]);
			residuals[1] = predicted_y - T(_obs->_xy[1]);
			return true;
		}
	};


	/*s = sqrt(sum(un. ^ 2));
	c = iDiv2toDiv2(cp(4), cp(5), s) . / ([1; 1; 1; 1] * s);
	c(abs(imag(c))>1e-10) = inf;
	c = real(c);
	[~, sel] = min(abs(c - 1));
	c = arrayfun(@(i)c(sel(i), i), 1:size(sel, 2));
	u = cp(1) * (un.*c) + cp(2:3);*/

	/**/


}

#endif //USFM_PROJECTION_INVERSEDIVISION2_H