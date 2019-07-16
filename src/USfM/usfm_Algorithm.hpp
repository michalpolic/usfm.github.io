// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Algorithm.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_ALGORITHM_H
#define USFM_ALGORITHM_H

#include "USfM/usfm_data_Scene.hpp"
#include "USfM/usfm_Statistics.hpp"
#include <Eigen/Core>
#include <Eigen/SparseCore>
#ifdef USE_MATLAB
  #include "mex.h"
#endif

typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SM;
typedef Eigen::SparseMatrix<double, Eigen::RowMajor> SM_row;
typedef Eigen::MatrixXd DM;
typedef Eigen::Triplet<double> Tri;


namespace usfm {
	
	class Algorithm {
	public:
		virtual void compute(Scene& scene, Statistic& statistic) = 0;

	protected:
		ceres::Solver::Options defaultOprimizationOptions() const;
		void computeJacobian(Scene& scene, Statistic& statistic) const;
		void composeCeresProblem(Scene& scene, Statistic& statistic, ceres::Problem* problem) const;
		void setupJacobianBlocksOrder(Scene& scene, std::vector<double*> &parameter_blocks) const;

		// auxilary functions 
		void init(Scene& scene) const;
		void finish() const;

		void computeScaleFromRight(const SM &A, SM &S) const;
		void computeScaleFromRight(const DM &A, SM &S) const;

		void computeJacobianNullspace(Scene &scene, DM &H) const;
		void threeRowsFormEachImageJacobian(const Scene &scene, SM_row &Jrows) const;
		void blockDiagonalRotationJacobian(const Scene &scene, const SM_row &Jrows, SM &Jdiag) const;
		
		void inv3x3(const SM &A, SM &invA)  const;
		void inversion(DM &A);
		void scaleJacobianAndNullspace(const SM_row &J, const DM &H, SM &Js, SM &Sa, DM &Hs, SM &Sb);

		void computeSchurComplement(const int num_points, const SM &Ms, DM &Zs, SM &invAs, SM &Bs);
		
		void unscaleCameraCovarianceMatrix(const SM &Sa, const SM &Sb, DM &iZs);
		void unscalePointsCovarianceMatrices(SM &S, std::vector<Eigen::Matrix3f> &cov_pts);

		void computeUncertaintyOfPoints(const int num_pts, const SM &Bs, const DM &iZs, const SM &iAs, std::vector<Eigen::Matrix3f>& cov_pts);

		// test functions
		bool isNan(const SM &A) const;
		bool isNan(const SM_row &A) const;
		bool isNan(const DM &A) const;
	};
}

#endif