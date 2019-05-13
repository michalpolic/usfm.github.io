// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Algorithm_NBUP.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/


#include "USfM/usfm_Algorithm_NBUP.hpp"
#include "USfM/usfm_InputCovarinace_Factory.hpp"

namespace usfm {

	void Algorithm_NBUP::compute(Scene& scene, Statistic& statistic) {
		// initialize the projections, camera and image models and sort parameters
		init(scene);
		std::cout.precision(17);

		// jacobian of the projection function
		computeJacobian(scene, statistic);
		//std::cout << scene._jacobian << "\n\n\n";
		//std::cout << "J: " << isNan(scene._jacobian) << "\n";

		// the nullspace
		DM &H = scene._H;		// the nullspace of jacobian of projection function
		computeJacobianNullspace(scene, H);
		//std::cout << scene._jacobian * H << "\n\n\n";
		//std::cout << "H: " << isNan(H) << "\n\n\n";
		//std::cout << H << "\n\n\n";


		// scale the Jacobian & nullspace H
		SM Js, Sa, Sb;			// scaled Jacobian: Js = J * Sa,  S = [Sa 0; 0 Sb]
		DM Hs;					// scaled nullspace: Hs = Sa * H * Sb
		scaleJacobianAndNullspace(scene._jacobian, H, Js, Sa, Hs, Sb);
		//std::cout << Js << "\n\n\n";
		//std::cout << Sa.diagonal() << "\n\n\n";
		//std::cout << Hs << "\n\n\n";
		//std::cout << Sb.diagonal() << "\n\n\n";
		//std::cout << "Js: " << isNan(Js) << "\n";
		//std::cout << "Hs: " << isNan(Hs) << "\n";

		// compute the covariance and precision matrix of input observations
		std::shared_ptr<InputCovariance> in_cov_est = InputCovariance_Factory::initInputCovariance(scene._settings._e_in_cov);
		in_cov_est->compute(scene, statistic);
		//std::cout << scene._in_cov << "\n\n\n";
		//std::cout << "in_cov: " << isNan(scene._in_cov) << "\n";

		// the information matrix
		SM M(Js.transpose() * scene._in_precision_mat * Js);
		//std::cout << "M: " << isNan(M) << "\n";
		//std::cout << M << "\n";
		
		// the Schur complement of block M related to point parameters
		SM invAs, Bs;			// inversion of scaled block A, scaled block B of matrix Q = [As, Bs; Bs^T, Ds] = S [J^T J, H; H^t 0] S
		DM &Zs = scene._iZ;		// scaled Schur complement matrix (it will be used to hold the covariance matrix for camera paraneters)
		computeSchurComplementUsingNullspace(scene, M, Hs, Zs, invAs, Bs);
		//std::cout << Zs << "\n";
		//std::cout << "Zs: " << isNan(Zs) << "\n";

		// fast inversion of Schur complement matrix, 
		// it works inside the matrix because bacause of the size for thousands of cameras 
		DM &iZs = Zs;
		inversion(iZs);
		//std::cout << iZs << "\n";			

		// uncertainty of points 
		std::vector<Eigen::Matrix3f> &cov_pts = scene._cov_pts;
		SM Ys = invAs * Bs;	
		//std::cout << Ys << "\n";
		computeUncertaintyOfPoints(scene._points3D.size(), Ys, iZs, invAs, cov_pts);
		//for (int i = 0; i < cov_pts.size(); ++i)
		//	std::cout << cov_pts[i] << "\n\n";

		// unscale the camera covariance matrix & covariance matrices of points
		DM &iZ = iZs;	// unscaled matrix iZs
		unscaleCameraCovarianceMatrix(Sa, Sb, iZ);
		//std::cout << iZ << "\n";

		// unscale the covariance matrices for points
		unscalePointsCovarianceMatrices(Sa, cov_pts);
		//for (int i = 0; i < cov_pts.size(); ++i)
		//	std::cout << cov_pts[i] << "\n\n";

		// close and clear
		finish();
	}

	// Compute the Schur complement with known nullspace
	void Algorithm_NBUP::computeSchurComplementUsingNullspace(const Scene &scene, const SM &Ms, const DM &Hs, DM &Zs, SM &invAs, SM &Bs) {
		// create extended matrix MsH = [M H; H^T 0]
		SM MsH(Ms.rows() + 7, Ms.cols() + 7);	
		std::vector<Tri> MsH_T;
		for (int i = 0; i<Ms.outerSize(); ++i){
			for (Eigen::SparseMatrix<double>::InnerIterator it(Ms, i); it; ++it)
				MsH_T.push_back(Tri(it.row(), it.col(), it.value()));	
		}
		for (int i = 0; i < 7; ++i) {
			for (int j = 0; j < Hs.rows(); ++j){
				MsH_T.push_back(Tri(j, Ms.cols() + i, Hs(j, i)));
				MsH_T.push_back(Tri(Ms.cols() + i, j, Hs(j, i)));
			}
		}
		MsH.setFromTriplets(MsH_T.begin(), MsH_T.end());
		//std::cout << MsH << "\n\n\n";

		// call the parent version on extended M
		computeSchurComplement(scene._points3D.size(), MsH, Zs, invAs, Bs);
	}
}