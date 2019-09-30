// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_Algorithm.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "usfm_Algorithm.hpp"
#include "USfM/usfm_Projection.hpp"
#include "USfM/usfm_Projection_Factory.hpp"
#include <thread>
#ifdef USE_MKL
	#include "mkl.h"
#endif 
//#ifdef USE_LAPACK
//	#include "openblas/cblas.h"
//	#include "f2c.h" 
//	#include "clapack.h"
//#endif

#include <cmath>

namespace usfm {
	
	inline bool point2DComparator(Point2D first, Point2D second) { return ((first._xy[0] - second._xy[0] < 0.01) ? (first._xy[1] < second._xy[1]) : (first._xy[0] < second._xy[0])); }
	
	void SceneFiltering(Scene &scene) {

		//creating point3D usage map <id, number_of_uses>
		std::map<int, int> point3D_usage_global;
		for (auto& p3d : scene._points3D) {
			point3D_usage_global.emplace(p3d.first,0);
		}

		//first filtering cycle - recognition of unvalid 3D points, afterwards deletenig 2Dpoints corresponding to them
		for (auto& image : scene._images) {
			std::map<int, short> point3D_rarity;	//could use different type, because of unused value here, bud map has easily accessible "contains" function
			for (int i = 0; i < image.second._point2D.size(); ++i) {
				int p3d_id = image.second._point2D[i]._id_point3D;
				if (!(scene._points3D.count(p3d_id))) {
					image.second._point2D.erase(image.second._point2D.begin() + i);
				}
				if (!point3D_rarity.count(p3d_id)) {
					point3D_usage_global[p3d_id] += 1;
					point3D_rarity.emplace(p3d_id, 0);
				}
				else {
					image.second._point2D.erase(image.second._point2D.begin() + i);
				}
			}

		}
		
		//second filtering cycle -- recognizing and multiple observations and afterwards deleting them based on number of uses of referred 3Dpoint
		//after deleting follows reducing the usage number 
		for (auto& image : scene._images) {
			std::sort(image.second._point2D.begin(), image.second._point2D.end(), point2DComparator);

			for (int i = 0; i < (image.second._point2D.size() - 1); ++i) {
				if (((image.second._point2D[i]._xy[0] - image.second._point2D[i + 1]._xy[0]) < 0.01) && ((image.second._point2D[i]._xy[1] - image.second._point2D[i + 1]._xy[1]) < 0.01)) {
					if (point3D_usage_global[image.second._point2D[i]._id_point3D] < point3D_usage_global[image.second._point2D[i + 1]._id_point3D]) {
						point3D_usage_global[image.second._point2D[i]._id_point3D] -= 1;
						image.second._point2D.erase(image.second._point2D.begin() + i);
					}
					else {
						point3D_usage_global[image.second._point2D[i + 1]._id_point3D] -= 1;
						image.second._point2D.erase(image.second._point2D.begin() + i + 1);
					}
				}
			}
		}

		//third filtering cycle - deleting of 3D points used less than two times

		for (auto& p3d : point3D_usage_global) {
			if (p3d.second < 2) {
				scene._points3D.erase(p3d.first);
			}
		}

		//fourth and last filtering cycle - deleting of invalid observations, which invalidity is due to removed 3D points

		for (auto& image : scene._images) {
			for (int i = 0; i < image.second._point2D.size(); ++i) {
				if (!(scene._points3D.count(image.second._point2D[i]._id_point3D))) {
					//image.second._point2D[i]._id_point3D = -1;				//unnecessary action, I suppose
					image.second._point2D.erase(image.second._point2D.begin() + i);
				}
			}
		}
	}


	void Algorithm::init(Scene& scene) const
	{
		//SceneFiltering(scene);

		// init multithread routines
		unsigned int num_threads = std::thread::hardware_concurrency();
		if (num_threads != 0) {
		#if defined(_OPENMP)
			omp_set_num_threads(num_threads);
		#endif
			Eigen::setNbThreads(num_threads);
		}

		// init projections
		scene._projections.clear();
		for (auto &c : scene._cameras) {
			// reorder all camera parameters according their models
			std::shared_ptr<Projection> proj = Projection_Factory::createProjection(&c.second, NULL, NULL, NULL);
			proj->reorderParams(&c.second);
		}
		for (auto &i : scene._images) {
			Image* image = &(i.second);
			Camera* camera = &(scene._cameras[image->_cam_id]);

			// optional step for comparison -> sort the observations according point ids
			std::sort(image->_point2D.begin(), image->_point2D.end());

			// reorder all image parameters according their models
			std::shared_ptr<Projection> proj = Projection_Factory::createProjection(NULL, image, NULL, NULL);
			proj->reorderParams(image);

			// observations -> projections
			for (auto &p2D : image->_point2D) {
				Point3D *p3D = &scene._points3D[p2D._id_point3D];
				std::shared_ptr<Projection> projection = Projection_Factory::createProjection(camera, image, p3D, &p2D);
				scene._projections.push_back(projection);
			}
		}
	}

	void Algorithm::finish() const
	{
		// close all resources, e.g., CUDA ...
	}

	// Compose the Ceres problem
	void Algorithm::composeCeresProblem(Scene& scene, Statistic& statistic, ceres::Problem* problem) const
	{
		statistic.updateLastTimestamp();
		for (std::shared_ptr<Projection> proj : scene._projections) {
			ceres::CostFunction* cost_f = proj->getCostFunction();
			ceres::LossFunction* loss_f = NULL;
			if (scene._settings._robust_lost)
				ceres::LossFunction* loss_f = new ceres::CauchyLoss(1.0);
			problem->AddResidualBlock(cost_f, loss_f, 
				proj->_X->_X, proj->_camera->_parameters.data(), proj->_image->_parameters.data());
		}
		statistic.addTimestampItem("Composition of the ceres::Problem");
	}

	// Compose the Jacobian in the order: points in 3D, camera external parameters, camera internal parameters
	void Algorithm::setupJacobianBlocksOrder(Scene& scene, std::vector<double*> &parameter_blocks) const
	{
		for (auto &p3D : scene._points3D)
			parameter_blocks.push_back(p3D.second._X);
		for (auto &i : scene._images)
			parameter_blocks.push_back(i.second._parameters.data());
		for (auto &c : scene._cameras)
			parameter_blocks.push_back(c.second._parameters.data());
	}

	// Compute the Jacobian for given scene (given parameters and projection functions)
	void Algorithm::computeJacobian(Scene& scene, Statistic& statistic) const
	{
		// convert Scene to ceres::Problem
		ceres::Problem problem;
		composeCeresProblem(scene, statistic, &problem);		
		
		// setup the order of Jacobian blocks
		std::vector<double*> parameter_blocks;
		setupJacobianBlocksOrder(scene, parameter_blocks);	
					
		// ceres options for computing the Jacobian
		double cost = 0.0;
		ceres::Problem::EvaluateOptions evalOpt;
		evalOpt.parameter_blocks = parameter_blocks;
		if (scene._settings._robust_lost)
			evalOpt.apply_loss_function = true;
		else 
			evalOpt.apply_loss_function = false;


		// optimize - we assume the problem close to optimal solution
		if (scene._settings._run_opt){
			ceres::Solver::Options options = defaultOprimizationOptions();
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			//std::cout << summary.FullReport() << "\n";
		}

		// evaluate Jacobain 
		ceres::CRSMatrix J = ceres::CRSMatrix();
		statistic.updateLastTimestamp();
		problem.Evaluate(evalOpt, &cost, NULL, NULL, &J);
		statistic.addTimestampItem("Composition of the Jacobian");

		// save to scene
		scene._jacobian = Eigen::Map<SM_row>(J.num_rows, J.num_cols, J.cols.size(), J.rows.data(), J.cols.data(), J.values.data());
	}

	// Return default optimization options
	ceres::Solver::Options Algorithm::defaultOprimizationOptions() const {
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; // DENSE_SCHUR;	//DENSE_SCHUR;
		options.minimizer_progress_to_stdout = false;
		options.max_num_iterations = 100;
		return options;
	}


	// Compute the scale S of sparse matrix A from right
	void Algorithm::computeScaleFromRight(const SM &A, SM &S) const {
		S.resize(A.cols(), A.cols());
		S.reserve(A.cols());
		std::vector<Tri> STriplets;
		for (int i = 0; i < A.cols(); ++i) {		// all columns
			double ss = 0;							// sum of squares
			for (int j = A.outerIndexPtr()[i]; j < A.outerIndexPtr()[i + 1]; ++j)  // all rows at column
				ss += A.valuePtr()[j] * A.valuePtr()[j];
			STriplets.push_back(Tri(i,i,1/sqrt(ss)));
		}
		S.setFromTriplets(STriplets.begin(), STriplets.end());
	}

	// Compute the scale S of dense matrix A from right
	void Algorithm::computeScaleFromRight(const DM &A, SM &S) const {
		S.resize(A.cols(), A.cols());
		S.reserve(A.cols());
		std::vector<Tri> STriplets;
		Eigen::VectorXd eucl_norm = A.colwise().norm();
		for (int i = 0; i < A.cols(); ++i)
			STriplets.push_back(Tri(i, i, 1/eucl_norm(i)));
		S.setFromTriplets(STriplets.begin(), STriplets.end());
	}
 
	// Create matrix Jrows composed of first 3 rows of each image Jacobian 
	// -> first step for composition of linear system of equations to compute Jacobian nullspace H
	void Algorithm::threeRowsFormEachImageJacobian(const Scene &scene, SM_row &Jrows) const {
		const SM_row &J = scene._jacobian;
		Jrows.resize(3 * scene._images.size(), J.cols());
		int i = 0, image_offset = 0;
		std::vector<Tri> JrowsTriplets;
		std::vector<double> first_values;
		for (auto image : scene._images) {
			
			// COLMAP produce some detections multiple times wich results in NaNs and rank deficient matrices
			// Test the thrid row if it is the same is the first one, and use different one if needed
			int t = 0;
			first_values.clear();
			int column_id = J.outerIndexPtr()[image_offset];		// first row of image Jacobian
			for (int j = 0; j < 3; ++j)
				first_values.push_back(J.valuePtr()[column_id + j]);

			// use 3 rows of each image Jacobian
			for (int k = 0; k < 3; ++k){		

				// third row of image Jacobian
				if (k == 2) {					
					bool the_same_rows = true;	
					while(the_same_rows){
						int column_id = J.outerIndexPtr()[image_offset + k];
						for (int j = 0; j < 3; ++j) {			// test the first 3 values (there must be at least 3 values)
							if (abs(first_values[j] - J.valuePtr()[column_id + j]) > abs(first_values[j]*0.005))	// there must be change at least 0.5% 
								the_same_rows = false;
						}
						if (the_same_rows){						// use next row if it is similar to the first one
							k += 2; 
							t += 2;
							if (k >= (image.second._point2D.size() * 2))
								throw std::runtime_error("Wrong COLMAP scene, one of the images has only one repetitive 2D point.");
						}
					}
				}

				// Copy the k-th row of Jacobian related to i-th image
				for (int j = J.outerIndexPtr()[image_offset + k]; j < J.outerIndexPtr()[image_offset + k + 1]; ++j)
					JrowsTriplets.push_back(Tri(3 * i + k - t, J.innerIndexPtr()[j], J.valuePtr()[j]));
			}
			image_offset += image.second._point2D.size() * 2;
			i++;
		}
		Jrows.setFromTriplets(JrowsTriplets.begin(), JrowsTriplets.end());
	}

	// Create submatrix "Jdiag" of blocks of the Jacobian related to rotation parameters 
	// It allows computation of unknown nullspace related to the rotation parameters: Hr = Jdiag^-1 * B
	void Algorithm::blockDiagonalRotationJacobian(const Scene &scene, const SM_row &Jrows, SM &Jdiag) const {
		Jdiag.resize(3 * scene._images.size(), 3 * scene._images.size());
		
		// each projection may have different number of parameters as the projection function is not fixed
		// but all the points has 3 parameters and the rows of Jrows start with derivative of point parameters 
		std::vector<Tri> JdiagTriplets;
		for (int row = 0; row < Jrows.rows(); ++row) {
			int col = Jrows.outerIndexPtr()[row] + 3;
			int image = floor(row / 3);
			for (int j = 0; j < 3; ++j)
				JdiagTriplets.push_back(Tri(row, 3 * image + j, Jrows.valuePtr()[col + j]));
		}
		Jdiag.setFromTriplets(JdiagTriplets.begin(), JdiagTriplets.end());
	}

	// Compute the inversion of block diagonal matrix with blocks of size 3x3
	// *It is used to compute inversion of Jdiag and block of Inform. matrix related to points in 3D
	void Algorithm::inv3x3(const SM &A, SM &invA) const {
		invA = A;		// clone all the indexes wich remain unchanged, i.e., the column and row indexes will be the same
		double const *val = A.valuePtr();
		double *Aval = invA.valuePtr();
		for (int i = 0; i < A.nonZeros(); i = i + 9) {
			double t1 = val[i+4] * val[i+8] - val[i+5] * val[i+7];
			double t2 = val[i+3] * val[i+7] - val[i+4] * val[i+6];
			double t3 = -val[i+3] * val[i+8] + val[i+5] * val[i+6];
			double t4 = 1 / (val[i] * t1 + val[i+1] * t3 + val[i+2] * t2);
			Aval[i + 0] = t1 * t4;
			Aval[i + 1] = -(val[i+1] * val[i+8] - val[i+2] * val[i+7]) * t4;
			Aval[i + 2] = (val[i+1] * val[i+5] - val[i+2] * val[i+4]) * t4;
			Aval[i + 3] = t3 * t4;
			Aval[i + 4] = (val[i] * val[i+8] - val[i+2] * val[i+6]) * t4;
			Aval[i + 5] = -(val[i] * val[i+5] - val[i+2] * val[i+3]) * t4;
			Aval[i + 6] = t2 * t4;
			Aval[i + 7] = -(val[i] * val[i+7] - val[i+1] * val[i+6]) * t4;
			Aval[i + 8] = (val[i] * val[i+4] - val[i+1] * val[i+3]) * t4;
		}
	}

	// Compute the nullspace for given scene and Jacobian
	void Algorithm::computeJacobianNullspace(Scene &scene, DM &H) const {
		if (scene._jacobian.nonZeros() == 0 && scene._jacobian.rows() == 0 && scene._jacobian.cols() == 0)
			throw std::runtime_error("The Jacobian have to be computed before calling computeJacobianNullspace().");

		NumSceneParams npar = scene.getNumSceneParams();
		H.resize(npar._numCamPar + npar._numImgPar + npar._numPtsPar, 7);		// we have 7 DoF defined by similarity function
		H.setZero();

		// Fill the known values of the nullspace H
		// Points in 3D
		int i = 0;
		for (auto &point : scene._points3D) {
			double *X = point.second._X;
			H.block(i * 3, 0, 3, 7) <<
				1, 0, 0, 0, -X[2], X[1], X[0],
				0, 1, 0, X[2], 0, -X[0], X[1],
				0, 0, 1, -X[1], X[0], 0, X[2];
			i++;
		}

		// Images - external camera parameters
		i = 0;
		int ptsOffset = 3 * scene._points3D.size();
		for (auto &image : scene._images) {
			double *C = image.second.value(e_C);
			H.block(i + ptsOffset + 3, 0, 3, 7) <<
				1, 0, 0, 0, -C[2], C[1], C[0],
				0, 1, 0, C[2], 0, -C[0], C[1],
				0, 0, 1, -C[1], C[0], 0, C[2];

			// setup the row offset according selected image model
			std::shared_ptr<Projection> proj = Projection_Factory::createProjection(&(image.second));
			i += proj->numImgParams();
		}

		// *Internal camera parameters are not influenced by similarity function, i.e., equal zero
		//std::cout << "H: " << isNan(H) << "\n\n\n";

		// Compose submatrix of J with enough contains to compute unknown values of H
		SM_row Jrows;
		SM Jdiag, invJdiag;
		threeRowsFormEachImageJacobian(scene, Jrows);
		//std::cout << "\n\n\nJrows = [" << Jrows.block(0,scene._points3D.size()*3,33,66) << "];\n\n";

		blockDiagonalRotationJacobian(scene, Jrows, Jdiag);
		//std::cout << "Jrows: " << isNan(Jrows) << "\n\n\n";
		//std::cout << "Jdiag: " << isNan(Jdiag) << "\n\n\n";
		//std::cout << "\n\n\nJdiag = [" << Jdiag << "];\n\n";
		
		inv3x3(Jdiag, invJdiag);
		//std::cout << "\n\n\ninvJdiag = [" << invJdiag << "];\n\n";
		//std::cout << "invJdiag: " << isNan(invJdiag) << "\n\n\n";

		//std::cout << "\n\n\ninvJdiag = [" << invJdiag << "];\n\n";
		DM H_rotation = - invJdiag * Jrows * H.block(0,3,H.rows(),3);		// B = Jrows * H
		//std::cout << "\n\n\nH_rotation = [" << H_rotation << "];\n\n";
		//std::cout << "H_rotation: " << isNan(H_rotation) << "\n\n\n";

		// Fill the unknown blocks in H
		int j = 0;
		i = 0;
		for (auto &image : scene._images) {
			double *C = image.second.value(e_C);
			H.block(i + ptsOffset, 3, 3, 3) = H_rotation.block(3*j,0,3,3);

			// setup the row offset according selected image model
			std::shared_ptr<Projection> proj = Projection_Factory::createProjection(&(image.second));
			i += proj->numImgParams();
			j++;
		}
		//std::cout << "\n\n\nH = [" << H << "];\n\n";
		//std::cout << "\n\n\n" << scene._jacobian * H << "\n\n";
	}

	// Scale the Jacobian and its nullspace to avoid numerical errors 
	void Algorithm::scaleJacobianAndNullspace(const SM_row &J, const DM &H, SM &Js, SM &Sa, DM &Hs, SM &Sb) {
		// scale Jacobian: Js = J * Sa
		Js = SM(J);	// reorder param., we need column major sparse matrix 	
		computeScaleFromRight(Js, Sa);
		Js = Js * Sa;

		// scale nullspace: Hs = Sa * H * Sb
		Hs = Sa * H;
		computeScaleFromRight(Hs, Sb);
		Hs = Hs * Sb;
	}
	
	// Compute the Schur from matrix M (the function assume ordering: points, extrinic, intrinsic parameters)
	void Algorithm::computeSchurComplement(const int num_points, const SM &Ms, DM &Zs, SM &invAs, SM &Bs) {	
		// Compose the submatrices Ms = [As Bs; Bs^T Ds]   
		// Matrix As is related to point parameters, Ds is related to cameras and Bs is related to both
		int num_pt_params = 3 * num_points;
		int num_cam_params = Ms.cols() - num_pt_params;
		SM As(num_pt_params, num_pt_params);
		SM Ds(num_cam_params, num_cam_params);
		invAs.resize(num_pt_params, num_pt_params);
		Bs.resize(num_pt_params, num_cam_params);
		std::vector<Tri> AsT, BsT, DsT;
		for (int i = 0; i < Ms.outerSize(); ++i) {
			for (Eigen::SparseMatrix<double>::InnerIterator it(Ms, i); it; ++it) {
				if (it.row() < num_pt_params && it.col() < num_pt_params)		// As
					AsT.push_back(Tri(it.row(), it.col(), it.value()));
				if (it.row() < num_pt_params && it.col() >= num_pt_params)		// Bs
					BsT.push_back(Tri(it.row(), it.col() - num_pt_params, it.value()));
				if (it.row() >= num_pt_params && it.col() >= num_pt_params)		// Ds
					DsT.push_back(Tri(it.row() - num_pt_params, it.col() - num_pt_params, it.value()));
			}
		}
		As.setFromTriplets(AsT.begin(), AsT.end());
		Bs.setFromTriplets(BsT.begin(), BsT.end());
		Ds.setFromTriplets(DsT.begin(), DsT.end());

		// Invert block diagonal matrix A
		inv3x3(As, invAs);
		//std::cout << invAs << "\n\n\n";
		//std::cout << Ds << "\n\n\n";
		//std::cout << Bs << "\n\n\n";

		// Compute Z
		Zs = DM(Ds - Bs.transpose() * invAs * Bs);
	}

	bool Algorithm::isNan(const SM_row &A) const {
		SM B(A);
		return isNan(B);
	}

	bool Algorithm::isNan(const SM &A) const {
		for (int i = 0; i < A.outerSize(); ++i) {
			for (Eigen::SparseMatrix<double>::InnerIterator it(A, i); it; ++it)
				if (isnan(it.value()))
					return true;
		}
		return false;
	}

	bool Algorithm::isNan(const DM &A) const {
		for (int i = 0; i < A.rows() * A.cols(); i++) {
			if (isnan(A.data()[i]))
				return true; 
		}
		return false;
	}


	// invert the square fullrank matrix A by LU decomposition
	void Algorithm::inversion(DM &A) {
    #ifdef USE_MKL
      // LU
		  MKL_INT N = A.rows();
		  MKL_INT *ipiv = (MKL_INT*) malloc(N * sizeof(MKL_INT));
		  if (LAPACKE_dgetrf(LAPACK_COL_MAJOR, N, N, A.data(), N, ipiv) != 0)
			  std::cerr << "LU decomposition error.";
		  // inverse
		  if (LAPACKE_dgetri(LAPACK_COL_MAJOR, N, A.data(), N, ipiv) != 0)
			  std::cerr << "Lapack inverse error.";
		  free(ipiv);
    #else
  //    #ifdef USE_LAPACK    
		//integer info = 0;
		//integer N = A.rows();
		//integer *ipiv = (integer*)malloc(N * sizeof(integer));
  //      // LU
		//dgetrf_(&N, &N, A.data(), &N, ipiv, &info);
  //      if (info != 0)
  //        std::cerr << "LU decomposition error.";
  //      // inverse
  //      double workdim = 0;
		//integer lwork = -1;
  //      dgetri_(&N, A.data(), &N, ipiv, &workdim, &lwork, &info);
  //      if (info != 0)
  //        std::cerr << "Lapack size of work array error.";
  //      lwork = (integer) workdim;
  //      double *work = (double*)malloc(workdim * sizeof(double));
  //      dgetri_(&N, A.data(), &N, ipiv, work, &lwork, &info);
  //      if (info != 0)
  //        std::cerr << "Lapack size of work array error.";
  //      free(ipiv);
  //    #else
        // Eigen
        A = A.inverse();
  // #endif
    #endif
	}


	// uscale the dense matrix A by scale on diagonal S = [Sa 0; 0 Sb]
	void Algorithm::unscaleCameraCovarianceMatrix(const SM &Sa, const SM &Sb, DM &iZs) {
		Eigen::VectorXd s(Sa.rows() + Sb.rows());
		s.block(0, 0, Sa.rows(), 1) = Sa.diagonal();
		s.block(Sa.rows(), 0, Sb.rows(), 1) = Sb.diagonal();
		SM S(s.block(s.rows() - iZs.rows(), 0, iZs.rows(), 1).asDiagonal());
		iZs = S * iZs * S;
	}


	// compute the uncertainty of points from known uncertatinty of cameras
	void Algorithm::computeUncertaintyOfPoints(const int num_pts, const SM &Ys, const DM &iZs, const SM &iAs, std::vector<Eigen::Matrix3f>& cov_pts) {
		cov_pts.resize(num_pts);

		// propagation to points is less numerical senzitive and we can compute it in floats to speed it up
		Eigen::SparseMatrix<float,Eigen::RowMajor> fYs = Eigen::SparseMatrix<float, Eigen::RowMajor>(Ys.cast<float>());
		Eigen::MatrixXf fiZs = iZs.cast<float>();
		float *iA = (float*) malloc(sizeof(float) * iAs.nonZeros());
		for (int i = 0; i < iAs.nonZeros(); ++i)
			iA[i] = static_cast<float>(iAs.valuePtr()[i]);

		// parallel sparse multiplication of needed blocks
		const int *rowsY = fYs.outerIndexPtr();
		const int *colsY = fYs.innerIndexPtr();
		const float *valuesY = fYs.valuePtr();
		const float *iZ = fiZs.data();

		int n = fiZs.cols();			// munber of camera parameters
		int nrowsY = fYs.rows();
		std::vector<std::vector<float>> h(nrowsY);

		#pragma omp parallel for
		for (int i = 0; i < nrowsY; ++i) {				// rows of Y
			int row_from = rowsY[i];
			int row_to = rowsY[i + 1];
			int ncols = row_to - row_from;
			h[i].resize(ncols);

			for (int j = 0; j < ncols; ++j) {		// all columns on the row
				int col_id = colsY[row_from + j];
				float hi = 0;
				for (int k = 0; k < ncols; ++k) 	// multiplication of row j-th column iZ
					hi += valuesY[k + row_from] * iZ[n*col_id + colsY[k + row_from]];
				h[i][j] = hi;
			}
		}

		#pragma omp parallel for
		for (int i = 0; i < num_pts; ++i) {				// each point m has values on three rows h
			std::vector<float> &r0 = h[3 * i + 0];
			std::vector<float> &r1 = h[3 * i + 1];
			std::vector<float> &r2 = h[3 * i + 2];
			int row_from = rowsY[3 * i];
			int row_to = rowsY[3 * i + 1];
			int ncols = row_to - row_from;

			float v00 = 0, v01 = 0, v02 = 0, v11 = 0, v12 = 0, v22 = 0;
			for (int j = 0; j < ncols; ++j) {
				v00 += r0[j] * valuesY[j + 0 * ncols + row_from];
				v01 += r0[j] * valuesY[j + 1 * ncols + row_from];
				v02 += r0[j] * valuesY[j + 2 * ncols + row_from];
				v11 += r1[j] * valuesY[j + 1 * ncols + row_from];
				v12 += r1[j] * valuesY[j + 2 * ncols + row_from];
				v22 += r2[j] * valuesY[j + 2 * ncols + row_from];
			}
			cov_pts[i].resize(3, 3);
			cov_pts[i] << 
				v00 + iA[9 * i + 0], v01 + iA[9 * i + 3], v02 + iA[9 * i + 6],
				v01 + iA[9 * i + 1], v11 + iA[9 * i + 4], v12 + iA[9 * i + 7],
				v02 + iA[9 * i + 2], v12 + iA[9 * i + 5], v22 + iA[9 * i + 8];
		}

		free(iA);
	}

	// unscale the covariance matrices for points
	void Algorithm::unscalePointsCovarianceMatrices(SM &S, std::vector<Eigen::Matrix3f> &cov_pts) {
		double *s_ptr = S.valuePtr();
		for (int i = 0; i < cov_pts.size(); ++i) {
			Eigen::Vector3f s;
			s << s_ptr[3 * i], s_ptr[3 * i + 1], s_ptr[3 * i + 2];
			cov_pts[i] = Eigen::Matrix3f(s.asDiagonal() * cov_pts[i] * s.asDiagonal());
		}
	}

}