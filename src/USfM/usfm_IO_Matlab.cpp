// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_IO_Colmap.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifdef USE_MATLAB
  #include "USfM/usfm_IO_Matlab.hpp"
  #include "USfM/usfm_IO_Colmap.hpp"
  #include "USfM/usfm_Algorithm_Factory.hpp"

  namespace usfm {

	  // called form Matlab - load colmap txt files  
	  void IO_Matlab::read(const std::string& input, Scene& scene) {
		  throw std::runtime_error("Error: Loading of txt files from Matlab is not implemented.");
	  }

	  bool IO_Matlab::readSettings(const mxArray *settings, Scene& scene) {
		  try {
			  const std::string alg = mxToStr(mxGetField(settings, 0, std::string("alg").c_str()));
			  const std::string cov_model = mxToStr(mxGetField(settings, 0, std::string("in_cov").c_str()));
			  const double *run_opt = mxGetPr(mxGetField(settings, 0, std::string("run_opt").c_str()));
			  mxArray *opt_radial = mxGetField(settings, 0, std::string("run_opt_radial").c_str());
			  mxArray *opt_robust_lost = mxGetField(settings, 0, std::string("robust_lost").c_str());
			  mxArray *opt_nullspace_computation = mxGetField(settings, 0, std::string("return_nullspace").c_str());

			  scene._settings._alg = EAlgorithm_stringToEnum(alg);
			  scene.setInputCovarianceEstimator(cov_model);
			  scene._settings._run_opt = (run_opt[0] == 0 ? false : true);
			
			  if (opt_radial != NULL) {			// find the radial distortion parameters and fix the rest
				  const double *run_opt_radial = mxGetPr(opt_radial);
				  scene._settings._run_opt_radial = (run_opt_radial[0] == 0 ? false : true);
			  }
			  if (opt_robust_lost != NULL) {		// process the optimization with robust lost function
				  const double *run_robust_lost = mxGetPr(opt_robust_lost);
				  scene._settings._robust_lost = (run_robust_lost[0] == 0 ? false : true);
			  }

        if (opt_nullspace_computation != NULL) {		// return nullspace as additional output
				const double *run_nullspace_computation = mxGetPr(opt_nullspace_computation);
          scene._settings._return_nullspace = (run_nullspace_computation[0] == 0 ? false : true);
        }
			
		  }
		  catch (const std::runtime_error& e) {
			  std::cerr << "Faild to load settings from Matlab: " << e.what();
		  }
		  return true;
	  }

	  // rewrite the Matlab structures into the scene
	  // assume the same Matlab structures from COLMAP: i.e. cameras, images, points3D as cell arrays
	  void IO_Matlab::read(const mxArray *prhs[], Scene& scene) {
		  if( !readSettings(prhs[0], scene) || !readCameras(prhs[1], scene) || 
			  !readImages(prhs[2], scene) || !readPoints3D(prhs[3], scene) )
			  throw std::runtime_error("Error: Loading failed!");
	  }

	  // read camera cells into scene class
	  bool IO_Matlab::readCameras(const mxArray *cameras, Scene& scene) {
		  try {
			  const mwSize *dims = mxGetDimensions(cameras);
			  mwIndex ncams = dims[1];
			  for (mwIndex i = 0; i < ncams; ++i) {
				  mxArray *camera_cell = mxGetCell(cameras, i);					// one camera
				  double *camera_id = mxGetPr(mxGetField(camera_cell, 0, std::string("camera_id").c_str()));
				  std::string model = mxToStr(mxGetField(camera_cell, 0, std::string("model").c_str()));
				  double *width = mxGetPr(mxGetField(camera_cell, 0, std::string("width").c_str()));
				  double *height = mxGetPr(mxGetField(camera_cell, 0, std::string("height").c_str()));
				  double *params = mxGetPr(mxGetField(camera_cell, 0, std::string("params").c_str()));
				  const mwSize *nparams = mxGetDimensions(mxGetField(camera_cell, 0, std::string("params").c_str()));

				  // save camera to scene
				  int cam_id = (int)*camera_id;
				  std::vector<double> vec_params;
				  vec_params.resize(nparams[0] + 2);		// the standard COLMAP format start with width and height
				  vec_params[0] = *width;
				  vec_params[1] = *height;
				  for (int j = 0; j < nparams[0]; ++j)
					  vec_params[j+2] = params[j];
				  scene._cameras[cam_id] = Camera(cam_id, toCameraModel(model), vec_params);
			  }
		  }
		  catch (const std::runtime_error& e) {
			  std::cerr << "Faild to load cameras from Matlab: " << e.what();
		  }
		  return true;
	  }

	  // read image cells into scene class
	  bool IO_Matlab::readImages(const mxArray *images, Scene& scene) {
		  try {
			  const mwSize *dims = mxGetDimensions(images);
			  mwIndex nimgs = dims[1];
			  for (mwIndex i = 0; i < nimgs; ++i) {
				  mxArray *image_cell = mxGetCell(images, i);					// one image
				  double *image_id = mxGetPr(mxGetField(image_cell, 0, std::string("image_id").c_str()));
				  double *camera_id = mxGetPr(mxGetField(image_cell, 0, std::string("camera_id").c_str()));
				  double *R = mxGetPr(mxGetField(image_cell, 0, std::string("R").c_str()));
				  double q[4];
				  if (mxGetField(image_cell, 0, std::string("q").c_str()) != NULL){
					  double *quat = mxGetPr(mxGetField(image_cell, 0, std::string("q").c_str()));
					  for (int j = 0; j < 4; ++j)
						  q[j] = quat[j];
				  }
				  else {
					  Image::R2q(R, q);
				  }
				  double *t = mxGetPr(mxGetField(image_cell, 0, std::string("t").c_str()));
				  double *xys = mxGetPr(mxGetField(image_cell, 0, std::string("xys").c_str()));
				  double *point3D_ids = mxGetPr(mxGetField(image_cell, 0, std::string("point3D_ids").c_str()));
				  const mwSize *npts = mxGetDimensions(mxGetField(image_cell, 0, std::string("point3D_ids").c_str()));

				  // covariances for observations
				  double *xys_cov = NULL;
				  mxArray *xys_cov_mx = mxGetField(image_cell, 0, std::string("xys_cov").c_str());
			      if (xys_cov_mx != NULL) {
					  const mwSize *nxys_cov = mxGetDimensions(xys_cov_mx);
					  if (nxys_cov[0] == npts[0])
						  xys_cov = mxGetPr(xys_cov_mx);
					  else
						  throw std::runtime_error("The size of the observations covariances does not match the number of observations.");
				  }

				  // covariance elipsoids, standard deviations for observations
				  double *xys_std = NULL;
				  mxArray *xys_std_mx = mxGetField(image_cell, 0, std::string("xys_std").c_str());
				  if (xys_std_mx != NULL) {
					  const mwSize *nxys_std = mxGetDimensions(xys_std_mx);
					  if (nxys_std[0] == npts[0])
						  xys_std = mxGetPr(xys_std_mx);
					  else
						  throw std::runtime_error("The size of the observations elipsoids does not match the number of observations.");
				  }

				  // save image to the scene
				  int img_id = (int)*image_id;
				  int cam_id = (int)*camera_id;
				  scene._images[img_id] = Image(img_id, cam_id, q, t);

				  // image points 2D - POINTS2D[] as (X, Y, POINT3D_ID)
				  for (int j = 0; j < npts[0]; ++j) {
					  Point2D p2d = Point2D();
					  p2d._xy[0] = xys[j];
					  p2d._xy[1] = xys[npts[0] + j];
					  p2d._id_point3D = (int)point3D_ids[j];
					  if (xys_cov != NULL){			// add covariance of input obsrvations if known
						  for (int k = 0; k < 4; ++k)
							  p2d._xy_cov[k] = xys_cov[k*npts[0] + j];
					  }
					  if (p2d._id_point3D != -1)		// don't assume the observations without a 3D point  (TODO: assume them and filter later)
						  scene._images[img_id]._point2D.push_back(p2d);
				  }
			  }
		  }
		  catch (const std::runtime_error& e) {
			  std::cerr << "Faild to load images from Matlab: " << e.what();
		  }
		  return true;
	  }

	  // read point cells into scene class
	  bool IO_Matlab::readPoints3D(const mxArray *points, Scene& scene) {
		  try {
			  const mwSize *dims = mxGetDimensions(points);
			  mwIndex npts = dims[1];
			  for (mwIndex i = 0; i < npts; ++i) {
				  mxArray *point_cell = mxGetCell(points, i);					// one point
				  double *point3D_id = mxGetPr(mxGetField(point_cell, 0, std::string("point3D_id").c_str()));
				  double *xyz = mxGetPr(mxGetField(point_cell, 0, std::string("xyz").c_str()));

				  // save point to the scene
				  Point3D p = Point3D();
				  p._id = (long)*point3D_id;
				  p._X[0] = xyz[0];
				  p._X[1] = xyz[1];
				  p._X[2] = xyz[2];
				  scene._points3D[p._id] = p;
			  }
		  }
		  catch (const std::runtime_error& e) {
			  std::cerr << "Faild to load images from Matlab: " << e.what();
		  }
		  return true;
	  }

	  // write the results to the Matlab structures: jacobian J, nullspace H, covariances - cam, img, pts parameters
	  void IO_Matlab::writeToMatlab(int nlhs, mxArray *plhs[], Scene &scene, Statistic &statistic) {
		  if (nlhs > 0)
			  covariancesToMatlab(plhs, scene, statistic);
		  if (nlhs > 1)
			  sceneToMatlab(plhs, scene, statistic);
		  if (nlhs > 2)
			  jacobianToMatlab(plhs, scene);
		  if (nlhs > 3)
			  nullspaceToMatlab(plhs, scene);
	  }

	  // write the covariances into plhs 
	  void IO_Matlab::covariancesToMatlab(mxArray *plhs[], Scene &scene, Statistic &statistic) {
		  // test iZ
		  if (scene._iZ.cols() == 0 || scene._iZ.rows() == 0) {
			  plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
			  return;
		  }

		  // create structure
		  const char *field_names[] = {"cameras", "images", "points" };				/* "iZ", */
		  mwSize dims[2] = { 1, 1 };
		  mxArray *structure_array = mxCreateStructArray(2, dims, 3, field_names);	/* (2, dims, 4, field_names) */
		
		  // save covariance matric iZ of all camera parameters (i.e., internal and external) to the output structure
		  DM &iZ = scene._iZ;

		
		  /*mxArray *mx_vals = mxCreateDoubleMatrix(iZ.rows(), iZ.cols(), mxREAL);
		  double *vals = mxGetPr(mx_vals);
		  for (int i = 0; i < (iZ.rows() * iZ.cols()); ++i)
			  vals[i] = iZ.data()[i];
		  mxSetFieldByNumber(structure_array, 0, 0, mx_vals);*/
		
		  // The order of parameters in the iZ matrix is: images, cameras (i.e., external, internal camera parameters)
		  // save covariances of images 
		  int i = 0;
		  int matrix_offset = 0;
		  mwSize img_dims[2] = { 1, scene._images.size() };
		  mxArray *img_cell_array = mxCreateCellArray(2, img_dims);
		  for (auto &img : scene._images) {
			  std::shared_ptr<Projection> proj = Projection_Factory::createProjection(NULL, &img.second, NULL, NULL);
			  int num_img_params = proj->numImgParams();
			  DM img_cov_eig = iZ.block(matrix_offset, matrix_offset, num_img_params, num_img_params);
			  matrix_offset += num_img_params;

			  mxArray *img_cov_array = mxCreateDoubleMatrix(num_img_params, num_img_params, mxREAL);
			  double *img_cov = mxGetPr(img_cov_array);
			  for (int j = 0; j < num_img_params * num_img_params; ++j)
				  img_cov[j] = img_cov_eig.data()[j];
			  mxSetCell(img_cell_array, i++, img_cov_array);
		  }
		  mxSetFieldByNumber(structure_array, 0, 1, img_cell_array);

		  // save covariances of cameras
		  i = 0; 
		  mwSize cam_dims[2] = { 1, scene._cameras.size() };
		  mxArray *cam_cell_array = mxCreateCellArray(2, cam_dims);
		  for (auto &cam : scene._cameras) {
			  std::shared_ptr<Projection> proj = Projection_Factory::createProjection(&cam.second, NULL, NULL, NULL);
			  int num_cam_params = proj->numCamParams();
			  DM camera_cov_eig = iZ.block(matrix_offset, matrix_offset, num_cam_params, num_cam_params);
			  matrix_offset += num_cam_params;

			  mxArray *camera_cov_array = mxCreateDoubleMatrix(num_cam_params, num_cam_params, mxREAL);
			  double *camera_cov = mxGetPr(camera_cov_array);
			  for (int j = 0; j < num_cam_params * num_cam_params; ++j)
				  camera_cov[j] = camera_cov_eig.data()[j];
			  mxSetCell(cam_cell_array, i++, camera_cov_array);
		  }
		  mxSetFieldByNumber(structure_array, 0, 0, cam_cell_array);
		
		  // save covariances of points
		  mwSize pt_dims[2] = { 1, scene._cov_pts.size() };
		  mxArray *pt_cell_array = mxCreateCellArray(2, pt_dims);
		  for (int k = 0; k < scene._cov_pts.size(); ++k) {
			  mxArray *pt_cov_array = mxCreateDoubleMatrix(3, 3, mxREAL);			// point covariance is 3x3 matrix, i.e. 9 values
			  double *pt_cov = mxGetPr(pt_cov_array);
			  for (int j = 0; j < 9; ++j)		
				  pt_cov[j] = (double)(scene._cov_pts[k].data()[j]);
			  mxSetCell(pt_cell_array, k, pt_cov_array);
		  }
		  mxSetFieldByNumber(structure_array, 0, 2, pt_cell_array);

		  // set the structure to the output
		  plhs[0] = structure_array;
	  }


	  // write the scene parameters after BA into plhs 
	  void IO_Matlab::sceneToMatlab(mxArray *plhs[], Scene &scene, Statistic &statistic) {
		  // scene 
		  const char *field_names[] = { "cameras", "images", "points3D" };
		  mwSize dims[2] = { 1, 1 };
		  mxArray *scene_structure = mxCreateStructArray(2, dims, 3, field_names);

		  /* Cameras, e.g.:
		  camera_id: 1
          model: 'SIMPLE_PINHOLE'
          width: 1920
		  height: 1080
		  params: [3×1 double]*/
		  mwSize cam_dims[2] = { 1, scene._cameras.size() };
		  mxArray *cam_cell_array = mxCreateCellArray(2, cam_dims);
		  int i = 0;
		  for (auto &cam : scene._cameras) {
			  std::shared_ptr<Projection> proj = Projection_Factory::createProjection(&cam.second);
			  const char *cam_field_names[] = { "camera_id", "model", "width", "height", "params" };
			  mxArray *camera_structure = mxCreateStructArray(2, dims, 5, cam_field_names);
			
			  mxSetFieldByNumber(camera_structure, 0, 0, mxCreateDoubleScalar((double)cam.second._id));		// camera_id
			  mxArray *cam_model = mxCreateString(IO_Colmap::toCameraModel(cam.second.getModel()).c_str());	// model
			  mxSetFieldByNumber(camera_structure, 0, 1, cam_model);											// model
			  mxSetFieldByNumber(camera_structure, 0, 2, mxCreateDoubleScalar(cam.second.value(e_img_width)));// width
			  mxSetFieldByNumber(camera_structure, 0, 3, mxCreateDoubleScalar(cam.second.value(e_img_height)));// height

			  mxArray *mx_cam_params = mxCreateDoubleMatrix(proj->numCamParams(), 1, mxREAL);					// params
			  double *cam_params = mxGetPr(mx_cam_params);
			  for (int j = 0; j < proj->numCamParams(); ++j)
				  cam_params[j] = cam.second._parameters.data()[j];
			  mxSetFieldByNumber(camera_structure, 0, 4, mx_cam_params);

			  mxSetCell(cam_cell_array, i++, camera_structure);
		  }
		  mxSetFieldByNumber(scene_structure, 0, 0, cam_cell_array);

		  /* Images, e.g.:
		  image_id: 1
		  q: [0.8565 -0.0508 -0.4693 -0.2087]
          R: [3×3 double]
          t: [3×1 double]
          camera_id: 1
          name: '↵'
          xys: [10×2 double]
          point3D_ids: [10×1 double]*/
		  mwSize img_dims[2] = { 1, scene._images.size() };
		  mxArray *imgs_cell_array = mxCreateCellArray(2, img_dims);
		  i = 0;
		  for (auto &img : scene._images) {
			  std::shared_ptr<Projection> proj = Projection_Factory::createProjection(&img.second);
			  const char *img_field_names[] = { "image_id", "q", "R", "t", "camera_id", "name", "xys", "xys_cov", "xys_std", "point3D_ids" };
			  mxArray *image_structure = mxCreateStructArray(2, dims, 10, img_field_names);

			  mxSetFieldByNumber(image_structure, 0, 0, mxCreateDoubleScalar((double)img.second._id));		// image_id
			
			  mxArray *mx_q = mxCreateDoubleMatrix(1, 4, mxREAL);												// q
			  double *q = mxGetPr(mx_q);
			  Image::aa2q(img.second.value(e_aa), q);
			  mxSetFieldByNumber(image_structure, 0, 1, mx_q);
			
			  mxArray *mx_R = mxCreateDoubleMatrix(3, 3, mxREAL);												// R
			  double *R = mxGetPr(mx_R);
			  double Rt[9]; 
			  Image::q2R(q, Rt);
			  R[0] = Rt[0]; R[1] = Rt[3]; R[2] = Rt[6]; R[3] = Rt[1]; 
			  R[4] = Rt[4]; R[5] = Rt[7]; R[6] = Rt[2]; R[7] = Rt[5]; R[8] = Rt[8];
			  mxSetFieldByNumber(image_structure, 0, 2, mx_R);

			  double *C = img.second.value(e_C);																// t
			  mxArray *mx_t = mxCreateDoubleMatrix(3, 1, mxREAL);
			  double *t = mxGetPr(mx_t);
			  t[0] = -C[0] * R[0] - C[1] * R[3] - C[2] * R[6];
			  t[1] = -C[0] * R[1] - C[1] * R[4] - C[2] * R[7];
			  t[2] = -C[0] * R[2] - C[1] * R[5] - C[2] * R[8];
			  mxSetFieldByNumber(image_structure, 0, 3, mx_t);

			  mxSetFieldByNumber(image_structure, 0, 4, mxCreateDoubleScalar(img.second._cam_id));			// camera_id
			  mxArray *img_name = mxCreateString("");															// name
			  mxSetFieldByNumber(image_structure, 0, 5, img_name);

			  int N = img.second._point2D.size();																// xys
			  mxArray *mx_xys = mxCreateDoubleMatrix(N, 2, mxREAL);											
			  double *xys = mxGetPr(mx_xys);
			  for (int j = 0; j < N; ++j) {
				  xys[j] = img.second._point2D[j]._xy[0];
				  xys[j + N] = img.second._point2D[j]._xy[1];
			  }
			  mxSetFieldByNumber(image_structure, 0, 6, mx_xys);
			
			  mxArray *mx_xys_cov = mxCreateDoubleMatrix(N, 4, mxREAL);
			  double *xys_cov = mxGetPr(mx_xys_cov);
			  for (int j = 0; j < N; ++j) {
				  xys_cov[j] = img.second._point2D[j]._xy_cov[0];
				  xys_cov[j + N] = img.second._point2D[j]._xy_cov[1];
				  xys_cov[j + 2*N] = img.second._point2D[j]._xy_cov[2];
				  xys_cov[j + 3*N] = img.second._point2D[j]._xy_cov[3];
			  }
			  mxSetFieldByNumber(image_structure, 0, 7, mx_xys_cov);

			  mxArray *mx_xys_std = mxCreateDoubleMatrix(N, 4, mxREAL);
			  double *xys_std = mxGetPr(mx_xys_std);
			  for (int j = 0; j < N; ++j) {
				  xys_std[j] = img.second._point2D[j]._xy_std[0];
				  xys_std[j + N] = img.second._point2D[j]._xy_std[1];
				  xys_std[j + 2 * N] = img.second._point2D[j]._xy_std[2];
				  xys_std[j + 3 * N] = img.second._point2D[j]._xy_std[3];
			  }
			  mxSetFieldByNumber(image_structure, 0, 8, mx_xys_std);

			  mxArray *mx_point3D_ids = mxCreateDoubleMatrix(N, 1, mxREAL);									// point3D_ids
			  double *point3D_ids = mxGetPr(mx_point3D_ids);
			  for (int j = 0; j < N; ++j)
				  point3D_ids[j] = img.second._point2D[j]._id_point3D;
			  mxSetFieldByNumber(image_structure, 0, 9, mx_point3D_ids);
			  mxSetCell(imgs_cell_array, i++, image_structure);
		  }
		  mxSetFieldByNumber(scene_structure, 0, 1, imgs_cell_array);


		  /* Point in 3D, e.g.:
		  point3D_id: 1
          xyz: [3×1 double]
          rgb: [3×1 int8]
          error: -1
          track: [0×2 int64]*/
		  mwSize pts_dims[2] = { 1, scene._points3D.size() };
		  mxArray *pts_cell_array = mxCreateCellArray(2, pts_dims);
		  i = 0;
		  for (auto &pt : scene._points3D) {
			  const char *pt_field_names[] = { "point3D_id", "xyz", "rgb", "error", "track" };
			  mxArray *pt_structure = mxCreateStructArray(2, dims, 5, pt_field_names);
			
			  mxSetFieldByNumber(pt_structure, 0, 0, mxCreateDoubleScalar((double)pt.second._id));	// point3D_id

			  mxArray *mx_xyz = mxCreateDoubleMatrix(3, 1, mxREAL);									// xyz
			  double *xyz = mxGetPr(mx_xyz);
			  for (int j = 0; j < 3; ++j)
				  xyz[j] = pt.second._X[j];
			  mxSetFieldByNumber(pt_structure, 0, 1, mx_xyz);

			  mxArray *mx_rgb = mxCreateDoubleMatrix(3, 1, mxREAL);									// rgb
			  double *rgb = mxGetPr(mx_rgb);
			  for (int j = 0; j < 3; ++j)
				  rgb[j] = 0.0;
			  mxSetFieldByNumber(pt_structure, 0, 2, mx_rgb);

			  mxSetFieldByNumber(pt_structure, 0, 3, mxCreateDoubleScalar(-1.0));						// error

			  mxArray *mx_track = mxCreateDoubleMatrix(0, 2, mxREAL);									// track
			  mxSetFieldByNumber(pt_structure, 0, 4, mx_track);
			  mxSetCell(pts_cell_array, i++, pt_structure);
		  }
		  mxSetFieldByNumber(scene_structure, 0, 2, pts_cell_array);

		  // scene
		  plhs[1] = scene_structure;
	  }

	  // write the Jacobian into plhs 
	  void IO_Matlab::jacobianToMatlab(mxArray *plhs[], Scene &scene) {
		  // create Jacobian structure
		  SM_row &J = scene._jacobian;
		  const char *field_names[] = { "num_rows", "num_columns", "rows", "columns", "values" };
		  mwSize dims[2] = { 1, 1 };
		  mxArray *structure_array = mxCreateStructArray(2, dims, 5, field_names);  

		  // write scalars: num_rows, num_columns
		  mxSetFieldByNumber(structure_array, 0, 0, mxCreateDoubleScalar(J.rows()));
		  mxSetFieldByNumber(structure_array, 0, 1, mxCreateDoubleScalar(J.cols()));

		  // write arrays: rows, columns, values
		  mxArray *mx_rows = mxCreateDoubleMatrix(1, J.nonZeros(), mxREAL);
		  mxArray *mx_cols = mxCreateDoubleMatrix(1, J.nonZeros(), mxREAL);
		  mxArray *mx_vals = mxCreateDoubleMatrix(1, J.nonZeros(), mxREAL);
		  double *rows = mxGetPr(mx_rows);
		  double *cols = mxGetPr(mx_cols);
		  double *vals = mxGetPr(mx_vals);
		  int i = 0;
		  for (int k = 0; k < J.outerSize(); ++k){
			  for (SM_row::InnerIterator it(J, k); it; ++it) {
				  rows[i] = it.row() + 1;		// Matlab indexes are from 1
				  cols[i] = it.col() + 1; 
				  vals[i++] = it.value();
			  }
		  }
		  mxSetFieldByNumber(structure_array, 0, 2, mx_rows);
		  mxSetFieldByNumber(structure_array, 0, 3, mx_cols);
		  mxSetFieldByNumber(structure_array, 0, 4, mx_vals);
		  plhs[2] = structure_array;
	  }

	  // write the nullspace into plhs 
	  void IO_Matlab::nullspaceToMatlab(mxArray *plhs[], Scene &scene) {
		  DM &H = scene._H;
		  mxArray *mx_vals = mxCreateDoubleMatrix(H.rows(), H.cols(), mxREAL);
		  double *vals = mxGetPr(mx_vals);
		  for (int i = 0; i < (H.rows()*H.cols()); ++i) 
			  vals[i] = H.data()[i];
		  plhs[3] = mx_vals;
	  }


	  // the camera model to enum - use the same models in Matlab as in COLMAP
	  ECameraModel IO_Matlab::toCameraModel(const std::string& model) {
		  return IO_Colmap::toCameraModel(model);
	  }

	  // convert mxArray into the string
	  std::string IO_Matlab::mxToStr(const mxArray *prhs) {
		  if (!mxIsChar(prhs))
			  mexErrMsgIdAndTxt("MATLAB:mxmalloc:invalidInput", "Input argument must be a string.");
		  size_t buflen = mxGetN(prhs) + 1;
		  char *buf = (char*)mxMalloc(buflen);
		  mxGetString(prhs, buf, (mwSize)buflen);
		  std::string s = std::string(buf);
		  mxFree(buf);
		  return s;
	  }
  }
#endif