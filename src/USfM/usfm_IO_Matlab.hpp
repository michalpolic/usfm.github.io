// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_IO_Matlab.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_IO_MATLAB_H
#define USFM_IO_MATLAB_H

#ifdef USE_MATLAB
  #include "USfM/usfm_IO.hpp"
  #include "USfM/usfm_data_Scene.hpp"
  #include "USfM/usfm_Projection_Factory.hpp"

  namespace usfm {

	  class IO_Matlab : public IO {
	  public:
		  // Read the scene structure (virtual method for all IO objects)
		  void read(const std::string& input, Scene& scene);
		  void read(const mxArray *prhs[], Scene& scene);

		  // read individual parts
		  bool readCameras(const mxArray *cameras, Scene& scene);
		  bool readImages(const mxArray *images, Scene& scene);
		  bool readPoints3D(const mxArray *points, Scene& scene);
		  bool readSettings(const mxArray *settings, Scene& scene);

		  // write the results into Matlab structures
		  void writeToMatlab(int nlhs, mxArray *plhs[], Scene &scene, Statistic &statistic);
		  void jacobianToMatlab(mxArray *plhs[], Scene &scene);
		  void nullspaceToMatlab(mxArray *plhs[], Scene &scene);
		  void covariancesToMatlab(mxArray *plhs[], Scene &scene, Statistic &statistic);
		  void sceneToMatlab(mxArray *plhs[], Scene &scene, Statistic &statistic);

		  // string to camera model
		  static ECameraModel toCameraModel(const std::string& model);
		  static std::string mxToStr(const mxArray *prhs);
	  };

  }
#endif
#endif 