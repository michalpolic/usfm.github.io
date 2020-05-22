// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_IO.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_IO.hpp"

namespace usfm {

	void IO::writeCov2File(const std::string& filepath, Scene& scene, Statistic& statistic) {

		// cameras
		std::ofstream camera_cov(filepath + std::string("/camera_covariance.txt"));
		camera_cov << "#IDs of images - number of parameters\n" << "#IDs of cameras -number of parameters\n" << "#Covariance matrices\n\n";

		for (const auto& img : scene._images) {
			camera_cov << img.second._id << "-" << img.second.numParams() << " ";
		}
		camera_cov << "\n";

		for (const auto& cam : scene._cameras) {
			camera_cov << cam.second._id << "-" << cam.second.numParams() << " ";
		}
		camera_cov << "\n";

		for (int i = 0; i < scene._iZ.cols(); ++i) {
			for (int j = 0; j <scene._iZ.rows(); ++j)
				camera_cov << scene._iZ.data()[i * scene._iZ.rows() + j] << " ";
			camera_cov << "\n";
		}
		camera_cov.close();

		// points
		std::ofstream pts_cov(filepath + std::string("/points_covariances.txt"));
		for (int i = 0; i < scene._cov_pts.size(); ++i) {
			Eigen::Matrix3f &pt = scene._cov_pts[i];
			for (int j = 0; j < 3; ++j) {
				for (int k = 0; k < 3; ++k)
					pts_cov << pt(k, j) << " ";
			}
			pts_cov << "\n";
		}
		pts_cov.close();


	}

}