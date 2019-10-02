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

	std::istream& IO::safeGetline(std::istream& is, std::string& t)
	{
		t.clear();
		std::istream::sentry se(is, true);
		std::streambuf* sb = is.rdbuf();
		for (;;) {
			int c = sb->sbumpc();
			switch (c) {
			case '\n':
				return is;
			case '\r':
				if (sb->sgetc() == '\n')
					sb->sbumpc();
				return is;
			case std::streambuf::traits_type::eof():
				// Also handle the case when the last line has no line ending
				if (t.empty())
					is.setstate(std::ios::eofbit);
				return is;
			default:
				t += (char)c;
			}
		}
	}

	void IO::writeCov2File(const std::string& filepath, Scene& scene, Statistic& statistic) {
		// cameras
		std::ofstream camera_cov(filepath + std::string("/camera_covariance.txt"));
		for (int i = 0; i < scene._iZ.cols(); ++i) {
			for (int j = 0; j < scene._iZ.rows(); ++j)
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


		// options, times etc. 
		// TODO ...
	}

}