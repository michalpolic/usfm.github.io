// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_data_Point3D.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_data_Point3D.hpp"

namespace usfm {

	Point3D::Point3D() {}
	Point3D::Point3D(long id, double* X) : _id(id) {
		_X[0] = X[0];
		_X[1] = X[1];
		_X[2] = X[2];
	}

	std::ostream& operator<< (std::ostream& out, const Point3D& p3D) {
		out << "> Point3D [" << p3D._id << ": " <<
			p3D._X[0] << ", " << p3D._X[1] << ", " << p3D._X[2] << "]\n";
		return out;
	};

}
