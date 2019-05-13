// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_data_Point2D.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_data_Point2D.hpp"

namespace usfm {

	bool Point2D::operator < (const Point2D& p) const {
		return (_id_point3D < p._id_point3D);
	}

	std::ostream& operator<< (std::ostream& out, const Point2D& p2D) {
		out << ">>> xy: " << p2D._xy[0] << ", " << p2D._xy[1] << "-> id_p3D: " << p2D._id_point3D << "\n";
		return out;
	};

}