// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_data_Point3D.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_DATA_POINT3D_H
#define USFM_DATA_POINT3D_H

#include <ostream>

namespace usfm {

	class Point3D {
	public:
		long _id;
		double _X[3];

		Point3D();
		Point3D(long id, double* X);
	};

	std::ostream& operator<< (std::ostream& out, const Point3D& p);
}

#endif