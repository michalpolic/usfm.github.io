// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_data_Point2D.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_DATA_POINT2D_H
#define USFM_DATA_POINT2D_H

#include <ostream>

namespace usfm {

	class Point2D {
	public:
		int _id_point3D;
		double _xy[2];
		double _xy_cov[4] = { 1,0,0,1 };
		double _xy_std[4] = { 1,0,0,1 };

		bool operator< (const Point2D& p) const;
	};

	std::ostream& operator<< (std::ostream& out, const Point2D& i);
}

#endif