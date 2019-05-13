// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_projection_Factory.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_PROJECTION_FACTORY_H
#define USFM_PROJECTION_FACTORY_H

#include <string>
#include <memory>
#include "USfM/usfm_data_Camera.hpp"
#include "USfM/usfm_data_Image.hpp"
#include "USfM/usfm_data_Point3D.hpp"
#include "USfM/usfm_Projection.hpp"

namespace usfm {

	class Projection_Factory {
	public:
		static std::shared_ptr<Projection> createProjection(Camera* camera);
		static std::shared_ptr<Projection> createProjection(Image* image);
		static std::shared_ptr<Projection> createProjection(Camera* camera, Image* image, Point3D* X, Point2D* obs);
	};

}

#endif