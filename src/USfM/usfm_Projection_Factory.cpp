// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_projection_Factory.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_data_Camera.hpp"
#include "USfM/usfm_Projection_Factory.hpp"

#include "USfM/usfm_Projection_Simple_Pinhole.hpp"
#include "USfM/usfm_Projection_Pinhole.hpp"
#include "USfM/usfm_Projection_Radial1.hpp"
#include "USfM/usfm_Projection_Radial2.hpp"
#include "USfM/usfm_Projection_Radial3.hpp"
#include "USfM/usfm_Projection_Radial4.hpp"
#include "USfM/usfm_Projection_Division1.hpp"
#include "USfM/usfm_Projection_Division2.hpp"
#include "USfM/usfm_Projection_Division3.hpp"
#include "USfM/usfm_Projection_Division4.hpp"
#include "USfM/usfm_Projection_Radial1Division1.hpp"
#include "USfM/usfm_Projection_Radial2Division2.hpp"
#include "USfM/usfm_Projection_Radial3Division1.hpp"
#include "USfM/usfm_Projection_Radial3Division2.hpp"
#include "USfM/usfm_Projection_Radial3Division3.hpp"
#include "USfM/usfm_Projection_InverseDivision1.hpp"
#include "USfM/usfm_Projection_InverseDivision2.hpp"
#include "USfM/usfm_Projection_OpenCV.hpp"
#include "USfM/usfm_Projection_OpenCV_Fisheye.hpp"
#include "USfM/usfm_Projection_OpenCV_Full.hpp"
#include "USfM/usfm_Projection_Simple_Radial_Fisheye.hpp"
#include "USfM/usfm_Projection_Radial_Fisheye.hpp"


namespace usfm {

	std::shared_ptr<Projection> Projection_Factory::createProjection(Camera* camera) {
		return createProjection(camera, NULL, NULL, NULL);
	}

	std::shared_ptr<Projection> Projection_Factory::createProjection(Image* image) {
		return createProjection(NULL, image, NULL, NULL);
	}

	// creates an projection object
	std::shared_ptr<Projection> Projection_Factory::createProjection(Camera* camera, Image* image, Point3D* X, Point2D* obs) {
		
		// if we have null pointer of input object (e.g. camera, image, ...), 
		// the code assumes plain initialization i.e. the object will not be usefull in any further evaluation  
		ECameraModel camModel = eSimplePinhole;
		EImgModel imgModel = eAAC;
		if (camera != NULL)
			camModel = camera->getModel();
		if (image != NULL)
			imgModel = image->getModel();

		// get projection object
		if (camModel==eSimplePinhole && imgModel==eAAC)
			return std::make_shared<ProjectionSimplePinhole>(camera, image, X, obs);

		if (camModel == ePinhole && imgModel == eAAC)
			return std::make_shared<ProjectionPinhole>(camera, image, X, obs);

		if (camModel == eRadial1 && imgModel == eAAC)
			return std::make_shared<ProjectionRadial1>(camera, image, X, obs);

		if (camModel == eRadial2 && imgModel == eAAC)
			return std::make_shared<ProjectionRadial2>(camera, image, X, obs);

		if (camModel == eRadial3 && imgModel == eAAC)
			return std::make_shared<ProjectionRadial3>(camera, image, X, obs);

		if (camModel == eRadial4 && imgModel == eAAC)
			return std::make_shared<ProjectionRadial4>(camera, image, X, obs);

		if (camModel == eDivision1 && imgModel == eAAC)
			return std::make_shared<ProjectionDivision1>(camera, image, X, obs);

		if (camModel == eDivision2 && imgModel == eAAC)
			return std::make_shared<ProjectionDivision2>(camera, image, X, obs);
		
		if (camModel == eDivision3 && imgModel == eAAC)
			return std::make_shared<ProjectionDivision3>(camera, image, X, obs);

		if (camModel == eDivision4 && imgModel == eAAC)
			return std::make_shared<ProjectionDivision4>(camera, image, X, obs);
    
		if (camModel == eRadial1Division1 && imgModel == eAAC)
			return std::make_shared<ProjectionRadial1Division1>(camera, image, X, obs);

		if (camModel == eRadial2Division2 && imgModel == eAAC)
			return std::make_shared<ProjectionRadial2Division2>(camera, image, X, obs);

		if (camModel == eRadial3Division1 && imgModel == eAAC)
			return std::make_shared<ProjectionRadial3Division1>(camera, image, X, obs);

		if (camModel == eRadial3Division2 && imgModel == eAAC)
			return std::make_shared<ProjectionRadial3Division2>(camera, image, X, obs);

		if (camModel == eRadial3Division3 && imgModel == eAAC)
			return std::make_shared<ProjectionRadial3Division3>(camera, image, X, obs);

		if (camModel == eInverseDivision1 && imgModel == eAAC)
			return std::make_shared<ProjectionInverseDivision1>(camera, image, X, obs);
    
		if (camModel == eInverseDivision2 && imgModel == eAAC)
			return std::make_shared<ProjectionInverseDivision2>(camera, image, X, obs);

		if (camModel == eOpenCv && imgModel == eAAC)
			return std::make_shared<ProjectionOpenCV>(camera, image, X, obs);

		if (camModel == eOpenCvFisheye && imgModel == eAAC)
			return std::make_shared<ProjectionOpenCVFisheye>(camera, image, X, obs);

		if (camModel == eOpenCFull && imgModel == eAAC)
			return std::make_shared<ProjectionOpenCVFull>(camera, image, X, obs);

		if (camModel == eRadial1Fisheye && imgModel == eAAC)
			return std::make_shared<ProjectionSimpleRadialFisheye>(camera, image, X, obs);

		if (camModel == eRadial2Fisheye && imgModel == eAAC)
			return std::make_shared<ProjectionRadialFisheye>(camera, image, X, obs);

		throw std::runtime_error(
			"The projection for selected camera and image model is not implemented (camModel: "
			+ std::to_string(int(camModel)) + ", imgModel: " + std::to_string(imgModel) + ").");
	}

}
