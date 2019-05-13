// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_data_Scene.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_data_Scene.hpp"

namespace usfm {

	EPoint2DCovariance str_to_EPoint2DCovariance(const std::string type) {
		if (strcmp(type.c_str(), "UNIT") == 0)
			return eUnit;
		if (strcmp(type.c_str(), "VARIANCE_FACTOR") == 0)
			return eVarianceFactor;
		if (strcmp(type.c_str(),"SQUARED_RESIDUALS") == 0)
			return eSquaredResiduals;
		if (strcmp(type.c_str(),"STRUCTURE_TENSOR") == 0)
			return eStructureTensor;
		throw std::runtime_error(std::string("Unrecognized algorithm for estimation of the covariance of the observations: ") + type);
	}

	Scene::Scene() {
		_settings = AlgSettings();
	}

	// the function assumes that all cameras, images and 3D points were used
	const NumSceneParams Scene::getNumSceneParams() const {
		NumSceneParams nsp;
		for (auto &cam : _cameras) 
			nsp._numCamPar += cam.second.numParams();
		for (auto &img : _images)
			nsp._numImgPar += img.second.numParams();
		nsp._numPtsPar = _points3D.size() * 3;	
		return nsp;
	}

	int Scene::numParams() {
		NumSceneParams nsp = Scene::getNumSceneParams();
		return (nsp._numCamPar + nsp._numImgPar + nsp._numPtsPar);
	}

	void Scene::setInputCovarianceEstimator(const std::string type) {
		_settings._e_in_cov = str_to_EPoint2DCovariance(type);
	}

	std::ostream& operator<< (std::ostream& out, const Scene& s) {
		out << "Scene [cams:" << " - " << s._cameras.size()
			<< ", images:" << s._images.size()
			<< ", points3D:" << s._points3D.size() << "\n";

		// print first cameras
		int i = 0;
		for (auto const &cam : s._cameras) {
			if (i < 10) {
				out << cam.second;
				i++;
			}
			else break;
		}

		// print first images
		i = 0;
		for (auto const &img : s._images) {
			if (i < 10) {
				out << img.second;
				i++;
			}
			else break;
		}

		// print first images
		i = 0;
		for (auto const &pt3D : s._points3D) {
			if (i < 10) {
				out << pt3D.second;
				i++;
			}
			else break;
		}
		return out;
	};

}