// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_IO_Colmap.hpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#ifndef USFM_IO_COLMAP_H
#define USFM_IO_COLMAP_H

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include "USfM/usfm_IO.hpp"
#include "USfM/usfm_data_Scene.hpp"
#include "USfM/usfm_Projection_Factory.hpp"

namespace usfm {

	class IO_Colmap : public IO {
	public:
		std::string _scene_path;

		// Read the scene structure (virtual method for all IO objects)
		void read(const std::string& input_dir, Scene& scene);

		// Read TXT files as input
		bool readCameras(const std::string& file_path, Scene& scene);
		bool readImages(const std::string& file_path, Scene& scene);
		bool readPoints3D(const std::string& file_path, Scene& scene);

		// Read database COLMAP
		bool readDatabase(const std::string& file_path, Scene& scene);

		// string to camera model
		static ECameraModel toCameraModel(const std::string& colmap_model);
		static std::string toCameraModel(const ECameraModel& colmap_model);

	};
}

#endif 