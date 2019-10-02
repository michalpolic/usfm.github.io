// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

/*
* File:   usfm_IO_Colmap.cpp
* Author: Michal Polic, michal.polic(at)cvut.cz
*/

#include "USfM/usfm_IO_Colmap.hpp"

namespace usfm {


	void IO_Colmap::read(const std::string& input_path, Scene& scene) {
		std::cout << "Read COLMAP reconstruction from: " << input_path << "\n";
		
		// Try to read the data from TXT files
		bool loaded = (
			readCameras(std::string(input_path + "/cameras.txt"), scene) &
			readImages(std::string(input_path + "/images.txt"), scene) &
			readPoints3D(std::string(input_path + "/points3D.txt"), scene));
		

		// Try to read the data from database
		if (!loaded) {
			loaded = readDatabase(input_path, scene);
		}

		// Show the result of loading
		if (loaded)
			std::cout << "Reading ... [done]\n";
		else
			throw std::runtime_error("Error: Loading of the scene on path '" + input_path + "' failed!");
	}


	// Read cameras from txt file 
	bool IO_Colmap::readCameras(const std::string& file_path, Scene& scene) {
		//std::ifstream file;
		//file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		bool camerasLoaded = false;

		try {
			//file.open(file_path, std::ios_base::in);
			std::ifstream file(file_path);

			// read lines
			std::string line = std::string();
			while (!safeGetline(file, line).eof()) {
				if (line.size() > 1 && line.at(0) != '#') {
					std::istringstream line_stream(line);
					//line_stream.exceptions(std::ios::failbit);

					// load parameters to camera
					int cam_id;
					std::string colmap_camera_model;
					line_stream >> cam_id >> colmap_camera_model;
					std::vector<double> params = std::vector<double>{
						std::istream_iterator<double>(line_stream),
						std::istream_iterator<double>()
					};

					// set the model of camera & add camera to scene
					scene._cameras[cam_id] = Camera(cam_id, toCameraModel(colmap_camera_model), params);
				}
			}
			camerasLoaded = true;
			file.close();
		} 
		catch (const std::ifstream::failure& e) {
			std::cerr << "Exception opening/reading file <path_to/cameras.txt>: " << e.what();
		}
		
		return camerasLoaded;
	}


	// Read images from txt file
	bool IO_Colmap::readImages(const std::string& file_path, Scene& scene) {
		std::ifstream file;
		//file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		bool imagesLoaded = false;

		try {
			file.open(file_path, std::ios_base::in);

			// read lines
			std::string line = std::string();
			while (!safeGetline(file, line).eof()) {
				if (line.size() > 1 && line.at(0) != '#') {
					std::istringstream line_stream(line);
					//line_stream.exceptions(std::ios::failbit);

					// read an image - IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
					int id, cam_id;
					double q[4];
					double t[3];
					line_stream >> id >> q[0] >> q[1] >> q[2] >> q[3] >> t[0] >> t[1] >> t[2] >> cam_id;

					// save image to scene & setup the vector of parameters
					scene._images[id] = Image(id, cam_id, q, t);

					// image points 2D - POINTS2D[] as (X, Y, POINT3D_ID)
					line.clear();
					getline(file, line);
					std::istringstream line_stream2(line);
					//line_stream2.exceptions(std::ios::failbit);
					while (line_stream2.good()) {
						Point2D p2d = Point2D();
						if( line_stream2 >> p2d._xy[0] >> p2d._xy[1] >> p2d._id_point3D ){
							if (p2d._id_point3D != -1)		// don't assume the observations without a 3D point
								scene._images[id]._point2D.push_back(p2d);
						}
					}
				}
			}
			imagesLoaded = true;
		}
		catch (const std::ifstream::failure& e) {
			std::cerr << "Exception opening/reading file <path_to/images.txt>: " << e.what();
		}
		file.close();
		return imagesLoaded;
	}


	bool IO_Colmap::readPoints3D(const std::string& file_path, Scene& scene) {
		std::ifstream file;
		//file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		bool pointsLoaded = false;

		try {
			file.open(file_path, std::ios_base::in);

			// read lines
			std::string line = std::string();
			while (!safeGetline(file, line).eof()) {
				if (line.size() > 1 && line.at(0) != '#') {
					Point3D p = Point3D();
					std::istringstream line_stream(line);

					// load one point3D - POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)
					if( line_stream >> p._id >> p._X[0] >> p._X[1] >> p._X[2])
						scene._points3D[p._id] = p;
				}
			}
		pointsLoaded = true;
		}
		catch (const std::ifstream::failure& e) {
			std::cerr << "Exception opening/reading file <path_to/points3D.txt>: " << e.what();
		}
		file.close();
		return pointsLoaded;
	}

	ECameraModel IO_Colmap::toCameraModel(const std::string& colmap_model) {
		if (colmap_model == "SIMPLE_PINHOLE")
			return eSimplePinhole;
		if (colmap_model == "PINHOLE")
			return ePinhole;
		if (colmap_model == "SIMPLE_RADIAL")
			return eRadial1;
		if (colmap_model == "RADIAL")
			return eRadial2;
		if (colmap_model == "RADIAL3")
			return eRadial3;
		if (colmap_model == "RADIAL4")
			return eRadial4;
		if (colmap_model == "DIVISION1")
			return eDivision1;
		if (colmap_model == "DIVISION2")
			return eDivision2;
		if (colmap_model == "DIVISION3")
			return eDivision3;
		if (colmap_model == "DIVISION4")
			return eDivision4;
    if (colmap_model == "I_DIVISION1")
      return eInverseDivision1;
    if (colmap_model == "I_DIVISION2")
      return eInverseDivision2;
		if (colmap_model == "SIMPLE_RADIAL_FISHEYE")
			return eRadial1Fisheye;
		if (colmap_model == "RADIAL_FISHEYE")
			return eRadial2Fisheye;
		if (colmap_model == "OPENCV")
			return eOpenCv;
		if (colmap_model == "OPENCV_FISHEYE")
			return eOpenCvFisheye;
		if (colmap_model == "FULL_OPENCV")
			return eOpenCFull;
		if (colmap_model == "FOV")
			return eFOV;
		if (colmap_model == "THIN_PRISM_FISHEYE")
			return eThinPrismFisheye;
		throw std::runtime_error(std::string("Unrecognized colmap camera model: ") + colmap_model);
	}

	std::string IO_Colmap::toCameraModel(const ECameraModel& colmap_model) {
		if (colmap_model == eSimplePinhole)
			return std::string("SIMPLE_PINHOLE");
		if (colmap_model == ePinhole)
			return std::string("PINHOLE");
		if (colmap_model == eRadial1)
			return std::string("SIMPLE_RADIAL");
		if (colmap_model == eRadial2)
			return std::string("RADIAL");
		if (colmap_model == eRadial3)
			return std::string("RADIAL3");
		if (colmap_model == eRadial4)
			return std::string("RADIAL4");
		if (colmap_model == eDivision1)
			return std::string("DIVISION1");
		if (colmap_model == eDivision2)
			return std::string("DIVISION2");
		if (colmap_model == eDivision3)
			return std::string("DIVISION3");
		if (colmap_model == eDivision4)
			return std::string("DIVISION4");
    if (colmap_model == eInverseDivision1)
      return std::string("I_DIVISION1");
    if (colmap_model == eInverseDivision2)
      return std::string("I_DIVISION2");
		if (colmap_model == eRadial1Fisheye)
			return std::string("SIMPLE_RADIAL_FISHEYE");
		if (colmap_model == eRadial2Fisheye)
			return std::string("RADIAL_FISHEYE");
		if (colmap_model == eOpenCv)
			return std::string("OPENCV");
		if (colmap_model == eOpenCvFisheye)
			return std::string("OPENCV_FISHEYE");
		if (colmap_model == eOpenCFull)
			return std::string("FULL_OPENCV");
		if (colmap_model == eFOV)
			return std::string("FOV");
		if (colmap_model == eThinPrismFisheye)
			return std::string("THIN_PRISM_FISHEYE");
		throw std::runtime_error("Unrecognized colmap camera model.");
	}


	bool IO_Colmap::readDatabase(const std::string& file_path, Scene& scene) {
		std::cerr << "Loading of COLMAP from the database is not implemented yet!\nUse path to directory with .txt files.";
		return false;
	}

}