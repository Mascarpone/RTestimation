#ifndef CAM_HPP
#define CAM_HPP

#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "openMVG/image/image.hpp"
#include "openMVG/numeric/numeric.h"
#include "openMVG/features/features.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG_dependencies/nonFree/sift/SIFT_describer.hpp"

std::string getFileName(std::string& s);

class Cam {

	protected:
		std::string _name;												// An image can be identified by its filename
		double _width;													// Width of the frame
		double _height; 												// Height of the frame
		openMVG::features::Regions *_regions;
		std::unique_ptr<openMVG::features::Regions> _regions_perImage;
		openMVG::features::PointFeatures _features;						// Feature points of the frame
		openMVG::Mat *_matched_kp;										// OpenMVG Matrix temporary containing the matched keypoints of this image with another one
        cv::Mat _mat;

	public:
		// Constructors & Destructors
		Cam(std::string path, bool manual=false);
		virtual ~Cam();

		// Convert the matched 2D keypoints associated to this object into 3D spherical coordinates
		virtual void toSpherical(openMVG::Mat& sphericalCoords) const;

		// Getters
        std::string getName() const { return _name; }
        double getWidth() const { return _width; }
        double getHeight() const { return _height; }
        openMVG::features::Regions *getRegions() const { return _regions_perImage.get(); }
        openMVG::features::PointFeatures getFeatures() const { return _features; }
        openMVG::Mat* getMatchedKp() const { return _matched_kp; }
        cv::Mat getMat() const { return _mat; }

		// Setters
        void setMatchedKp(openMVG::Mat &kp) { _matched_kp = new openMVG::Mat(); *_matched_kp = kp; }

};

#endif
