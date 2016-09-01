#include "OmniCam.hpp"

using namespace openMVG;
using namespace std;

OmniCam::~OmniCam() {
}

// Convert the matched 2D keypoints associated to this object into 3D spherical coordinates
void OmniCam::toSpherical(Mat& sphericalCoords) const {
	sphericalCoords.resize(3, _matched_kp->cols());
	for (size_t iCol = 0; iCol < _matched_kp->cols(); ++iCol) {
		// The spherical coordinates of a point on a panoramic frame
		// can be calculated considering the width to be between 0 and 2*Pi
		// and the height to be between 0 and Pi
		const Vec2 & xy = _matched_kp->col(iCol);
		double uval = xy(0) / _width;
		double vval = xy(1) / _height;

		sphericalCoords.col(iCol) =
			(Vec3 (sin(vval*M_PI)*cos(M_PI*(2.0*uval+0.5)),
				   cos(vval*M_PI),
				   sin(vval*M_PI)*sin(M_PI*(2.0*uval+0.5)))).normalized();
	}
}
