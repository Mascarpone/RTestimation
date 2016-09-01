#ifndef OMNICAM_HPP
#define OMNICAM_HPP

#include "Cam.hpp"


class OmniCam : public Cam {

    public:
        // Constructors & Destructors
        OmniCam(std::string path, bool manual=false) : Cam(path, manual){}
        virtual ~OmniCam();

        // Convert the matched 2D keypoints associated to this object into 3D spherical coordinates
        virtual void toSpherical(openMVG::Mat& sphericalCoords) const;

};

#endif
