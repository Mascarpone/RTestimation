#ifndef AUTOMATCHING_HPP
#define AUTOMATCHING_HPP

#include "../cameras/Cam.hpp"
#include "../cameras/OmniCam.hpp"

// Match the features of the two frames and temporary save the results in their respective `_matched_kp` attributes
void autoMatch(Cam* ref,  Cam* cam);

#endif
