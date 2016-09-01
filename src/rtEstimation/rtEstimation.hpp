#ifndef RTESTIMATION_HPP
#define RTESTIMATION_HPP

#include "../cameras/Cam.hpp"
#include "../cameras/OmniCam.hpp"

// Estimate the Rotation and Translation matrices 
// between the ref Cam and the cam Cam in the ref Cam system of coordinates
void estimateRT(Cam* ref, Cam* cam, cv::Mat& outR, cv::Mat& outT);

// Considering a frame 0 that can be located with (R0_X, T0_X) in the system of coordinates of a frame X
// and           frame 1 that can be located with (R1_X, T1_X) in the system of coordinates of a frame X
// calculate the (R1_0, T1_0) pair that locates frame 1 in the system of coordinates of the frame 0
void changeCoordRef(const cv::Mat& R0_X, const cv::Mat& T0_X, const cv::Mat& R1_X, const cv::Mat& T1_X, cv::Mat& outR1_0, cv::Mat& outT1_0);

#endif
