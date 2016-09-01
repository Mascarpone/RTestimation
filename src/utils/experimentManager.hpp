#ifndef EXPERIMENTMANAGER_HPP
#define EXPERIMENTMANAGER_HPP

#include "../cameras/Cam.hpp"
#include "../cameras/OmniCam.hpp"

// Load a set of images following a specific structure in the file system
// NOTE: see the README.md file to understand the structure of an experiment
void loadExperiment(const std::string& path, std::vector<Cam*>& outCams, OmniCam*& outOmniCam, bool manual=false);

// Save Rotation and Translation matrices pairs in a Python format so the results can be easily plotted
void saveRTpy(const std::string& path, const std::vector<cv::Mat>& Rs, const std::vector<cv::Mat>& Ts);

std::string getID(const Cam* cam1, const Cam* cam2);

#endif
