#include "experimentManager.hpp"
#include <dirent.h>
#include "cstdio"

using namespace std;
using namespace cv;


// Load a set of images following a specific structure in the file system
// NOTE: see the README.md file to understand the structure of an experiment
void loadExperiment(const string& path, vector<Cam*>& outCams, OmniCam*& outOmniCam, bool manual) {

    struct dirent* dirent;

    // Load the cameras frames
    DIR* camDir = opendir(string(path + "/cam/").c_str());
    rewinddir(camDir);
    while ((dirent = readdir(camDir)) != NULL) {
        string name = dirent->d_name;
        if (dirent->d_type == DT_REG && name.find(".") != 0) {
            outCams.push_back(new Cam(path + "/cam/" + name, manual));
        }
    }
    closedir(camDir);

    // Load the omnidirectional camera frame
    DIR* omniDir = opendir(string(path + "/omni/").c_str());
    rewinddir(omniDir);
    if ((dirent = readdir(omniDir)) != NULL) {
        string name = dirent->d_name;
        if (dirent->d_type == DT_REG && name.find(".") != 0) {
            outOmniCam = new OmniCam(path + "/omni/" + name, manual);
            cout << outOmniCam->getName() << " has been selected as the reference omni frame" << endl;
        }
    }
    closedir(omniDir);

}

// Save Rotation and Translation matrices pairs in a Python format so the results can be easily plotted
void saveRTpy(const string& path, const vector<Mat>& Rs, const vector<Mat>& Ts) {

    ofstream res;
    res.open(path);

    for (int i = 0; i < Rs.size(); i++) {
        res << format(Rs[i], cv::Formatter::FMT_PYTHON);
        res << "|\n";
        res << format(Ts[i], cv::Formatter::FMT_PYTHON);
        res << "|\n";
    }

    res.close();

}

string getID(const Cam* cam1, const Cam* cam2){
    return "fs_" + cam1->getName() + "_" + cam2->getName();
}
