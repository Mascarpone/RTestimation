#include <cstdlib>
#include <iostream>
#include <fstream>

#include "manualMatching.hpp" 
#include "../utils/experimentManager.hpp"

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::features;
using namespace std;

vector<cv::Point2f> loadKp(const string& dir, const string& id) {
    vector<cv::Point2f> kp;
    cv::FileStorage fs(dir, cv::FileStorage::READ);
    fs[id] >> kp;
    fs.release();
    return kp;
}

void manualMatch(Cam* ref, Cam* cam, string dir){
    vector<cv::Point2f> ref_kp = loadKp(dir, getID(ref, cam)); 
    vector<cv::Point2f> cam_kp = loadKp(dir, getID(cam, ref)); 

    Mat refMatches(2, ref_kp.size());
    Mat camMatches(2, cam_kp.size());
    for (size_t i = 0; i < ref_kp.size(); ++i)  {
        openMVG::features::PointFeature imaRef(ref_kp[i].x, ref_kp[i].y);
        openMVG::features::PointFeature imaCam(cam_kp[i].x, cam_kp[i].y);
        refMatches.col(i) = imaRef.coords().cast<double>();
        camMatches.col(i) = imaCam.coords().cast<double>();
    }

    ref->setMatchedKp(refMatches);
    cam->setMatchedKp(camMatches);
}
