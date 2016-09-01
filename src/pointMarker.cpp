#include <cstdlib>
#include <iostream>
#include <fstream>
#include "./cameras/Cam.hpp"
#include "./cameras/OmniCam.hpp"
#include "./utils/experimentManager.hpp"

using namespace std;

Cam *curr0, *curr1;
int cx0, cy0, cx1, cy1;
string dir;

void drawAndSavePoints(Cam* cam0, Cam* cam1, cv::FileStorage& fs_out) {
    curr0 = cam0;
    curr1 = cam1;

    vector<cv::Point2f> kp0, kp1;
    cv::FileStorage fs_in(dir + "/keypoints.yml", cv::FileStorage::READ);
    fs_in[getID(cam0, cam1)] >> kp0; 
    fs_in[getID(cam1, cam0)] >> kp1; 
    fs_in.release();

    char key = '0';
    while(key != 'n' && key != 'N') {
        key = cv::waitKey(10);
        cv::Mat res0, res1;
        cam0->getMat().copyTo(res0);
        cam1->getMat().copyTo(res1);
        for(int i = 0; i < kp0.size(); i++) {
            int d = 7;
            cv::Size text = getTextSize(to_string(i), cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, 0);
            circle(res0, cv::Point(kp0[i].x, kp0[i].y), d, cv::Scalar(0, 255, 0), CV_FILLED);
            circle(res1, cv::Point(kp1[i].x, kp1[i].y), d, cv::Scalar(0, 255, 0), CV_FILLED);
            putText(res0, to_string(i), cv::Point(kp0[i].x-text.width/2, kp0[i].y+text.height/2), cv::FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(0, 0, 0), 1, CV_AA);
            putText(res1, to_string(i), cv::Point(kp1[i].x-text.width/2, kp1[i].y+text.height/2), cv::FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(0, 0, 0), 1, CV_AA);
        }
        if(cx0 != -1 && cy0 != -1)
            circle(res0, cv::Point(cx0, cy0), 8, cv::Scalar(0, 0, 255), CV_FILLED);
        if(cx1 != -1 && cy1 != -1)
            circle(res1, cv::Point(cx1, cy1), 8, cv::Scalar(0, 0, 255), CV_FILLED);

        cv::imshow("Image 1", res0);
        cv::imshow("Image 2", res1);
        if(key == 'a'){
            if(cx0 != -1 && cy0 != -1 && cx1 != -1 && cy1 != -1) {
                kp0.push_back(cv::Point2f(cx0, cy0));
                kp1.push_back(cv::Point2f(cx1, cy1));
            }
            cx0 = cy0 = cx1 = cy1 = -1;
        }

        if(key == 'd'){
            if(kp0.size()){
                kp0.pop_back();
                kp1.pop_back();
            }
        }
    }


    cout << "Saving " << kp0.size() << " points to " << getID(cam0, cam1) << endl; 
    fs_out << getID(cam0, cam1) << kp0; 
    cout << "Saving " << kp1.size() << " points to " << getID(cam1, cam0) << endl; 
    fs_out << getID(cam1, cam0) << kp1; 

}


void onMouse(int event, int x, int y, int flags, void *param) {
    if(event == CV_EVENT_LBUTTONUP) {
        if(param){
            cx1 = x;
            cy1 = y;
        } else {
            cx0 = x;
            cy0 = y;
        }
    }
}

int main(int argc, char const *argv[]) {
    cx0 = cy0 = cx1 = cy1 = -1;

    if (argc < 2) {
        cout << "usage: " << argv[0] << " <image_directory>" << endl;
        exit(EXIT_FAILURE);
    }

    dir = argv[1];

    vector<Cam*> cams;
    OmniCam* omni;
    loadExperiment(dir, cams, omni, true);

    cv::namedWindow("Image 1", 0);
    cv::resizeWindow("Image 1", 800, 800);
    cv::setMouseCallback("Image 1", onMouse, 0);
    cv::moveWindow("Image 1", 0, 0);

    cv::namedWindow("Image 2", 0);
    cv::resizeWindow("Image 2", 800, 800);
    cv::setMouseCallback("Image 2", onMouse, (void *) 1);
    cv::moveWindow("Image 2", 800, 0);

    cv::FileStorage fs_out(dir + "/tmp_keypoints.yml", cv::FileStorage::WRITE);
    for (int i = 0; i < cams.size(); i++) {
        drawAndSavePoints(cams[i], omni, fs_out);
    }
    fs_out.release();

    ifstream  src(dir + "/tmp_keypoints.yml", ios::binary);
    ofstream  dst(dir + "/keypoints.yml",   ios::binary);
    dst << src.rdbuf();
    remove(string(dir + "/tmp_keypoints.yml").c_str());

    return EXIT_SUCCESS;
}
