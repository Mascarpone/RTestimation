#include "./cameras/Cam.hpp"
#include "./cameras/OmniCam.hpp"
#include "./matching/autoMatching.hpp"
#include "./matching/manualMatching.hpp"
#include "./rtEstimation/rtEstimation.hpp"
#include "./utils/experimentManager.hpp"

using namespace std;


int main(int argc, char const *argv[]) {

    // Initialization
    const string keys =
        "{help h usage ?    |           | print this message                                                                }"
        "{manual m          | false     | match the keypoints manually between the input images                             }"
        "{reference ref r   |           | name of the image to use as the reference for coordinates                         }"
        "{output o          | ./res.txt | name of the output file containing (R,T) in the ref system of coordinates         }"
        "{@experimentPath   |           | path to the experiment folder that must respect a certain format (cf. README.md)  }"
        ;

    cv::CommandLineParser cmdlParser(argc, argv, keys);
    if (cmdlParser.has("help")) {
        cmdlParser.printMessage();
        return EXIT_SUCCESS;
    }

    bool manual = cmdlParser.get<bool>("manual");
    string ref = cmdlParser.get<string>("reference");
    getFileName(ref);
    int refIndex = -1;
    string output = cmdlParser.get<string>("output");
    string experimentPath = cmdlParser.get<string>(0);
    if(experimentPath.empty()) {
        cerr << "Missing experiment path " << endl;
        return EXIT_FAILURE;
    }

    if (!cmdlParser.check()) {
        cmdlParser.printMessage();
        cmdlParser.printErrors();
        exit(EXIT_FAILURE);
    }

    // Loading the experiment
    cout << "Loading the experiment " << endl;
    vector<Cam*> cams;
    OmniCam* omni;
    loadExperiment(experimentPath, cams, omni, manual);


    // For each camera, match the keypoints with the reference and find (R,T)
    cout << "Matching the keypoints " << ((manual) ? "manually" : "using SIFT") << " and estimating (R,T) " << endl;

    vector<cv::Mat> Rs, Ts;

    for (int i = 0; i < cams.size(); i++) {
        // Getting the ref camera index
        if (ref.length() > 0 && cams[i]->getName() == ref) refIndex = i;

        // Matching the keypoints
        // NOTE: the vectors of matched keypoints are temporary saved into the Cam objects

        if (manual) 
            manualMatch(omni, cams[i], experimentPath);
        else 
            autoMatch(omni, cams[i]);

        // Finding (R,T)
        cv::Mat Ri, Ti;
        estimateRT(omni, cams[i], Ri, Ti);
        Rs.push_back(Ri);
        Ts.push_back(Ti);

    }

    cout << endl;

    // Changing coordinates into the system of the ref camera (default = omni)
    if (ref.length() > 0 && refIndex >= 0) {

        cout << "Changing coordinates into the system of the ref camera '" << ref << "' ";

        for (int i = 0; i < Rs.size(); i++) {
            changeCoordRef(Rs[refIndex], Ts[refIndex], Rs[i], Ts[i], Rs[i], Ts[i]);
        }

    }

    // Saving the results
    cout << "Saving the results into " << output << " ";
    saveRTpy(output, Rs, Ts);
    cout << "." << endl;

    return EXIT_SUCCESS;
}
