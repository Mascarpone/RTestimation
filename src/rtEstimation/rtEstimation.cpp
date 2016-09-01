#include "rtEstimation.hpp"
#include "sphericalPoseSolver.hpp"

#include "openMVG/multiview/essential.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansac.hpp"
#include "openMVG/multiview/conditioning.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansacKernelAdaptator.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::matching;
using namespace openMVG::robust;

using namespace std;


void estimateRT(Cam* ref, Cam* cam, cv::Mat& outR, cv::Mat& outT) {

    using namespace openMVG::features;

    // Essential geometry filtering of putative matches

    // Convert planar to spherical coordinates
    Mat cam_spherical, ref_spherical;
    cam->toSpherical(cam_spherical);
    ref->toSpherical(ref_spherical);

    // Essential matrix robust estimation from spherical bearing vectors
    vector<size_t> vec_inliers;

    // Use the 8 point solver in order to estimate E
    typedef openMVG::sphericalPoseSolver::EssentialKernel_spherical Kernel;

    // Define the AContrario angular error adaptor
    typedef openMVG::robust::ACKernelAdaptor_AngularRadianError<
    openMVG::sphericalPoseSolver::EightPointRelativePoseSolver,
    openMVG::sphericalPoseSolver::AngularError,
    Mat3>
    KernelType;

    KernelType kernel(ref_spherical, cam_spherical);

    // Robust estimation of the Essential matrix and it's precision
    Mat3 E;
    const double precision = numeric_limits<double>::infinity();
    const pair<double,double> ACRansacOut =
    ACRANSAC(kernel, vec_inliers, 1024, &E, precision, true);
    const double& threshold = ACRansacOut.first;
    const double& NFA = ACRansacOut.second;

    // cout << "DEBUG" << endl;
    // cout << "\n Angular threshold found: " << R2D(threshold) << "(Degree)"<<endl;
    // cout << "\n #Putatives/#inliers : " << ref_spherical.cols() << "/" << vec_inliers.size() << "\n" << endl;

    if (vec_inliers.size() > 80) {
        // If an essential matrix have been found
        // Extract R|t
        //  - From the 4 possible solutions extracted from E keep the best
        //  - (check cheirality of correspondence once triangulation is done)

        // Accumulator to find the best solution
        vector<size_t> f(4, 0);

        vector<Mat3> Es;  // Essential matrix
        vector<Mat3> Rs;  // Rotation matrix
        vector<Vec3> ts;  // Translation matrix

        Es.push_back(E);

        // Recover best rotation and translation from E
        MotionFromEssential(E, &Rs, &ts);

        // Test the 4 solutions
        Mat34 P1;
        P_From_KRt(Mat3::Identity(), Mat3::Identity(), Vec3::Zero(), &P1);
        vector< vector<size_t> > vec_newInliers(4);
        vector< vector<Vec3> > vec_3D(4);

        for (int kk = 0; kk < 4; ++kk) {
            const Mat3& R2 = Rs[kk];
            const Vec3& t2 = ts[kk];
            Mat34 P2;
            P_From_KRt(Mat3::Identity(), R2, t2, &P2);

            // For each inlier, triangulate and check chierality
            for (size_t k = 0; k < vec_inliers.size(); ++k) {
                const Vec3& x1_ = ref_spherical.col(vec_inliers[k]);
                const Vec3& x2_ = cam_spherical.col(vec_inliers[k]);

                //Triangulate
                Vec3 X;
                openMVG::sphericalPoseSolver::TriangulateDLT(P1, x1_, P2, x2_, &X);

                //Check positivity of the depth (sign of the dot product)
                const Vec3 Mc = R2 * X + t2;
                if (x2_.dot(Mc) > 0 && x1_.dot(X) > 0) {
                    ++f[kk];
                    vec_newInliers[kk].push_back(vec_inliers[k]);
                    vec_3D[kk].push_back(X);
                }
            }
        }

        // Find the best solution thanks to the accumulator
        int max = 0, max_i = 0;
        for(int i = 0; i < 4; i++){
            if(f[i] > max){
                max = f[i];
                max_i = i;
            }
        }

        // Return the found rotation and translation matrices with openCV Mat type
        cv::eigen2cv(Rs[max_i], outR);
        cv::eigen2cv(ts[max_i], outT);

    } else {

        // When there are not enough inliers, the user is invited to change his data
        cerr << "Not enough inliers found between " << ref->getName() << " and " << cam->getName() << endl;

    }

}

// Considering a frame 0 that can be located with (R0_X, T0_X) in the system of coordinates of a frame X
// and           frame 1 that can be located with (R1_X, T1_X) in the system of coordinates of a frame X
// calculate the (R1_0, T1_0) pair that locates frame 1 in the system of coordinates of the frame 0
void changeCoordRef(const cv::Mat& R0_X, const cv::Mat& T0_X, const cv::Mat& R1_X, const cv::Mat& T1_X, cv::Mat& outR1_0, cv::Mat& outT1_0) {

    outR1_0 = R1_X * R0_X.t();
    outT1_0 = T1_X - outR1_0 * T0_X;

}
