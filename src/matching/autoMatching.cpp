#include "autoMatching.hpp"

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::features;


// Match the features of the two frames and temporary save the results in their respective `_matched_kp` attributes
void autoMatch(Cam* ref, Cam* cam) {

    std::vector<IndMatch> vec_PutativeMatches;

    // Find corresponding points
    matching::DistanceRatioMatch(
            0.8, matching::ANN_L2,
            *(ref->getRegions()),
            *(cam->getRegions()),
            vec_PutativeMatches);

    IndMatchDecorator<float> matchDeduplicator(vec_PutativeMatches, ref->getFeatures(), cam->getFeatures());
    matchDeduplicator.getDeduplicated(vec_PutativeMatches);

    // Get back interest point and send it to the robust estimation framework
    Mat refMatches(2, vec_PutativeMatches.size());
    Mat camMatches(2, vec_PutativeMatches.size());

    for (size_t k = 0; k < vec_PutativeMatches.size(); ++k)  {
        const PointFeature & imaRef = ref->getFeatures()[vec_PutativeMatches[k].i_];
        const PointFeature & imaCam = cam->getFeatures()[vec_PutativeMatches[k].j_];
        refMatches.col(k) = imaRef.coords().cast<double>();
        camMatches.col(k) = imaCam.coords().cast<double>();
    }

    // Save temporary the results in the objects attributes
    ref->setMatchedKp(refMatches);
    cam->setMatchedKp(camMatches);

}
