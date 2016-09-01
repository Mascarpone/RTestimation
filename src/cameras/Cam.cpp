#include "Cam.hpp"

using namespace openMVG;
using namespace std;

// Parse a path into only a file name
string getFileName(string& s) {
    replace(s.begin(), s.end(), '.', '_');
    char sep = '/';
#ifdef _WIN32
    sep = '\\';
#endif
    size_t i = s.rfind(sep, s.length());
    if (i != string::npos)
        return(s.substr(i+1, s.length() - i));
    return("");
}

// Cam constructor
Cam::Cam(string path, bool manual) {
    using namespace openMVG::image;
    using namespace openMVG::features;

    // Initialize the default image characteristics
    if(!manual){
        Image<unsigned char> image;
        ReadImage(path.c_str(), &image);
        _width = image.Width();
        _height = image.Height();

        // Calculate the image feature points and describers and save them as attribute of the object
        std::unique_ptr<Image_describer> image_describer(new SIFT_Image_describer(SiftParams(-1)));
        image_describer->Describe(image, _regions_perImage);
        _regions = _regions_perImage.get();
        cout << (*_regions).DescriptorLength() << endl;
        const SIFT_Regions* regions = dynamic_cast<SIFT_Regions*>(_regions_perImage.get());
        _features = _regions_perImage->GetRegionsPositions();
        cout << "Image " << _name << " features count: " << _features.size() << std::endl;
    } else {
        _mat = cv::imread(path);
        _width = _mat.cols;
        _height = _mat.rows;
    }

    _name = getFileName(path);
}

// Cam destructor
Cam::~Cam() {
    delete _matched_kp;
}

// Convert the matched 2D keypoints associated to this object into 3D spherical coordinates
void Cam::toSpherical(Mat& sphericalCoords) const {
	sphericalCoords.resize(3, _matched_kp->cols());
	for (size_t iCol = 0; iCol < _matched_kp->cols(); ++iCol) {
		// The spherical coordinates of keypoints from the flat image are assumed
        // to be the same as cartesian coordinates with a depth of 1
        // NOTE: this assertion provides quite relevant result but could be improved
        const Vec2 & xy = _matched_kp->col(iCol);
        sphericalCoords.col(iCol) = (Vec3 (xy(0), xy(1), 1));
	}
}
