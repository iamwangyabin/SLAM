#include <iostream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

void triangulation(
    const vector< KeyPoint > & keypoint_1,
    const vector< KeyPoint > & keypoint_2,
    const std::vector< DMatch > & matches,
    const Mat& R, const Mat& t,
    vector<Point3d>& points
);

void triangulation(
    const vector< KeyPoint > & keypoint_1,
    const vector< KeyPoint > & keypoint_2,
    const std::vector< DMatch > & matches,
    const Mat& R, const Mat& t,
    vector<Point3d>& points
)
{
    Mat T1 = (Mat_<double> (3,4)<<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0);
    Mat T2 = (Mat_<double> (3,4)<<
        )
}