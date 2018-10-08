#include <iostream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

void pose_estimation_2d2d(
    std::vector<KeyPoint> keypoints_1,
    std::vector<KeyPoint> keypoints_2,
    std::vector< DMatch > matches,
    cv::Mat R, cv::Mat &t)
{
    cv::Mat K = (cv::Mat_<double> (3,3) << 520.9,0,325.1,0,521.0,249.7,0,0,1);
    std::vector<Point2f> points1;
    std::vector<Point2f> points2;

    for(int i=0;i<(int)matches.size();i++){
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    //
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat( points1, points2, CV_FM_8POINT );
    cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    Point2d principal_point (325.1,249.7);
    int focal_length = 521;
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1,points2,focal_length,principal_point,RANSAC);
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    Mat homography_matrix;
    homography_matrix = findHomography(points1,points2,RANSAC,3,noArray(),2000,0.99);
    cout<<"homography is"<<endl<<homography_matrix<<endl;

    recoverPose(essential_matrix,points1,points2,R,t,focal_length,principal_point);
    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;
}