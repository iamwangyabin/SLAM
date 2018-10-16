#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam 
{
    
// forward declare 
class MapPoint;
class Frame{
    public:
        typedef std::shared_ptr<Frame> Ptr;
        unsigned long id_;
        double time_stamp_;
        SE3 T_c_w;
        Camera::Ptr camera_;
        Mat color_, depth_;
    
    public:
        Frame();
        Frame(long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat());
        ~Frame();

        static Frame::Ptr createFrame();

        double findDepth(const cv::KeyPoint& kp);
        Vector3d getCamCenter() const;

        bool isInFrame(const Vector3d& pt_world);
};

class MapPoint{
    public:
        typedef std::shared_ptr<MapPoint> Ptr;
        unsigned long id_;
        Vector3d pos_;
        Vector3d norm_;
        Mat descriptor_;
        int observed_times_;
        int correct_times_;

        MapPoint();
        MapPoint(long id, Vector3d position, Vector3d norm);
        
        static MapPoint::Ptr createMapPoints();
};
}

#endif