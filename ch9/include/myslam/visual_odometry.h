#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam 
{
class VisualOdometry{
    public:
        typedef shared_ptr<VisualOdometry> Ptr;
        enum VOState{
            INITIALIZING=-1,
            OK=0,
            LOST
        };
        VOState 
}
}