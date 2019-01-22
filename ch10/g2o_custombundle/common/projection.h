//
// Created by wang on 19-1-22.
//

#ifndef SLAMBOOK_PROJECTION_H
#define SLAMBOOK_PROJECTION_H

#include "tools/rotation.h"

// camera : 9 dims array with
// [0-2] : angle-axis rotation
// [3-5] : translateion
// [6-8] : camera parameter, [6] focal length, [7-8] second and forth order radial distortion
// point : 3D location.
// predictions : 2D predictions with center of the image plane.
/**
 *g2o还是用图模型和边，顶点就是相机和路标，边就是观测，就是像素坐标。只不过这里的相机是由旋转（3个参数，轴角形式，就是theta*nx,theta*ny,theta*ny),位移(3个参数），f,k1,k2.就是之前BA模型的实现。
 *但是这里归一化平面坐标取得是负值，而且最后没有加cx,cy.
 * */
template<typename T>
inline bool CamProjectionWithDistortion(const T* camera, const T* point, T* predictions){
    // Rodrigues' formula
    T p[3];
    AngleAxisRotatePoint(camera, point, p);
    // camera[3,4,5] are the translation
    p[0] += camera[3]; p[1] += camera[4]; p[2] += camera[5];

    // Compute the center fo distortion
    T xp = -p[0]/p[2];
    T yp = -p[1]/p[2];

    // Apply second and fourth order radial distortion
    const T& l1 = camera[7];
    const T& l2 = camera[8];

    T r2 = xp*xp + yp*yp;
    T distortion = T(1.0) + r2 * (l1 + l2 * r2);

    const T& focal = camera[6];
    predictions[0] = focal * distortion * xp;
    predictions[1] = focal * distortion * yp;

    return true;
}


#endif //SLAMBOOK_PROJECTION_H
