/*
 * LinePlaneIntersectionBase.cpp
 *
 *  Created on: Mar 23, 2017
 *      Author: wilczynski
 */

#include "monolidar_fusion/LinePlaneIntersectionBase.h"

namespace Mono_Lidar {

bool LinePlaneIntersectionBase::GetIntersectionPoint(const Eigen::Vector3d& planeNormal,
                                                     const Eigen::Vector3d& planePoint,
                                                     const Eigen::Vector3d& n0,
                                                     const Eigen::Vector3d& n1,
                                                     Eigen::Vector3d& intersectionPoint,
                                                     double& intersectionDistance) {
    auto plane = Eigen::Hyperplane<double, 3>(planeNormal, planePoint);

    return GetIntersection(plane, n0, n1, intersectionPoint, intersectionDistance);
}

bool LinePlaneIntersectionBase::GetIntersectionPoint(const Eigen::Vector3d& planeNormal,
                                                     const double planeDistance,
                                                     const Eigen::Vector3d& n0,
                                                     const Eigen::Vector3d& n1,
                                                     Eigen::Vector3d& intersectionPoint,
                                                     double& intersectionDistance) {
    auto plane = Eigen::Hyperplane<double, 3>(planeNormal, planeDistance);

    return GetIntersection(plane, n0, n1, intersectionPoint, intersectionDistance);
}

bool LinePlaneIntersectionBase::GetIntersectionPoint(const Eigen::Vector3d& p1,
                                                     const Eigen::Vector3d& p2,
                                                     const Eigen::Vector3d& p3,
                                                     const Eigen::Vector3d& n0,
                                                     const Eigen::Vector3d& n1,
                                                     Eigen::Vector3d& intersectionPoint,
                                                     double& intersectionDistance) {
    auto plane = Eigen::Hyperplane<double, 3>::Through(p1, p2, p3);

    return GetIntersection(plane, n0, n1, intersectionPoint, intersectionDistance);
}


bool LinePlaneIntersectionBase::GetTriangleIntersectionPoint(const Eigen::Vector3d &p1,
                                                             const Eigen::Vector3d &p2,
                                                             const Eigen::Vector3d &p3,
                                                             const Eigen::Vector3d &rayOrigin,
                                                             const Eigen::Vector3d &rayDirection,
                                                             Eigen::Vector3d &intersectionPoint,
                                                             double &intersectionDistance) {
    // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
    const double EPSILON = 0.0000001;
    Eigen::Vector3d direction = rayDirection.normalized();
    Eigen::Vector3d edge1, edge2, h, s, q;
    double a,f,u,v;
    edge1 = p2 - p1;
    edge2 = p3 - p1;
    h = rayDirection.cross(edge2);
    a = edge1.dot(h);
    if (a > -EPSILON && a < EPSILON)
        return false;    // This ray is parallel to this triangle.
    f = 1.0/a;
    s = rayOrigin - p1;
    u = f * (s.dot(h));
    if (u < 0.0 || u > 1.0)
        return false;
    q = s.cross(edge1);
    v = f * rayDirection.dot(q);
    if (v < 0.0 || u + v > 1.0)
        return false;
    // At this stage we can compute t to find out where the intersection point is on the line.
    intersectionDistance = f * edge2.dot(q);
    if (intersectionDistance > EPSILON) // ray intersection
    {
        intersectionPoint = rayOrigin + rayDirection * intersectionDistance;
        return true;
    }
    else // This means that there is a line intersection but not a ray intersection.
        return false;
}

} // end namespace Mono_Lidar
