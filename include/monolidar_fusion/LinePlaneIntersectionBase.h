/*
 * LinePlaneIntersectionBase.h
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#pragma once

#include <Eigen/Eigen>

namespace Mono_Lidar {
/*
 * Calculates the intersection between a line and a plane.
 *
 * Is used for the 3d interpolation of 2d feature points using it's 3D neighbor lidar points
 */
class LinePlaneIntersectionBase {
public:
    virtual ~LinePlaneIntersectionBase() {
    }

    /*
     * Gets the intersection point between a plane and a parameterized line (starting by n0 through n1)
     * @param planeNormal [in] Normal of the plane
     * @param planePoint [in] Arbritrary point on the plane
     * @param n0 [in] Starting point of the parameterized line
     * @param n1 [in] passageway point of the parameterized line
     * @param intersectionPoint [out] Intersection point between the line and the plane
     * @param intersectionDistance [out] Distance between the intersection point and the origin of the line [meter]
     */
    bool GetIntersectionPoint(const Eigen::Vector3d& planeNormal,
                              const Eigen::Vector3d& planePoint,
                              const Eigen::Vector3d& n0,
                              const Eigen::Vector3d& n1,
                              Eigen::Vector3d& intersectionPoint,
                              double& intersectionDistance);

    /*
     * Gets the intersection point between a plane  and a parameterized line (starting by n0 through n1)
     * @param planeNormal [in] Normal of the plane
     * @param planeDistance [in] Distance from the plane and the coordinate base
     * @param n0 [in] Starting point of the parameterized line
     * @param n1 [in] passageway point of the parameterized line
     * @param intersectionPoint [out] Intersection point between the line and the plane
     * @param intersectionDistance [out] Distance between the intersection point and the origin of the line [meter]
     */
    bool GetIntersectionPoint(const Eigen::Vector3d& planeNormal,
                              const double planeDistance,
                              const Eigen::Vector3d& n0,
                              const Eigen::Vector3d& n1,
                              Eigen::Vector3d& intersectionPoint,
                              double& intersectionDistance);

    /*
     * Gets the intersection point between a plane (spanned by three points p1, p2, p3) and a parameterized line
     * (starting by n0 through n1)
     * @param p1 [in] First point of the triangle which spans a plane
     * @param p2 [in] Second point of the triangle which spans a plane
     * @param p3 [in] Third point of the triangle which spans a plane
     * @param n0 [in] Starting point of the parameterized line
     * @param n1 [in] passageway point of the parameterized line
     * @param intersectionPoint [out] Intersection point between the line and the plane
     * @param intersectionDistance [out] Distance between the intersection point and the origin of the line [meter]
     */
    bool GetIntersectionPoint(const Eigen::Vector3d& p1,
                              const Eigen::Vector3d& p2,
                              const Eigen::Vector3d& p3,
                              const Eigen::Vector3d& n0,
                              const Eigen::Vector3d& n1,
                              Eigen::Vector3d& intersectionPoint,
                              double& intersectionDistance);

  /*
   * Gets the intersection point between a triangle and a ray using the Moeller-Trumbore algorithm
   * @param p1 [in] First point of the triangle which spans a plane
   * @param p2 [in] Second point of the triangle which spans a plane
   * @param p3 [in] Third point of the triangle which spans a plane
   * @param rayOrigin [in] Starting point of the ray
   * @param rayDirection [in] Direction of the ray
   * @param intersectionPoint [out] Intersection point between the line and the plane
   * @param intersectionDistance [out] Distance between the intersection point and the origin of the line [meter]
   */
    bool GetTriangleIntersectionPoint(const Eigen::Vector3d& p1,
                                      const Eigen::Vector3d& p2,
                                      const Eigen::Vector3d& p3,
                                      const Eigen::Vector3d& rayOrigin,
                                      const Eigen::Vector3d& rayDirection,
                                      Eigen::Vector3d& intersectionPoint,
                                      double& intersectionDistance);


    /*
     * Gets the intersection point between a plane  and a parameterized line (starting by n0 through n1)
     * @param plane [in] The Plane
     * @param n0 [in] Starting point of the parameterized line
     * @param n1 [in] passageway point of the parameterized line
     * @param intersectionPoint [out] Intersection point between the line and the plane
     * @param intersectionDistance [out] Distance between the intersection point and the origin of the line [meter]
     */
    virtual bool GetIntersection(const Eigen::Hyperplane<double, 3>& plane,
                                 const Eigen::Vector3d& n0,
                                 const Eigen::Vector3d& n1,
                                 Eigen::Vector3d& intersectionPoint,
                                 double& intersectionDistance) = 0;

  /*
   * Gets the intersection point between a plane  and a parameterized line (starting by n0 through n1)
   * @param plane [in] The Plane
   * @param n0 [in] Starting point of the parameterized line
   * @param n1 [in] passageway point of the parameterized line
   * @param intersectionPoint [out] Intersection point between the line and the plane
   * @param intersectionDistance [out] Distance between the intersection point and the origin of the line [meter]
   */
  virtual bool GetIntersectionDistance(const Eigen::Vector3d& p1,
                                       const Eigen::Vector3d& p2,
                                       const Eigen::Vector3d& p3,
                                       const Eigen::Vector3d& rayOrigin,
                                       const Eigen::Vector3d& rayDirection,
                                       double& intersectionDistance) = 0;
};
}
