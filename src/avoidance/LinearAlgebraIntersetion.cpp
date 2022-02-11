//
// Created by micha on 11/28/2021.
//

#include "../avoidance/LinearAlgebraIntersetion.h"

bool LinearAlgebraIntersection::ThereIsIntersectionAlongThePath(const Vec3 &LinePointStart,
                                                                const Vec3 &LinePointEnd,
                                                                const Vec3 &obstacleCoords,
                                                                double droneRadius,
                                                                double obstacleRadius,
                                                                double obstacleHeight) const {
    Vec3 LinePointStartNew(LinePointStart);
    Vec3 LinePointEndNew(LinePointEnd);
    Vec3 obstacleCoordsNew(obstacleCoords);

    if (obstacleHeight > 0) { // if the obstacle is a cylinder
        LinePointStartNew.z = 0;
        LinePointEndNew.z = 0;
        obstacleCoordsNew.z = 0;
    }

    const Vec3 LineDiffVect = LinePointEndNew - LinePointStartNew;

    const Vec3 &copy = LineDiffVect;

    const double lineSegSqrLength = LineDiffVect | copy;

    const Vec3 LineToPointVect = obstacleCoordsNew - LinePointStartNew;

    const double dotProduct = LineDiffVect | LineToPointVect;

    double percAlongLine = dotProduct / lineSegSqrLength;

    if (percAlongLine < 0.0l) {
        percAlongLine = 0.0l;
    } else if (percAlongLine > 1.0l) {
        percAlongLine = 1.0l;
    }

    const Vec3 IntersectionPt = ((LinePointEndNew - LinePointStartNew) * percAlongLine) + LinePointStartNew;
    //    std::cout << "IntersectionPt(closest to center?): " << IntersectionPt.x << " " << IntersectionPt.y << " " << IntersectionPt.z << "\n";
    const Vec3 SpherePtToIntersect = IntersectionPt - obstacleCoordsNew;
    const Vec3 &copy2 = SpherePtToIntersect;
    const double SqrLengthSphereToLine = SpherePtToIntersect | copy2;

    if (SqrLengthSphereToLine <= pow(obstacleRadius, 2)) {
        if (obstacleHeight > 0) { // is a cylinder

            double bottom_of_a_cylinder = obstacleCoords.z - (obstacleHeight / 2.0);
            double top_of_a_cylinder = obstacleCoords.z + (obstacleHeight / 2.0);

            if ((LinePointStart.z > top_of_a_cylinder and LinePointEnd.z > top_of_a_cylinder) or
            (LinePointStart.z < bottom_of_a_cylinder and LinePointEnd.z < bottom_of_a_cylinder)) {
                return false;
            }
        }
        return true;
    }
    else return false;
}
