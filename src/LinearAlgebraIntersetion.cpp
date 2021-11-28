//
// Created by micha on 11/28/2021.
//

#include "LinearAlgebraIntersetion.h"

bool LinearAlgebraIntersection::ThereIsIntersectionAlongThePath(const Vec3 &LinePointStart,
                                                                const Vec3 &LinePointEnd,
                                                                const Vec3 &obstacleCoords,
                                                                double droneRadius,
                                                                double obstacleRadius) const {
    {
        const Vec3 LineDiffVect = LinePointEnd - LinePointStart;

        const Vec3 &copy = LineDiffVect;

        const double lineSegSqrLength = LineDiffVect | copy;

        const Vec3 LineToPointVect = obstacleCoords - LinePointStart;

        const double dotProduct = LineDiffVect | LineToPointVect;

        double percAlongLine = dotProduct / lineSegSqrLength;

        if (percAlongLine < 0.0l) {
            percAlongLine = 0.0l;
        } else if (percAlongLine > 1.0l) {
            percAlongLine = 1.0l;
        }

        const Vec3 IntersectionPt = ((LinePointEnd - LinePointStart) * percAlongLine) + LinePointStart;
        //    std::cout << "IntersectionPt(closest to center?): " << IntersectionPt.x << " " << IntersectionPt.y << " " << IntersectionPt.z << "\n";
        const Vec3 SpherePtToIntersect = IntersectionPt - obstacleCoords;
        const Vec3 &copy2 = SpherePtToIntersect;
        const double SqrLengSphereToLine = SpherePtToIntersect | copy2;

        return (SqrLengSphereToLine <= pow(obstacleRadius, 2));
    }
}
