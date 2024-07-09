#ifndef __UTILITY__
#define __UTILITY__
#include "parser.h"
namespace utility
{
    void printVec(parser::Vec3f vector);

    double Distance(parser::Vec3f first, parser::Vec3f second);
    parser::Vec3f Sum(parser::Vec3f first, parser::Vec3f second);
    parser::Vec3f Scale(parser::Vec3f vector, double scaleFactor);
    double DotProduct(parser::Vec3f first, parser::Vec3f second);
    parser::Vec3f Normalize(parser::Vec3f vector);
    parser::Vec3f CrossProduct(parser::Vec3f first, parser::Vec3f second);
    double Determinant(parser::Vec3f &firstColumn, parser::Vec3f &secondColumn, parser::Vec3f &thirdColumn);
    parser::Color clamp(parser::Color color);
    double myMax(double n1, double n2);

    parser::Vec3f triangleNormalFinder(parser::Triangle triangle, parser::Scene &scene);
    parser::Vec3f sphereNormalFinder(parser::hitRecord hitRecord, parser::Scene &scene);


}


#endif