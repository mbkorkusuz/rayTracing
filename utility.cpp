#include "parser.h"
#include "utility.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <math.h>


parser::Vec3f utility::CrossProduct(parser::Vec3f first, parser::Vec3f second)
{
    parser::Vec3f result;

    result.x = (first.y * second.z) - (first.z * second.y);
    result.y = -1 * ((first.x * second.z) - (first.z * second.x));
    if (result.y == -0)
    {
        result.y = 0;
    }
    
    result.z = (first.x * second.y) - (first.y * second.x);

    return result;
}

parser::Vec3f utility::Normalize(parser::Vec3f vector)
{
    parser::Vec3f normalized;
    double length;

    length = sqrt(pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2));
    

    normalized.x = vector.x / length;
    normalized.y = vector.y / length;
    normalized.z = vector.z / length;

    return normalized;
}

double utility::DotProduct(parser::Vec3f first, parser::Vec3f second)
{
    double result;
    result = (first.x * second.x) + (first.y * second.y) + (first.z * second.z);
    return result;
}

parser::Vec3f utility::Scale(parser::Vec3f vector, double scaleFactor)
{
    if (vector.x != 0)
    {
        vector.x *= scaleFactor;
    }
    if (vector.y != 0)
    {
        vector.y *= scaleFactor;
    }
    if (vector.z != 0)
    {
        vector.z *= scaleFactor;
    }
    return vector;
}

parser::Vec3f utility::Sum(parser::Vec3f first, parser::Vec3f second)
{
    parser::Vec3f result;

    result.x = first.x + second.x;
    result.y = first.y + second.y;
    result.z = first.z + second.z;

    return result;
}

double utility::Distance(parser::Vec3f first, parser::Vec3f second)
{
    double distance;
    distance = sqrt(pow(first.x - second.x, 2) + pow(first.y - second.y, 2) + pow(first.z - second.z, 2));
    return distance;
}

double utility::Determinant(parser::Vec3f &firstColumn, parser::Vec3f &secondColumn, parser::Vec3f &thirdColumn)
{

    double detValue;
    detValue = (firstColumn.x* (secondColumn.y * thirdColumn.z - thirdColumn.y * secondColumn.z)) -
               (secondColumn.x*(firstColumn.y * thirdColumn.z - thirdColumn.y * firstColumn.z)) + 
               (thirdColumn.x*(firstColumn.y * secondColumn.z - secondColumn.y* firstColumn.z));

    
    return detValue;
}

void utility::printVec(parser::Vec3f vector)
{
    std::cout << "x: " << vector.x << " y: " << vector.y << " z: " << vector.z << "\n";
}

parser::Color utility::clamp(parser::Color color)

{
    if (color.R > 255.0f)
    {
        color.R = 255;
    }
    if (color.G > 255.0f)
    {
        color.G = 255;
    }
    if (color.B > 255.0f)
    {
        color.B = 255;
    }

    if (color.R < 0.0f)
    {
        color.R = 0;
    }
    if (color.G < 0.0f)
    {
        color.G = 0;
    }
    if (color.B < 0.0f)
    {
        color.B = 0;
    }
    return color;
}

double utility::myMax(double n1, double n2)
{
    if (n1 > n2)
    {
        return n1;
    }

    return n2;
    
}

parser::Vec3f utility::triangleNormalFinder(parser::Triangle triangle, parser::Scene &scene)
{

    parser::Vec3f normal;
    parser::Vec3f a,b,c;

    a = scene.vertex_data[triangle.indices.v0_id - 1];
    b = scene.vertex_data[triangle.indices.v1_id - 1];
    c = scene.vertex_data[triangle.indices.v2_id - 1];

    normal = CrossProduct(Sum(b, Scale(a,-1)), Sum(c, Scale(a,-1)));

    double length = sqrt(pow(normal.x, 2) + pow(normal.y, 2) + pow(normal.z, 2));

    normal.x = normal.x / length;
    normal.y = normal.y / length;
    normal.z = normal.z / length;

    return normal;
}

parser::Vec3f utility::sphereNormalFinder(parser::hitRecord hitRecord, parser::Scene &scene)
{

    parser::Vec3f normal;

    parser::Vec3f sphereCenter = scene.vertex_data[hitRecord.sphere.center_vertex_id - 1];




    normal = Sum(hitRecord.hitPointForSphere, Scale(sphereCenter, - 1));
    normal = Normalize(normal);
 
    return normal;

    
}