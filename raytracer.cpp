#include <iostream>
#include "parser.h"
#include "ppm.h"
#include "utility.h"

#include <math.h>
#define epsilon 0.001
typedef unsigned char RGB[3];

using namespace utility;

parser::Color computeColor(parser::Ray ray, parser::Scene &scene);
parser::Color applyShading(parser::Ray ray, parser::hitRecord hitRecord, parser::Scene &scene);

bool sphereIntersect(parser::Ray &ray, parser::hitRecord &hitRecord, double sphereRadius, parser::Vec3f &sphereCenter);
bool triangleIntersect(parser::Ray &ray, parser::hitRecord &hitRecord, parser::Vec3f &a, parser::Vec3f &b, parser::Vec3f &c);

bool shadowIntersect(parser::Ray &ray, parser::Scene &scene, parser::PointLight &light);

bool sphereIntersect(parser::Ray &ray, parser::hitRecord &hitRecord, double sphereRadius, parser::Vec3f &sphereCenter)
{

    double discriminant;

    parser::Vec3f eksiC = Scale(sphereCenter, -1);
    parser::Vec3f oEksiC = Sum(ray.origin, eksiC);

    double A = DotProduct(ray.direction,ray.direction);
    double B = 2*DotProduct(ray.direction, oEksiC);
    double C = DotProduct(oEksiC,oEksiC) - sphereRadius*sphereRadius;

    discriminant = B*B - 4*A*C;

    if (discriminant < 0) // no intersection
    {
        return false;
    }

    else // there is intersection
    {       
            
        double t1 = (-1*B + sqrt(discriminant)) / (2*A);
        double t2 = (-1*B - sqrt(discriminant)) / (2*A);

        if (t1 < t2 && t1 > 0)
        {
            hitRecord.hitPointForSphere = Sum(ray.origin, Scale(ray.direction, t1));
            hitRecord.tValue = t1;
            return true;

        }
        if (t2 < t1 && t2 > 0)
        {
            hitRecord.hitPointForSphere = Sum(ray.origin, Scale(ray.direction, t2));
            hitRecord.tValue = t2;
            return true;

        }

        hitRecord.tValue = -1;
        return false;
    }
    return false;
}
bool triangleIntersect(parser::Ray &ray, parser::hitRecord &hitRecord, parser::Vec3f &a, parser::Vec3f &b, parser::Vec3f &c)
{

    double beta, gama, T;

    parser::Vec3f firstColumnA; parser::Vec3f secondColumnA; parser::Vec3f thirdColumnA;

    firstColumnA = Sum(a, Scale(b,-1));
    secondColumnA = Sum(a, Scale(c,-1));
    thirdColumnA = ray.direction;

    double detA = Determinant(firstColumnA, secondColumnA, thirdColumnA);

    parser::Vec3f firstColumnBeta; parser::Vec3f secondColumnBeta; parser::Vec3f thirdColumnBeta;
    firstColumnBeta = Sum(a, Scale(ray.origin, -1));
    secondColumnBeta = Sum(a,Scale(c, -1));
    thirdColumnBeta = ray.direction;

    beta = Determinant(firstColumnBeta, secondColumnBeta, thirdColumnBeta) / detA;

    if (beta < -epsilon)
    {
        hitRecord.tValue = -1;
        return false;
    }

    parser::Vec3f firstColumnGama; parser::Vec3f secondColumnGama; parser::Vec3f thirdColumnGama;
    firstColumnGama = Sum(a, Scale(b, -1));
    secondColumnGama = Sum(a,Scale(ray.origin, -1));
    thirdColumnGama = ray.direction;

    gama = Determinant(firstColumnGama, secondColumnGama, thirdColumnGama) / detA;

    if (gama < -epsilon)
    {
        hitRecord.tValue = -1;
        return false;
    }

    parser::Vec3f firstColumnT; parser::Vec3f secondColumnT; parser::Vec3f thirdColumnT;
    firstColumnT = Sum(a, Scale(b, -1));
    secondColumnT = Sum(a,Scale(c, -1));
    thirdColumnT = Sum(a,Scale(ray.origin, -1));

    T = Determinant(firstColumnT, secondColumnT, thirdColumnT) / detA;

    if (T < -epsilon)
    {
        hitRecord.tValue = -1;
        return false;
    }

    if (((beta + gama) < 1))
    {
        hitRecord.tValue = T;
        return true;
    }
    
    hitRecord.tValue = -T;
    return false;

}

parser::hitRecord hitRecord;
std::vector<parser::Vec3f> triangleNormals;

int main(int argc, char* argv[])
{
    parser::Scene scene;
    scene.loadFromXml(argv[1]);
    parser::Camera camera;

    for (int i = 0; i < scene.meshes.size(); i++)
    {
        for (int j = 0; j < scene.meshes[i].faces.size(); j++)
        {
            
            parser::Triangle meshTriangle;
            meshTriangle.material_id = scene.meshes[i].material_id;
            meshTriangle.indices.v0_id = scene.meshes[i].faces[j].v0_id;
            meshTriangle.indices.v1_id = scene.meshes[i].faces[j].v1_id;
            meshTriangle.indices.v2_id = scene.meshes[i].faces[j].v2_id;

            scene.triangles.push_back(meshTriangle);
        }
    }
    
    for (int i = 0; i < scene.triangles.size(); i++)
    {
        triangleNormals.push_back(triangleNormalFinder(scene.triangles[i], scene));
    }
    
    for(int c = 0; c < scene.cameras.size(); c++) // for each camera another rendering will take place // c is the camera number
    {
        camera = scene.cameras[c];
        unsigned char* image = new unsigned char [camera.image_width * camera.image_height * 3];

        double r,l,t,b;

        l = camera.near_plane.x;
        r = camera.near_plane.y;
        b = camera.near_plane.z;
        t = camera.near_plane.w;

        double nearDistance = camera.near_distance;
        int imageWidth = camera.image_width;
        int imageHeight = camera.image_height;
        std::string imageName = camera.image_name;

        parser::Vec3f cameraV = Normalize(camera.up);
        parser::Vec3f cameraW = Normalize(Scale(camera.gaze, -1));
        parser::Vec3f cameraU = Normalize(CrossProduct(cameraV, cameraW));

        parser::Vec3f e = camera.position; //origin of the camera 
        
        parser::Vec3f m; // middle point of the image plane

        m = Sum(e, Scale(camera.gaze, nearDistance));

        parser::Vec3f q; // left top position of the image plane
        q = Sum(m, Sum(Scale(cameraU, l), Scale(cameraV, t)));

        int image_index = 0;

        double sU = 0;
        double sV = 0;
        for(int i = 0; i < imageHeight; i++)
        {
            sV = (i + 0.5) * ((t-b)/imageHeight);
            for (int j = 0; j < imageWidth; j++)
            {
                parser::Vec3f s;
                s = Sum(q, Sum(Scale(cameraU, sU), Scale(cameraV, -1*sV)));
                sU = (j + 0.5) * ((r-l)/imageWidth);

                parser::Vec3f d = Normalize(Sum(s, Scale(e, -1)));

                parser::Ray primRay;

                primRay.origin = e;
                primRay.direction = d;
                primRay.depth = 0;

                hitRecord.objectType = 0;
                
                parser::Color color = computeColor(primRay, scene);
                color = clamp(color);
                image[image_index++] = (unsigned char)color.R; image[image_index++] = (unsigned char)color.G; image[image_index++] = (unsigned char)color.B;
                
            }
        }

        write_ppm(camera.image_name.c_str(), image, camera.image_width, camera.image_height);
    }

}


parser::Color computeColor(parser::Ray ray, parser::Scene &scene)
{   
    parser::Color returnColor;
    if (ray.depth > scene.max_recursion_depth)
    {
        returnColor.R = 0; returnColor.G = 0; returnColor.B = 0;
        return returnColor;
    }
    
    hitRecord.objectType = 0;  // 1 for sphere, 2 for triangle, 3 for meshes

    returnColor.R = scene.background_color.x;
    returnColor.G = scene.background_color.y;
    returnColor.B = scene.background_color.z;

    double minT = 10000000;

    for (int i = 0; i < scene.spheres.size(); i++) // looking for spheres
    {   
        if (sphereIntersect(ray, hitRecord, (double)scene.spheres[i].radius, scene.vertex_data[scene.spheres[i].center_vertex_id - 1])) // look if there is a t value
        {
            if (hitRecord.tValue < minT && hitRecord.tValue > 0)
            {   
                minT = hitRecord.tValue;
                hitRecord.material = scene.materials[scene.spheres[i].material_id - 1];
                hitRecord.objectType = 1;
                hitRecord.sphere = scene.spheres[i];
                hitRecord.hitPointForSphere = Sum(ray.origin, Scale(ray.direction, hitRecord.tValue));
            }
        }   
    }

    for (int i = 0; i < scene.triangles.size(); i++)
    {   
        parser::Triangle triangle = scene.triangles[i];
        parser::Material triangleMaterial;

        triangleMaterial = scene.materials[triangle.material_id - 1];

        if (triangleIntersect(ray, hitRecord, scene.vertex_data[triangle.indices.v0_id - 1], scene.vertex_data[triangle.indices.v1_id - 1], scene.vertex_data[triangle.indices.v2_id - 1])) // look if there is a t value
        {
            if (hitRecord.tValue < minT && hitRecord.tValue > -epsilon)
            {                
                hitRecord.material = triangleMaterial;
                minT = hitRecord.tValue;
                hitRecord.objectType = 2;
                hitRecord.hitPointForTriangle = Sum(ray.origin, Scale(ray.direction, hitRecord.tValue));
                hitRecord.triangle = triangle;
                hitRecord.triangleID = i;
            }
        }   
    }

    if (hitRecord.objectType == 1) // object founded is sphere
    { 
        return applyShading(ray,hitRecord, scene);
    }
    
    if (hitRecord.objectType == 2) // object founded is a triangle
    { 
        return applyShading(ray, hitRecord, scene);
    }
    

    return returnColor;
    
}

parser::Color applyShading(parser::Ray ray, parser::hitRecord hitRecord, parser::Scene &scene)
{   
    parser::Color returnColorValue;
    returnColorValue.R = 0; returnColorValue.G = 0; returnColorValue.B = 0;

    parser::Material material = hitRecord.material;

    //ambient shading applied
    returnColorValue.R = scene.ambient_light.x * material.ambient.x;
    returnColorValue.G = scene.ambient_light.y * material.ambient.y;
    returnColorValue.B = scene.ambient_light.z * material.ambient.z;

    parser::Color colorReturnedFromDiffuse;
    parser::Color colorReturnedFromSpecular;

    
    if (hitRecord.material.is_mirror) // if the object we are looking is a mirror
    {
        parser::Ray reflectionRay;
        

        //reflection
        if (hitRecord.objectType == 1) // sphere
        {
            parser::Vec3f surfaceNormal;
            surfaceNormal = sphereNormalFinder(hitRecord, scene);


            parser::Ray returnRay;
            double cosTeta;
            cosTeta = myMax(DotProduct(Normalize(Sum((ray.origin), Scale(hitRecord.hitPointForSphere, -1))), surfaceNormal), 0); // bak
            returnRay.origin = Sum(hitRecord.hitPointForSphere, Scale(surfaceNormal, epsilon));
            returnRay.direction = Normalize(Sum((Scale(Normalize(Sum((ray.origin), Scale(hitRecord.hitPointForSphere, -1))) , -1)), Scale((surfaceNormal), 2*cosTeta)));

            reflectionRay = returnRay;
            //reflectionRay = reflect(ray, surfaceNormal, hitRecord.hitPointForSphere);
        }
        if (hitRecord.objectType == 2) // triangle
        {
            parser::Vec3f surfaceNormal = triangleNormals[hitRecord.triangleID];
            //reflectionRay = reflect(ray, triangleNormals[hitRecord.triangleID], hitRecord.hitPointForTriangle);

            //parser::Ray returnRay;
            double cosTeta;
            cosTeta = myMax(DotProduct(Normalize(Sum((ray.origin), Scale(hitRecord.hitPointForTriangle, -1))), surfaceNormal), 0); // bak
            reflectionRay.origin = Sum(hitRecord.hitPointForTriangle, Scale(surfaceNormal, epsilon));
            reflectionRay.direction = Normalize(Sum((Scale(Normalize(Sum((ray.origin), Scale(hitRecord.hitPointForTriangle, -1))) , -1)), Scale((surfaceNormal), 2*cosTeta)));
        }

        reflectionRay.depth = ray.depth + 1;
        parser::Color returnFromRecursion = computeColor(reflectionRay, scene);

        returnColorValue.R += returnFromRecursion.R * material.mirror.x;
        returnColorValue.G += returnFromRecursion.G * material.mirror.y;
        returnColorValue.B += returnFromRecursion.B * material.mirror.z;
    }
    


    for (int l = 0; l < scene.point_lights.size(); l++)
    {   
        bool shadowFlag;
        parser::Ray shadowRay;
        parser::PointLight light = scene.point_lights[l];

        if (hitRecord.objectType == 1) // sphere
        {
            shadowRay.origin = Sum(hitRecord.hitPointForSphere, Scale(sphereNormalFinder(hitRecord, scene), scene.shadow_ray_epsilon));
            shadowRay.direction = Normalize(Sum(light.position, Scale(shadowRay.origin, -1)));
            shadowFlag = shadowIntersect(shadowRay, scene, light);
        }

        if (hitRecord.objectType == 2) // triangle
        {
            shadowRay.origin = Sum(hitRecord.hitPointForTriangle, Scale(triangleNormals[hitRecord.triangleID], scene.shadow_ray_epsilon));
            shadowRay.direction = Normalize(Sum(light.position, Scale(shadowRay.origin, -1)));
            shadowFlag = shadowIntersect(shadowRay, scene, light);

        }

        if (!shadowFlag)
        {

            parser::Vec3f initialRadiance;
            parser::PointLight light = scene.point_lights[l];

            double distanceSquare;
            if (hitRecord.objectType == 1)
            {
                distanceSquare =  pow(Distance(light.position, hitRecord.hitPointForSphere), 2);
            }
            else if (hitRecord.objectType == 2)
            {
                distanceSquare =  pow(Distance(light.position, hitRecord.hitPointForTriangle), 2);
            }

            initialRadiance.x = light.intensity.x / distanceSquare;initialRadiance.y = light.intensity.y / distanceSquare;initialRadiance.z = light.intensity.z / distanceSquare;
    
            parser::Vec3f Wi;
            parser::Vec3f Wo;

            parser::Vec3f normal;

            parser::Vec3f h;
            parser::Vec3f sumVector;

            double phongExponent = hitRecord.material.phong_exponent;

            if (hitRecord.objectType == 1) // sphere
            {
                Wo = Sum(ray.origin, Scale(hitRecord.hitPointForSphere, -1));
                Wi = Sum(light.position, Scale(hitRecord.hitPointForSphere, -1));

                Wi = Normalize(Wi);
                Wo = Normalize(Wo);
                normal = sphereNormalFinder(hitRecord, scene);


                double cosTeta = myMax(DotProduct(Wi, normal), 0);

                colorReturnedFromDiffuse.R = hitRecord.material.diffuse.x * cosTeta * initialRadiance.x;
                colorReturnedFromDiffuse.G = hitRecord.material.diffuse.y * cosTeta * initialRadiance.y;
                colorReturnedFromDiffuse.B = hitRecord.material.diffuse.z * cosTeta * initialRadiance.z;

                sumVector = Sum(Wi, Wo);
                double sumVectorLenght = sqrt(DotProduct(sumVector, sumVector));
                h.x = sumVector.x/sumVectorLenght;
                h.y = sumVector.y/sumVectorLenght;
                h.z = sumVector.z/sumVectorLenght;


                double cosAlpha;
                
                cosAlpha = pow(myMax(DotProduct(normal, h),0), phongExponent);

                /*
                if (cosAlpha <= 0)
                {
                    continue;
                }
                */
                colorReturnedFromSpecular.R = hitRecord.material.specular.x * cosAlpha * initialRadiance.x;
                colorReturnedFromSpecular.G = hitRecord.material.specular.y * cosAlpha * initialRadiance.y;
                colorReturnedFromSpecular.B = hitRecord.material.specular.z * cosAlpha * initialRadiance.z;
    
            }

            else if (hitRecord.objectType == 2) // triangle
            {   
                Wo = Sum(ray.origin, Scale(hitRecord.hitPointForTriangle, -1));
                Wi = Sum(light.position, Scale(hitRecord.hitPointForTriangle, -1));

                Wi = Normalize(Wi);
                Wo = Normalize(Wo);
        
                //normal = triangleNormalFinder(hitRecord.triangle, scene);
                normal = triangleNormals[hitRecord.triangleID];
                double cosTeta = myMax(DotProduct(Wi, normal), 0);

                colorReturnedFromDiffuse.R = hitRecord.material.diffuse.x * cosTeta * initialRadiance.x;
                colorReturnedFromDiffuse.G = hitRecord.material.diffuse.y * cosTeta * initialRadiance.y;
                colorReturnedFromDiffuse.B = hitRecord.material.diffuse.z * cosTeta * initialRadiance.z;

                sumVector = Sum(Wi, Wo);
                double sumVectorLenght = sqrt(DotProduct(sumVector, sumVector));
                h.x = sumVector.x/sumVectorLenght;
                h.y = sumVector.y/sumVectorLenght;
                h.z = sumVector.z/sumVectorLenght;
    
                double cosAlpha;

                normal = triangleNormals[hitRecord.triangleID];
                cosAlpha = pow(myMax(DotProduct(normal, h),0), phongExponent);

                /*
                if (cosAlpha <= 0)
                {
                    continue;
                }
                */
                

                colorReturnedFromSpecular.R = hitRecord.material.specular.x * cosAlpha * initialRadiance.x;
                colorReturnedFromSpecular.G = hitRecord.material.specular.y * cosAlpha * initialRadiance.y;
                colorReturnedFromSpecular.B = hitRecord.material.specular.z * cosAlpha * initialRadiance.z;

                
            }    


            returnColorValue.R += colorReturnedFromDiffuse.R + colorReturnedFromSpecular.R;
            returnColorValue.G += colorReturnedFromDiffuse.G + colorReturnedFromSpecular.G;
            returnColorValue.B += colorReturnedFromDiffuse.B + colorReturnedFromSpecular.B;
        }
        
        
    }

    return returnColorValue;
}


bool shadowIntersect(parser::Ray &ray, parser::Scene &scene, parser::PointLight &light)
{

    double tLight;
    double tObject;

    parser::hitRecord hitRecord;

    tLight = Distance(light.position, ray.origin);
    
    for (int i = 0; i < scene.spheres.size(); i++) // looking for spheres
    {
        /*
        parser::Vec3f sphereCenter;
        sphereCenter = scene.vertex_data[scene.spheres[i].center_vertex_id - 1];

        if (sphereIntersect(ray, hitRecord, scene.spheres[i].radius, sphereCenter)) // look if there is a t value
        {
            if (hitRecord.tValue < tLight && hitRecord.tValue > 0)
            {
                return true;
            }
        } */

        parser::Vec3f sphereCenter;
        sphereCenter = scene.vertex_data[scene.spheres[i].center_vertex_id - 1];
        double sphereRadius = scene.spheres[i].radius;


        double discriminant;

        parser::Vec3f eksiC = Scale(sphereCenter, -1);
        parser::Vec3f oEksiC = Sum(ray.origin, eksiC);

        double A = DotProduct(ray.direction,ray.direction);
        double B = 2*DotProduct(ray.direction, oEksiC);
        double C = DotProduct(oEksiC,oEksiC) - sphereRadius*sphereRadius;

        discriminant = B*B - 4*A*C;

        if (discriminant < 0) // no intersection
        {
            continue;
        }

        else // there is intersection
        {       
            hitRecord.tValue = -1;
            double t1 = (-1*B + sqrt(discriminant)) / (2*A);
            double t2 = (-1*B - sqrt(discriminant)) / (2*A);

            if (t1 < t2 && t1 > -epsilon)
            {
                hitRecord.tValue = t1;

            }
            if (t2 < t1 && t2 > -epsilon)
            {
                hitRecord.tValue = t2;
            }
        }

        if (hitRecord.tValue < tLight && hitRecord.tValue > 0)
        {
            return true;
        }


    }

    for (int i = 0; i < scene.triangles.size(); i++)
    {

        parser::Triangle triangle = scene.triangles[i];

        if (triangleIntersect(ray, hitRecord, scene.vertex_data[triangle.indices.v0_id - 1], scene.vertex_data[triangle.indices.v1_id - 1], scene.vertex_data[triangle.indices.v2_id - 1])) // look if there is a t value
        {
            if (hitRecord.tValue < tLight && hitRecord.tValue > 0)
            {
                return true;
            }
        } 
    }
    return false;
}