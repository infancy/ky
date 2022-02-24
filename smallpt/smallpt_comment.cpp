// http://www.kevinbeason.com/smallpt
#include "erand48.h" // pseudo-random number generator, not in <stdlib.h>
#include <cmath>    // smallpt, a Path Tracer by Kevin Beason, 2008
#include <cstdlib>  // Make : g++ -O3 -fopenmp smallpt.cpp -o smallpt
#include <cstdio>   //        Remove "-fopenmp" for g++ version < 4.2

#include <format>
#include <numbers>

constexpr double Pi = std::numbers::pi;

struct Vector3
{                   // Usage: time ./smallpt 5000 && xv image.ppm
    double x, y, z; // position, also color (r,g,b)

    Vector3(double x_ = 0, double y_ = 0, double z_ = 0)
    {
        x = x_;
        y = y_;
        z = z_;
    }

    Vector3 operator+(const Vector3& b) const { return Vector3(x + b.x, y + b.y, z + b.z); }
    Vector3 operator-(const Vector3& b) const { return Vector3(x - b.x, y - b.y, z - b.z); }
    Vector3 operator*(double b) const { return Vector3(x * b, y * b, z * b); }

    // only for Color
    Vector3 Multiply(const Vector3& b) const { return Vector3(x * b.x, y * b.y, z * b.z); }

    Vector3& Normalize() { return *this = *this * (1 / sqrt(x * x + y * y + z * z)); }

    double Dot(const Vector3& b) const { return x * b.x + y * b.y + z * b.z; }
    Vector3 Cross(Vector3& b) { return Vector3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x); }
};

using Float3 = Vector3;
using Point3 = Vector3;
using Normal3 = Vector3;
using UnitVector3 = Vector3;
using Color = Vector3;

struct Ray
{
    Point3 origin;
    UnitVector3 direction;

    Ray(Point3 origin_, UnitVector3 direction_): origin(origin_), direction(direction_) {}
};

enum class MaterialEnum
{
    DIFFUSE,
    SPECULAR,
    REFRACT
}; // material types, used in radiance()

struct Sphere
{
    double radius;
    Point3 position;

    Color emission; // for area light
    Color color; // surface reflectance
    MaterialEnum materialEnum; // scattering type

    Sphere(double radius_, Vector3 position_, Vector3 emission_, Vector3 color_, MaterialEnum materialEnum):
        radius(radius_), position(position_), emission(emission_), color(color_), materialEnum(materialEnum) {}

    double Intersect(const Ray& ray) const
    {
        // returns distance, 0 if nohit

        // Solve t^2*d.d + 2*t*(o-p).d + (o-p).(o-p)-R^2 = 0
        Vector3 op = position - ray.origin;
        double t{};
        double epsilon = 1e-4;
        double b = op.Dot(ray.direction);
        double det = b * b - op.Dot(op) + radius * radius;

        if (det < 0)
            return 0;
        else
            det = sqrt(det);

        return (t = b - det) > epsilon ? t : ((t = b + det) > epsilon ? t : 0);
    }
};

Sphere Scene[] =
{
    //Scene: radius, position, emission, color, material
    Sphere(1e5, Vector3(1e5 + 1, 40.8, 81.6),   Color(), Color(.75, .25, .25), MaterialEnum::DIFFUSE), //Left
    Sphere(1e5, Vector3(-1e5 + 99, 40.8, 81.6), Color(), Color(.25, .25, .75), MaterialEnum::DIFFUSE), //Right
    Sphere(1e5, Vector3(50, 40.8, 1e5),         Color(), Color(.75, .75, .75), MaterialEnum::DIFFUSE), //Back
    Sphere(1e5, Vector3(50, 40.8, -1e5 + 170),  Color(), Color(),              MaterialEnum::DIFFUSE), //Front
    Sphere(1e5, Vector3(50, 1e5, 81.6),         Color(), Color(.75, .75, .75), MaterialEnum::DIFFUSE), //Bottom
    Sphere(1e5, Vector3(50, -1e5 + 81.6, 81.6), Color(), Color(.75, .75, .75), MaterialEnum::DIFFUSE), //Top

    Sphere(16.5, Vector3(27, 16.5, 47),          Color(), Color(1, 1, 1) * .999, MaterialEnum::SPECULAR), //Mirror
    Sphere(16.5, Vector3(73, 16.5, 78),          Color(), Color(1, 1, 1) * .999, MaterialEnum::REFRACT),  //Glass
    Sphere(600,  Vector3(50, 681.6 - .27, 81.6), Color(12, 12, 12), Color(),     MaterialEnum::DIFFUSE)   //Light
};

inline double Clamp(double x) { return x < 0 ? 0 : x > 1 ? 1 : x; }
inline int GammaEncoding(double x) { return int(pow(Clamp(x), 1 / 2.2) * 255 + .5); }

inline bool Intersect(const Ray& ray, double& minDistance, int& id)
{
    double infinity = 1e20;
    minDistance = infinity;

    int sphereNum = sizeof(Scene) / sizeof(Sphere);
    double distance{};

    for (int i = sphereNum; i--;)
    {
        if ((distance = Scene[i].Intersect(ray)) && distance < minDistance)
        {
            minDistance = distance;
            id = i;
        }
    }

    return minDistance < infinity;
}

Vector3 Radiance(const Ray& ray, int depth, unsigned short* sampler)
{
    double distance;   // distance to intersection
    int id = 0; // id of intersected object

    if (!Intersect(ray, distance, id))
        return Color(); // if miss, return black

    const Sphere& obj = Scene[id]; // the hit object

    if (depth > 100)
        return obj.emission;

    Vector3 position = ray.origin + ray.direction * distance;
    Normal3 normal = (position - obj.position).Normalize();
    Normal3 shading_normal = normal.Dot(ray.direction) < 0 ? normal : normal * -1;

    Color f = obj.color; // bsdf value
    double max_component = f.x > f.y && f.x > f.z ? f.x : f.y > f.z ? f.y : f.z; // max refl

    //russian roulette
    if (++depth > 5)
    {
        if (erand48(sampler) < max_component)
            f = f * (1 / max_component);
        else
            return obj.emission;
    }

    if (obj.materialEnum == MaterialEnum::DIFFUSE)
    { // Ideal DIFFUSE reflection
        double random1 = 2 * Pi * erand48(sampler);
        double random2 = erand48(sampler);
        double random2Sqrt = sqrt(random2);

        // shading coordinate on intersection
        Vector3 w = shading_normal;
        Vector3 u = ((fabs(w.x) > .1 ? Vector3(0, 1, 0) : Vector3(1, 0, 0)).Cross(w)).Normalize();
        Vector3 v = w.Cross(u);

        Vector3 direction = (u * cos(random1) * random2Sqrt + v * sin(random1) * random2Sqrt + w * sqrt(1 - random2)).Normalize();

        return obj.emission + f.Multiply(Radiance(Ray(position, direction), depth, sampler));
    }
    else if (obj.materialEnum == MaterialEnum::SPECULAR) // Ideal SPECULAR reflection
    {
        Vector3 direction = ray.direction - normal * 2 * normal.Dot(ray.direction);
        return obj.emission + f.Multiply(Radiance(Ray(position, direction), depth, sampler));
    }
    else
    {
        Ray reflRay(position, ray.direction - normal * 2 * normal.Dot(ray.direction)); // Ideal dielectric REFRACTION
        bool into = normal.Dot(shading_normal) > 0;                // Ray from outside going in?
        double nc = 1, nt = 1.5, nnt = into ? nc / nt : nt / nc, ddn = ray.direction.Dot(shading_normal), cos2t;

        if ((cos2t = 1 - nnt * nnt * (1 - ddn * ddn)) < 0) // Total internal reflection
            return obj.emission + f.Multiply(Radiance(reflRay, depth, sampler));

        Vector3 tdir = (ray.direction * nnt - normal * ((into ? 1 : -1) * (ddn * nnt + sqrt(cos2t)))).Normalize();
        double a = nt - nc, b = nt + nc, R0 = a * a / (b * b), c = 1 - (into ? -ddn : tdir.Dot(normal));
        double Re = R0 + (1 - R0) * c * c * c * c * c, Tr = 1 - Re, P = .25 + .5 * Re, RP = Re / P, TP = Tr / (1 - P);

        return obj.emission + f.Multiply(depth > 2 ? (erand48(sampler) < P ? // Russian roulette
                                               Radiance(reflRay, depth, sampler) * RP : Radiance(Ray(position, tdir), depth, sampler) * TP)
                                        : Radiance(reflRay, depth, sampler) * Re + Radiance(Ray(position, tdir), depth, sampler) * Tr);
    }
}

int main(int argc, char* argv[])
{
    int width = 1024, height = 768;
    int samples_per_pixel = argc == 2 ? atoi(argv[1]) / 4 : 10;

    Ray camera(Vector3(50, 52, 295.6), Vector3(0, -0.042612, -1).Normalize()); // camera posotion, direction
    Vector3 cx = Vector3(width * .5135 / height), cy = (cx.Cross(camera.direction)).Normalize() * .5135;
    
    Color* film = new Vector3[width * height];

    #pragma omp parallel for schedule(dynamic, 1) // OpenMP
    for (int y = 0; y < height; y++) // Loop over image rows
    {
        fprintf(stderr, "\rRendering (%d spp) %5.2f%%", samples_per_pixel * 4, 100. * y / (height - 1));

        unsigned short sampler[3] = { 0, 0, y * y * y };
        for (unsigned short x = 0; x < width; x++) // Loop cols
        {
            Color color{};
            for (int sy = 0, i = (height - y - 1) * width + x; sy < 2; sy++) // 2x2 subpixel rows
            {
                for (int sx = 0; sx < 2; sx++) // 2x2 subpixel cols
                {
                    for (int s = 0; s < samples_per_pixel; s++)
                    {
                        double random1 = 2 * erand48(sampler), dx = random1 < 1 ? sqrt(random1) - 1 : 1 - sqrt(2 - random1);
                        double random2 = 2 * erand48(sampler), dy = random2 < 1 ? sqrt(random2) - 1 : 1 - sqrt(2 - random2);

                        Vector3 direction =
                            cx * (((sx + .5 + dx) / 2 + x) /  width - .5) + 
                            cy * (((sy + .5 + dy) / 2 + y) / height - .5) + camera.direction;

                        color = color + Radiance(Ray(camera.origin + direction * 140, direction.Normalize()), 0, sampler) * (1. / samples_per_pixel);
                    } // Camera rays are pushed ^^^^^ forward to start in interior

                    film[i] = film[i] + Vector3(Clamp(color.x), Clamp(color.y), Clamp(color.z)) * .25;
                }
            }
        }
    }

    FILE* image = fopen("image.ppm", "w"); // Write image to PPM file.
    fprintf(image, "P3\n%d %d\n%d\n", width, height, 255);
    for (int i = 0; i < width * height; i++)
        fprintf(image, "%d %d %d ", GammaEncoding(film[i].x), GammaEncoding(film[i].y), GammaEncoding(film[i].z));

    return 0;
}