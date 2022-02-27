// http://www.kevinbeason.com/smallpt
#include <cmath>    // smallpt, a Path Tracer by Kevin Beason, 2008
#include <cstdlib>  // Make : g++ -O3 -fopenmp smallpt.cpp -o smallpt
#include <cstdio>   //        Remove "-fopenmp" for g++ version < 4.2

#include <algorithm>
#include <array>
#include <fstream>
#include <memory>
#include <numbers>
#include <random>
#include <string>
#include <string_view>
#include <vector>
using namespace std::literals::string_literals;



#pragma region Math/Utility

// https://github.com/infancy/pbrt-v3/blob/master/src/core/pbrt.h

using Float = double;

using Radian = Float;
using Degree = Float;


constexpr Float Infinity = std::numeric_limits<Float>::infinity();
constexpr Float Pi = std::numbers::pi;


constexpr Radian radians(Degree deg) { return (Pi / 180) * deg; }
constexpr Degree degrees(Radian rad) { return (180 / Pi) * rad; }


#pragma endregion

#pragma region Geometry

// https://github.com/infancy/pbrt-v3/blob/master/src/core/geometry.h

struct Vector2
{
    Float x, y;

    Vector2(Float x = 0, Float y = 0) { this->x = x; this->y = y; }

    Float operator[](int index) const
    {
        if (index == 0) return x;
        else return y;
    }

    Vector2 operator+(const Vector2& vec2) const { return Vector2(x + vec2.x, y + vec2.y); }
    Vector2 operator-(const Vector2& vec2) const { return Vector2(x - vec2.x, y - vec2.y); }

    friend Vector2 operator*(Float scalar, Vector2 v) { return Vector2(v.x * scalar, v.y * scalar); }
};

using Float2 = Vector2;
using Point2 = Vector2;


struct Vector3
{ 
    union
    {
        struct { Float x, y, z; };
        struct { Float r, g, b; };
    };

    Vector3(Float x_ = 0, Float y_ = 0, Float z_ = 0)
    {
        x = x_;
        y = y_;
        z = z_;
    }

    Vector3 operator-() const { return Vector3(-x, -y, -z); }

    Vector3 operator+(const Vector3& b) const { return Vector3(x + b.x, y + b.y, z + b.z); }
    Vector3 operator-(const Vector3& b) const { return Vector3(x - b.x, y - b.y, z - b.z); }
    Vector3 operator*(Float b) const { return Vector3(x * b, y * b, z * b); }
    Vector3 operator/(Float b) const { return Vector3(x / b, y / b, z / b); }

    Vector3& Normalize() { return *this = *this * (1 / sqrt(x * x + y * y + z * z)); }

    Float Dot(const Vector3& b) const { return x * b.x + y * b.y + z * b.z; }
    Vector3 Cross(Vector3& b) { return Vector3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x); }

    friend Vector3 operator*(Float b, Vector3 v) { return v * b; }
    friend Float Dot(const Vector3& a, const Vector3& b) { return a.Dot(b); }

public:

    // only for Color
    Vector3 operator*(const Vector3& c) const { return Vector3(r * c.r, g * c.g, b * c.b); }
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
    Float distance; // distance from ray to intersection

    Ray(Point3 origin, UnitVector3 direction, Float distance = Infinity) :
        origin{ origin }, direction{ direction }, distance{ distance }
    {
    }

    Point3 operator()(Float t) const
    {
        return origin + t * direction;
    }
};



// https://github.com/infancy/pbrt-v3/blob/master/src/core/interaction.h

// TODO: remove
class Sphere;
enum class MaterialType
{
    Diffuse,
    Specular,
    Refract
}; // material types, used in radiance()

/*
  surface intersection, called `SurfaceInteraction` on pbrt

  prev   n   next
  ----   ^   ----
    ^    |    ^
     \   | ¦È /
   wo \  |  / wi is unknown, sampling for bsdf (or light)
       \ | /
        \|/
      -------
       isect
*/
class Isect
{
public:
    Isect() = default;
    Isect(const Point3& position, const Normal3& normal, UnitVector3 wo) :
        position{ position },
        normal{ normal },
        wo{ wo }
    {
    }

public:
    Point3 position{}; // world position of intersection
    Normal3 normal{};
    UnitVector3 wo{};

    //const bsdf_t* bsdf() const { return bsdf_.get(); }
    MaterialType materialType{}; // TODO: remove
    Color bsdfValue{}; // TODO: remove

    // prev <- isect, against ray's direction
    Color Le() const { return emission; }

private:
    //bsdf_uptr_t bsdf_{};
    Color emission{};

    // TODO: remove
    friend Sphere;
};

#pragma endregion



#pragma region Sampling


#pragma endregion

#pragma region Sampler

// https://github.com/mmp/pbrt-v3/blob/master/src/core/rng.h

// random number generator
// https://github.com/SmallVCM/SmallVCM/blob/master/src/rng.hxx
class RNG
{
public:
    RNG(int seed = 1234) : rngEngine(seed)
    {
    }

    // [0, int_max]
    int UniformInt()
    {
        return intDist(rngEngine);
    }

    // [0, uint_max]
    uint32_t UniformUint()
    {
        return uintDist(rngEngine);
    }

    // [0, 1)
    Float UniformFloat()
    {
        return float01Dist(rngEngine);
    }

    // [0, 1), [0, 1)
    Vector2 UniformFloat2()
    {
        return Vector2(UniformFloat(), UniformFloat());
    }

private:
    std::mt19937_64 rngEngine;

    std::uniform_int_distribution<int> intDist;
    std::uniform_int_distribution<uint32_t> uintDist;
    std::uniform_real_distribution<Float> float01Dist{ (Float)0, (Float)1 };
};


struct CameraSample
{
    Point2 pFilm{}; // sample point's position on film
    // Point2 pLens{};
};

// https://github.com/infancy/pbrt-v3/blob/master/src/core/sampler.h

class Sampler
{
public:
    virtual ~Sampler() {}
    Sampler(int samplesPerPixel) :
        samplesPerPixel{ samplesPerPixel }
    {
    }

    virtual int SamplesPerPixel()
    {
        return samplesPerPixel;
    }

    virtual std::unique_ptr<Sampler> Clone() = 0;

public:
    virtual void StartPixel()
    {
        currentSampleIndex = 0;
    }

    virtual bool StartNextSample()
    {
        currentSampleIndex += 1;
        return currentSampleIndex < samplesPerPixel;
    }

public:
    virtual Float Get1D() = 0;
    virtual Vector2 Get2D() = 0;
    virtual CameraSample GetCameraSample(Point2 pFilm) = 0;

protected:
    RNG rng{};

    int samplesPerPixel{};
    int currentSampleIndex{};
};

// https://github.com/mmp/pbrt-v3/blob/master/src/samplers/random.cpp
class RandomSampler : public Sampler
{
public:
    using Sampler::Sampler;

    std::unique_ptr<Sampler> Clone() override
    {
        return std::make_unique<RandomSampler>(samplesPerPixel);
    }

public:
    Float Get1D() override
    {
        return rng.UniformFloat();
    }

    Vector2 Get2D() override
    {
        return rng.UniformFloat2();
    }

    CameraSample GetCameraSample(Point2 pFilm) override
    {
        return { pFilm + rng.UniformFloat2() };
    }
};

// https://computergraphics.stackexchange.com/questions/3868/why-use-a-tent-filter-in-path-tracing
class TrapezoidalSampler : public Sampler
{
public:
    using Sampler::Sampler;

    int SamplesPerPixel() override
    {
        return samplesPerPixel * SubPixelNum;
    }

    std::unique_ptr<Sampler> Clone() override
    {
        return std::make_unique<TrapezoidalSampler>(samplesPerPixel);
    }

public:
    void StartPixel() override
    {
        Sampler::StartPixel();
        currentSubPixelIndex = 0;
    }

    bool StartNextSample() override
    {
        currentSampleIndex += 1;
        if (currentSampleIndex < samplesPerPixel)
        {
            return true;
        }
        else if (currentSampleIndex == samplesPerPixel)
        {
            currentSampleIndex = 0;
            currentSubPixelIndex += 1;

            return currentSubPixelIndex < SubPixelNum;
        }
        else
        {
            return false;
        }
    }

public:
    Float Get1D() override
    {
        return rng.UniformFloat();
    }

    Vector2 Get2D() override
    {
        return rng.UniformFloat2();
    }

    CameraSample GetCameraSample(Point2 pFilm) override
    {
        int subPixelX = currentSubPixelIndex % 2;
        int subPixelY = currentSubPixelIndex / 2;

        Float random1 = 2 * rng.UniformFloat();
        Float random2 = 2 * rng.UniformFloat();

        // uniform dist [0, 1) => triangle dist [-1, 1)
        Float deltaX = random1 < 1 ? sqrt(random1) - 1 : 1 - sqrt(2 - random1);
        Float deltaY = random2 < 1 ? sqrt(random2) - 1 : 1 - sqrt(2 - random2);

        Point2 samplePoint
        {
            (subPixelX + deltaX + 0.5) / 2,
            (subPixelY + deltaY + 0.5) / 2
        };

        return { pFilm + samplePoint };
    }

private:
    static constexpr int SubPixelNum = 4; // 2x2

    int currentSubPixelIndex{};
};

#pragma endregion



#pragma region Filter

// https://github.com/infancy/pbrt-v3/blob/master/src/core/filter.h

#pragma endregion

#pragma region Film

// https://github.com/infancy/pbrt-v3/blob/master/src/core/film.h

inline Float Clamp(Float x) { return x < 0 ? 0 : x > 1 ? 1 : x; }
inline Vector3 Clamp(Vector3 vec3) { return Vector3(Clamp(vec3.x), Clamp(vec3.y), Clamp(vec3.z)); }

inline int GammaEncoding(Float x) { return int(pow(Clamp(x), 1 / 2.2) * 255 + .5); }

/*
  warpper of `Color pixels[]`
  features:
    * get/set color
    * save image
*/
class Film
{
public:
    Film(const Vector2& resolution, /*std::unique_ptr<Filter> filter,*/ const std::string& filename) :
        fullResolution{ resolution },
        filename{ filename },
        pixels{ std::make_unique<Color[]>(Width() * Height()) }
    {
    }

public:
    int  Width() const { return (int)fullResolution.x; }
    int Height() const { return (int)fullResolution.y; }
    Vector2 Resolution() const { return fullResolution; }

    Color& operator()(int x, int y)
    {
        return *(pixels.get() + Width() * y + x);
    }

    void add_color(int x, int y, const Color& delta)
    {
        Color& color_ = operator()(x, y);
        color_ = color_ + delta;
    }

public:
    virtual bool store_image() const
    {
        return store_bmp_impl(filename, Width(), Height(), 3, (Float*)pixels.get());
    }

    // https://github.com/SmallVCM/SmallVCM/blob/master/src/framebuffer.hxx#L149-L215
    static bool store_bmp_impl(const std::string& filename, int width, int height, int channel, const Float* floats)
    {
        std::fstream img_file(filename, std::ios::binary | std::ios::out);


        uint32_t padding_line_bytes = (width * channel + 3) & (~3);
        uint32_t padding_image_bytes = padding_line_bytes * height;

        const uint32_t FILE_HEADER_SIZE = 14;
        const uint32_t INFO_HEADER_SIZE = 40;

        // write file header
        struct BITMAP_FILE_HEADER_INFO_HEADER
        {
            // file header
            //char8_t type[2]{ 'B', 'M' };
            uint32_t file_size{};
            uint32_t reserved{ 0 };
            uint32_t databody_offset{ FILE_HEADER_SIZE + INFO_HEADER_SIZE };

            // info header
            uint32_t	info_header_size{ INFO_HEADER_SIZE };

            int32_t     width{};
            int32_t		height{};
            int16_t	    color_planes{ 1 };
            int16_t	    per_pixel_bits{};
            uint32_t	compression{ 0 };
            uint32_t	image_bytes{ 0 };

            uint32_t	x_pixels_per_meter{ 0 };
            uint32_t	y_pixels_per_meter{ 0 };
            uint32_t	color_used{ 0 };
            uint32_t	color_important{ 0 };
        }
        bmp_header
        {
            .file_size{ FILE_HEADER_SIZE + INFO_HEADER_SIZE + padding_image_bytes },
            .width{ width },
            .height{ height },
            .per_pixel_bits{ (int16_t)(channel * 8) },
            //.image_bytes{ padding_image_bytes }
        };

        img_file
            .write("BM", 2)
            .write((char*)&bmp_header, sizeof(bmp_header));


        // without color table


        // gamma encoding
        int byte_num = width * height * channel;
        auto bytes = std::make_unique<uint8_t[]>(byte_num);
        for (int i = 0; i < byte_num; i += 3)
        {
            // BGR
            bytes[i]     = GammaEncoding(floats[i + 2]);
            bytes[i + 1] = GammaEncoding(floats[i + 1]);
            bytes[i + 2] = GammaEncoding(floats[i]);
        }

        // write data body 
        int line_num = width * channel;
        // bmp is stored from bottom to up
        for (int y = height - 1; y >= 0; --y)
            img_file.write((char*)(bytes.get() + y * line_num), line_num);


        return true;
    }

private:
    const Vector2 fullResolution;
    //std::unique_ptr<Filter> filter;
    const std::string filename;

    std::unique_ptr<Color[]> pixels;
};

#pragma endregion

#pragma region Camera

// https://github.com/infancy/pbrt-v3/blob/master/src/core/camera.h

/*
  pbrt camera space:
    left hand

  y (0, 1, 0)         z(0, 0, 1)
        |            /
        |          /
        |        /
        |      /
        |    /
        |  /
        |/_ _ _ _ _ _ x(1, 0, 0)
        o

  features:
    generate ray
*/
class Camera
{
public:
    virtual ~Camera() {}
    Camera() {}

public:
    virtual Ray GenerateRay(const CameraSample& sample) const = 0;
};

// https://github.com/infancy/pbrt-v3/blob/master/src/cameras/perspective.cpp
class PerspectiveCamera : public Camera
{
public:
    PerspectiveCamera(const Vector3& position, const UnitVector3& direction, const Vector3& up,
        Degree fov, Vector2 resolution) :
        position{ position },
        front{ direction },
        up{ up },
        resolution{ resolution }
    {
        // `front` is a unit vector, it's length is 1
        Float tan_fov = std::tan(radians(fov) / 2);

        right = this->up.Cross(front).Normalize() * tan_fov * Aspect();
        this->up = front.Cross(right).Normalize() * tan_fov;
    }

public:
    virtual Ray GenerateRay(const CameraSample& sample) const
    {
        Vector3 direction =
            front +
            right * (sample.pFilm.x / resolution.x - 0.5) +
            up * (0.5 - sample.pFilm.y / resolution.y);

        return Ray{ position + direction * 140, direction.Normalize() };
    }

private:
    Float Aspect() { return resolution.x / resolution.y; }

private:
    Vector3 position;
    UnitVector3 front;
    UnitVector3 right;
    UnitVector3 up;

    Vector2 resolution;
};

#pragma endregion



#pragma region Shape

// https://github.com/infancy/pbrt-v3/blob/master/src/core/shape.h


// https://github.com/infancy/pbrt-v3/blob/master/src/shapes/sphere.cpp
struct Sphere
{
    Float radius;
    Point3 center;

    // TODO: remove
    Color emission; // for area light
    MaterialType materialType;
    Color color; // surface reflectance

    Sphere(Float radius_, Vector3 center_, Color emission_, Color color_, MaterialType materialType) :
        radius(radius_), center(center_), emission(emission_), color(color_), materialType(materialType) {}

    bool Intersect(Ray& ray, Isect* isect) const
    {
        /*
          ray: p(t) = o + t*d,
          sphere: ||p - c||^2 = r^2

          if ray and sphere have a intersection p, then:
             ||p(t) - c||^2 = r^2
          => ||o + t*d - c||^2 = r^2
          => (t*d + o - c).(t*d + o - c) = r^2
          => d.d*t^2 + 2d.(o-c)*t + (o-c).(o-c)-r^2 = 0

          compare with:
             at^2 + bt + c = 0

          there have:
             co = o - c
             a = dot(d, d) = 1;
             b = 2 * dot(d, co), neg_b' = dot(d, oc);
             c = dot(co, co) - r^2;

          so:
             t = (-b +/- sqrt(b^2 - 4ac)) / 2a
               = (-b +/- sqrt(b^2 - 4c)) / 2
               = ((-2 * dot(d, co) +/- sqrt(4 * dot(d, co)^2 - 4 * (dot(co, co) - r^2))) / 2
               = -dot(d, co) +/- sqrt( dot(d, co)^2 - dot(co, co) + r^2 )
               = neg_b' +/- sqrt(Delta)
        */
        Vector3 oc = center - ray.origin;
        Float neg_b = oc.Dot(ray.direction);
        Float det = neg_b * neg_b - oc.Dot(oc) + radius * radius;

        bool hit = false;
        Float distance = 0;
        if (det >= 0)
        {
            Float sqrtDet = sqrt(det);

            Float epsilon = 1e-4;
            if (distance = neg_b - sqrtDet; distance > epsilon && distance < ray.distance)
            {
                hit = true;
            }
            else if (distance = neg_b + sqrtDet; distance > epsilon && distance < ray.distance)
            {
                hit = true;
            }
        }

        if (hit)
        {
            ray.distance = distance;

            Point3 hit_point = ray(distance);
            *isect = Isect(hit_point, (hit_point - center).Normalize(), -ray.direction);

            // TODO: remove
            isect->materialType = materialType;
            isect->bsdfValue = color;
            isect->emission = emission;
        }

        return hit;
    }
};

#pragma endregion



#pragma region BSDF


#pragma endregion

#pragma region Texture


#pragma endregion

#pragma region Material


#pragma endregion



#pragma region Light

#pragma endregion



#pragma region Sureface(Primitive)

#pragma endregion

#pragma region Accelerator

#pragma endregion

#pragma region Scene

Sphere Scene[] =
{
    //Scene: radius, center, emission, color, material
    Sphere(1e5, Vector3(1e5 + 1, 40.8, -81.6),   Color(), Color(.75, .25, .25), MaterialType::Diffuse), //Left
    Sphere(1e5, Vector3(-1e5 + 99, 40.8, -81.6), Color(), Color(.25, .25, .75), MaterialType::Diffuse), //Right
    Sphere(1e5, Vector3(50, 40.8, -1e5),         Color(), Color(.75, .75, .75), MaterialType::Diffuse), //Back
    Sphere(1e5, Vector3(50, 40.8, 1e5 - 170),    Color(), Color(),              MaterialType::Diffuse), //Front
    Sphere(1e5, Vector3(50, 1e5, -81.6),         Color(), Color(.75, .75, .75), MaterialType::Diffuse), //Bottom
    Sphere(1e5, Vector3(50, -1e5 + 81.6, -81.6), Color(), Color(.75, .75, .75), MaterialType::Diffuse), //Top

    Sphere(16.5, Vector3(27, 16.5, -47),          Color(), Color(1, 1, 1) * .999, MaterialType::Specular), //Mirror
    Sphere(16.5, Vector3(73, 16.5, -78),          Color(), Color(1, 1, 1) * .999, MaterialType::Refract),  //Glass
    Sphere(600,  Vector3(50, 681.6 - .27, -81.6), Color(12, 12, 12), Color(),     MaterialType::Diffuse)   //Light
};

inline Float Lerp(Float a, Float b, Float t) { return a + t * (b - a); }

inline bool Intersect(Ray& ray, Isect* isect)
{
    bool bHit = false;

    int sphereNum = sizeof(Scene) / sizeof(Sphere);
    for (int i = sphereNum; i--;)
    {
        if (Scene[i].Intersect(ray, isect))
        {
            bHit = true;
        }
    }

    return bHit;
}

#pragma endregion



#pragma region Integrater

Color Radiance(Ray ray, int depth, Sampler& sampler)
{
    Isect isect;
    if (!Intersect(ray, &isect))
        return Color(); // if miss, return black

    if (depth > 100)
        return isect.Le();

    // intersection property
    Vector3 position = isect.position;
    Normal3 normal = isect.normal;
    Normal3 shading_normal = normal.Dot(ray.direction) < 0 ? normal : normal * -1;

    Color f = isect.bsdfValue; // bsdf value
    Float max_component = f.x > f.y && f.x > f.z ? f.x : f.y > f.z ? f.y : f.z; // max refl

    //russian roulette
    if (++depth > 5)
    {
        if (sampler.Get1D() < max_component)
            f = f * (1 / max_component);
        else
            return isect.Le();
    }

    if (isect.materialType == MaterialType::Diffuse) // Ideal Diffuse reflection
    {
        Float random1 = 2 * Pi * sampler.Get1D();
        Float random2 = sampler.Get1D();
        Float random2Sqrt = sqrt(random2);

        // shading coordinate on intersection
        Vector3 w = shading_normal;
        Vector3 u = ((fabs(w.x) > .1 ? Vector3(0, 1, 0) : Vector3(1, 0, 0)).Cross(w)).Normalize();
        Vector3 v = w.Cross(u);

        // Cosine importance sampling of the hemisphere for diffuse reflection
        Vector3 direction = (u * cos(random1) * random2Sqrt + v * sin(random1) * random2Sqrt + w * sqrt(1 - random2)).Normalize();

        f = f / Pi; // for lambert brdf, f = R / Pi;
        Float abs_cos_theta = std::abs(shading_normal.Dot(direction));
        Float pdf = abs_cos_theta / Pi; // cosine-weighted sampling
        return isect.Le() + (f * Radiance(Ray(position, direction), depth, sampler) * abs_cos_theta) / pdf;
    }
    else if (isect.materialType == MaterialType::Specular) // Ideal Specular reflection
    {
        Vector3 direction = ray.direction - normal * 2 * normal.Dot(ray.direction);
        return isect.Le() + f * Radiance(Ray(position, direction), depth, sampler);
    }
    else // Ideal Dielectric Refraction
    {
        bool into = normal.Dot(shading_normal) > 0; // Ray from outside going in?

        // IOR(index of refractive)
        Float etaI = 1; // vacuum
        Float etaT = 1.5; // glass
        Float eta = into ? etaI / etaT : etaT / etaI;


        // compute reflect direction by refection law
        Ray reflectRay(position, ray.direction - normal * 2 * normal.Dot(ray.direction));

        // compute refract direction by Snell's law
        Float cosThetaI = ray.direction.Dot(shading_normal);
        Float cosThetaT2 = cosThetaT2 = 1 - eta * eta * (1 - cosThetaI * cosThetaI);
        if (cosThetaT2 < 0) // Total internal reflection
            return isect.Le() + f * Radiance(reflectRay, depth, sampler);

        // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#SpecularTransmission see `Refract()`
        Vector3 refractDirection = (ray.direction * eta - normal * ((into ? 1 : -1) * (cosThetaI * eta + sqrt(cosThetaT2)))).Normalize();


        // compute the fraction of incoming light that is reflected or transmitted
        // by Schlick Approximation of Fresnel Dielectric 1994 https://en.wikipedia.org/wiki/Schlick%27s_approximation
        Float a = etaT - etaI;
        Float b = etaT + etaI;
        Float R0 = a * a / (b * b);
        Float c = 1 - (into ? -cosThetaI : refractDirection.Dot(normal));

        Float Re = R0 + (1 - R0) * c * c * c * c * c;
        Float Tr = 1 - Re;


        // probability of reflected or transmitted
        Float P = .25 + .5 * Re;
        Float RP = Re / P;
        Float TP = Tr / (1 - P);

        Color Li;
        if (depth > 2)
        {
            // Russian roulette
            if (sampler.Get1D() < P)
            {
                Li = Radiance(reflectRay, depth, sampler) * RP;
            }
            else
            {
                Li = Radiance(Ray(position, refractDirection), depth, sampler) * TP;
            }
        }
        else
        {
            Li = Radiance(reflectRay, depth, sampler) * Re + Radiance(Ray(position, refractDirection), depth, sampler) * Tr;
        }

        return isect.Le() + f * Li;
    }
}

#pragma endregion



int main(int argc, char* argv[])
{
    int width = 256, height = 256;

    Film film({ (Float)width, (Float)height }, "image.bmp"s);

    int samplesPerPixel = argc == 2 ? atoi(argv[1]) / 4 : 100;
    std::unique_ptr<Sampler> originalSampler = std::make_unique<RandomSampler>(samplesPerPixel);

    std::unique_ptr<Camera> camera = std::make_unique<PerspectiveCamera>(
        Vector3{ 50, 52, -295.6 }, Vector3{ 0, -0.042612, 1 }.Normalize(), Vector3{ 0, 1, 0 }, 53, film.Resolution());

#pragma omp parallel for schedule(dynamic, 1) // OpenMP
    for (int y = 0; y < height; y++) // Loop over image rows
    {
        std::unique_ptr<Sampler> sampler = originalSampler->Clone();
        fprintf(stderr, "\rRendering (%d spp) %5.2f%%", sampler->SamplesPerPixel(), 100. * y / (height - 1));

        for (int x = 0; x < width; x++) // Loop cols
        {
            Color Li{};

            sampler->StartPixel();
            do
            {
                auto cameraSample = sampler->GetCameraSample({ (Float)x, (Float)y });
                auto ray = camera->GenerateRay(cameraSample);

                Li = Li + Radiance(ray, 0, *sampler) * (1. / sampler->SamplesPerPixel());
            }
            while (sampler->StartNextSample());

            film.add_color(x, y, Clamp(Li));
        }
    }

    film.store_image();
#if defined(_WIN32) || defined(_WIN64)
    system("mspaint image.bmp");
#endif

    return 0;
}