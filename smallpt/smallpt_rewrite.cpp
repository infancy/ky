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
constexpr Float InvPi = std::numbers::inv_pi;


constexpr Radian radians(Degree deg) { return (Pi / 180) * deg; }
constexpr Degree degrees(Radian rad) { return (180 / Pi) * rad; }


#pragma endregion

#pragma region Geometry

// https://www.pbr-book.org/3ed-2018/Geometry_and_Transformations
// https://github.com/infancy/pbrt-v3/blob/master/src/core/geometry.h

struct Vector2
{
    Float x{}, y{};

    Vector2() = default;
    Vector2(Float x, Float y) : x{ x }, y{ y } {}

    Float operator[](int index) const
    {
        if (index == 0) return x;
        else return y;
    }

    Vector2 operator+(const Vector2& b) const { return Vector2(x + b.x, y + b.y); }
    Vector2 operator-(const Vector2& b) const { return Vector2(x - b.x, y - b.y); }

    friend Vector2 operator*(Float a, Vector2 b) { return Vector2(a * b.x, a * b.y); }
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

    Vector3() : x{ 0 }, y{ 0 }, z{ 0 } {}
    Vector3(Float x, Float y, Float z) : x{ x }, y{ y }, z{ z } {}

    Vector3 operator-() const { return Vector3(-x, -y, -z); }

    Vector3 operator+(const Vector3& b) const { return Vector3(x + b.x, y + b.y, z + b.z); }
    Vector3 operator-(const Vector3& b) const { return Vector3(x - b.x, y - b.y, z - b.z); }
    Vector3 operator*(Float b) const { return Vector3(x * b, y * b, z * b); }
    Vector3 operator/(Float b) const { return Vector3(x / b, y / b, z / b); }

    Vector3 Normalize() const { return *this * (1 / sqrt(x * x + y * y + z * z)); }
    Float Dot(const Vector3& b) const { return x * b.x + y * b.y + z * b.z; }
    Vector3 Cross(const Vector3& b) const { return Vector3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x); }

    friend Vector3 operator*(Float a, Vector3 v) { return v * a; }
    friend Vector3 Normalize(const Vector3& a) { return a.Normalize(); }
    friend Float Dot(const Vector3& a, const Vector3& b) { return a.Dot(b); }
    friend Float AbsDot(const Vector3& a, const Vector3& b) { return std::abs(a.Dot(b)); }
    friend Vector3 Cross(const Vector3& a, const Vector3& b) { return a.Cross(b); }

public:
    // only for Color
    Vector3 operator*(const Vector3& c) const { return Vector3(r * c.r, g * c.g, b * c.b); }

    Float MaxComponentValue() const
    {
        return std::max({ r, g, b });
    }

    bool IsBlack() const { return (r <= 0) && (g <= 0) && (b <= 0); }
};

using Float3 = Vector3;
using Point3 = Vector3;
using Normal3 = Vector3;
using UnitVector3 = Vector3;

// https://www.pbr-book.org/3ed-2018/Color_and_Radiometry/RGBSpectrum_Implementation 
using Color = Vector3;



// https://github.com/SmallVCM/SmallVCM/blob/master/src/frame.hxx
class Frame
{
public:
    Frame(const Vector3& s, const Vector3& t, const Normal3& n) :
        s_{ s.Normalize() },
        t_{ t.Normalize() },
        n_{ n.Normalize() }
    {
    }

    Frame(const Normal3& n) :
        n_{ n.Normalize() }
    {
        SetFromZ();
    }

public:
    // think if {s, t, n} is (1, 0, 0), (0, 1, 0), (0, 0, 1)
    Vector3 ToLocal(const Vector3& worldVec3) const
    {
        return Vector3(
            Dot(s_, worldVec3),
            Dot(t_, worldVec3),
            Dot(n_, worldVec3));
    }

    Vector3 ToWorld(const Vector3& localVec3) const
    {
        return
            s_ * localVec3.x +
            t_ * localVec3.y +
            n_ * localVec3.z;
    }

    const Vector3& Binormal() const { return s_; }
    const Vector3& Tangent() const { return t_; }
    const Vector3& Normal() const { return n_; }

private:
    void SetFromZ()
    {
        Vector3 tmp_s = (std::abs(n_.x) > 0.99f) ? Vector3(0, 1, 0) : Vector3(1, 0, 0);
        t_ = Normalize(Cross(n_, tmp_s));
        s_ = Normalize(Cross(t_, n_));
    }

private:
    // world frame basic vector
    Vector3 s_{ 1, 0, 0 }; // x
    Vector3 t_{ 0, 1, 0 }; // y
    Normal3 n_{ 0, 0, 1 }; // z
};


// https://www.pbr-book.org/3ed-2018/Geometry_and_Transformations/Rays

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


// https://www.pbr-book.org/3ed-2018/Geometry_and_Transformations/Interactions
// https://github.com/infancy/pbrt-v3/blob/master/src/core/interaction.h

class BSDF;
class Primitive;

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

    const BSDF* bsdf() const { return bsdfPtr.get(); } 
    Color Le() const { return emission; } // prev <- isect, against ray's direction

private:
    std::unique_ptr<BSDF> bsdfPtr{};
    Color emission{};

    friend Primitive;
};

#pragma endregion



#pragma region Sampling

// https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations
// https://github.com/infancy/pbrt-v3/blob/master/src/core/sampling.h#L138-L153
// https://github.com/infancy/pbrt-v3/blob/master/src/core/sampling.cpp#L199-L230

Point2 UniformSampleDisk(const Point2& random)
{
    Float radius = std::sqrt(random[0]);
    Float theta = 2 * Pi * random[1];
    return Point2(radius * std::cos(theta), radius * std::sin(theta));
}

// cosine-weighted sampling
inline Vector3 CosineSampleHemisphere(const Float2& random)
{
    // Cosine importance sampling of the hemisphere for diffuse reflection
    Point2 pDisk = UniformSampleDisk(random);
    Float z = std::sqrt(std::max((Float)0, 1 - pDisk.x * pDisk.x - pDisk.y * pDisk.y));
    return Vector3(pDisk.x, pDisk.y, z);
}

inline Float CosineHemispherePdf(Float cosTheta)
{
    return cosTheta * InvPi;
}

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

// https://www.pbr-book.org/3ed-2018/Camera_Models
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

// https://www.pbr-book.org/3ed-2018/Shapes
// https://github.com/infancy/pbrt-v3/blob/master/src/core/shape.h

class Shape
{
public:
    virtual bool Intersect(Ray& ray, Isect* isect) const = 0;
};

class Sphere : public Shape
{
public:
    Sphere(Float radius, Vector3 center) :
        radius(radius), center(center)
    {
    }

public:
    bool Intersect(Ray& ray, Isect* isect) const override
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
        }

        return hit;
    }

private:
    Float radius;
    Point3 center;
};

#pragma endregion



#pragma region BSDF

// https://www.pbr-book.org/3ed-2018/Reflection_Models
// https://github.com/mmp/pbrt-v3/blob/master/src/core/reflection.h

// local shading coordinate
inline Float cosTheta(const Vector3& w) { return w.z; }
inline Float AbsCosTheta(const Vector3& w) { return std::abs(w.z); }
inline bool SameHemisphere(const Vector3& w, const Vector3& wp) { return w.z * wp.z > 0; }

struct BSDFSample
{
    Color f; // scattering rate 
    Vector3 wi; // world wi
    Float pdf{};
};

/*
  https://www.pbr-book.org/3ed-2018/Reflection_Models#x0-GeometricSetting


  shading frame:

       z, n(0, 0, 1)
       |
       |
       |
       |
       |_ _ _ _ _ _ x, s(1, 0, 0)
      / p
     /
    /
   y, t(0, 1, 0)
*/

class BSDF
{
public:
    virtual ~BSDF() = default;
    BSDF(Frame shadingFrame) :
        shadingFrame{ shadingFrame }
    {
    }

public:
    // or called `eval`, `evaluate`
    Color f(const Vector3& world_wo, const Vector3& world_wi) const
    {
        return f_(ToLocal(world_wo), ToLocal(world_wi));
    }

    Float Pdf(const Vector3& world_wo, const Vector3& world_wi) const
    {
        return Pdf_(ToLocal(world_wo), ToLocal(world_wi));
    }

    // or called `sample`, `sample_direction`
    BSDFSample Sample_f(const Vector3& world_wo, const Float2& random) const
    {
        auto sample = Sample_f_(ToLocal(world_wo), random);
        sample.wi = ToWorld(sample.wi);

        return sample;
    }

protected:
    virtual Color f_(const Vector3& wo, const Vector3& wi) const = 0;
    virtual Float Pdf_(const Vector3& wo, const Vector3& wi) const = 0;

    virtual BSDFSample Sample_f_(const Vector3& wo, const Float2& random) const = 0;

private:
    Vector3 ToLocal(const Vector3& worldVec3) const
    {
        return shadingFrame.ToLocal(worldVec3);
    }

    Vector3 ToWorld(const Vector3& localVec3) const
    {
        return shadingFrame.ToWorld(localVec3);
    }

private:
    Frame shadingFrame;

    // extension point:
    // std::array<bxdf_uptr, 2> BxDFList;
};

class LambertionReflection : public BSDF
{
public:
    LambertionReflection(const Frame& shadingFrame, const Color& R) :
        BSDF(shadingFrame), R{ R }
    {
    }

    Color f_(const Vector3& wo, const Vector3& wi) const override { return R * InvPi; }

    Float Pdf_(const Vector3& wo, const Vector3& wi) const override
    {
        return SameHemisphere(wo, wi) ? CosineHemispherePdf(AbsCosTheta(wi)) : 0;
    }

    BSDFSample Sample_f_(const Vector3& wo, const Float2& random) const override
    {
        BSDFSample sample;

        // Cosine-sample the hemisphere, flipping the direction if necessary
        sample.wi = CosineSampleHemisphere(random);
        if (wo.z < 0)
            sample.wi.z *= -1;

        sample.pdf = Pdf_(wo, sample.wi);
        sample.f = f_(wo, sample.wi);

        return sample;
    }

private:
    Color R; // surface reflectance
};

class SpecularReflection : public BSDF
{
public:
    SpecularReflection(const Frame& shadingFrame, const Color& R) :
        BSDF(shadingFrame), R{ R }
    {
    }

    Color f_(const Vector3& wo, const Vector3& wi) const override { return Color(); }
    Float Pdf_(const Vector3& wo, const Vector3& wi) const override { return 0; }

    BSDFSample Sample_f_(const Vector3& wo, const Float2& random) const override
    {
        // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#SpecularReflection
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.h#L387-L408
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.cpp#L181-L191

        BSDFSample sample; 
        sample.wi = Vector3(-wo.x, -wo.y, wo.z);
        sample.pdf = 1;
        sample.f = R / AbsCosTheta(sample.wi); // for `(R / cos_theta) * Li * cos_theta / pdf = R * Li`

        return sample;
    }

private:
    Color R;
};

class FresnelSpecular : public BSDF
{
public:
    FresnelSpecular(const Frame& shadingFrame, const Color& R, const Color& T, Float etaI, Float etaT) :
        BSDF(shadingFrame), R{ R }, T{ T }, etaI{ etaI }, etaT{ etaT }
    {
    }

    Color f_(const Vector3& wo, const Vector3& wi) const override { return Color(); }
    Float Pdf_(const Vector3& wo, const Vector3& wi) const override { return 0; }

    BSDFSample Sample_f_(const Vector3& wo, const Float2& random) const override
    {
        // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#FresnelReflectance
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.h#L440-L463
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.cpp#L627-L667

        BSDFSample sample;

        Normal3 normal(0, 0, 1); // use `z` as normal
        bool into = normal.Dot(wo) > 0; // ray from outside going in?

        Normal3 woNormal = into ? normal : normal * -1;
        // IOR(index of refractive)
        Float eta = into ? etaI / etaT : etaT / etaI;


        // compute reflect direction by refection law
        Vector3 reflectDirection = Vector3(-wo.x, -wo.y, wo.z);

        // compute refract direction by Snell's law
        // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#SpecularTransmission see `Refract()`
        Float cosThetaI = Dot(wo, woNormal);
        Float cosThetaT2 = 1 - eta * eta * (1 - cosThetaI * cosThetaI);
        if (cosThetaT2 < 0) // Total internal reflection
        {
            return sample;
        }
        Float cosThetaT = sqrt(cosThetaT2);
        Vector3 refractDirection = (-wo * eta + woNormal * (cosThetaI * eta - cosThetaT)).Normalize();


        // compute the fraction of incoming light that is reflected or transmitted
        // by Schlick Approximation of Fresnel Dielectric 1994 https://en.wikipedia.org/wiki/Schlick%27s_approximation
        Float a = etaT - etaI;
        Float b = etaT + etaI;
        Float R0 = a * a / (b * b);
        Float c = 1 - (into ? cosThetaI : cosThetaT);

        Float Re = R0 + (1 - R0) * c * c * c * c * c;
        Float Tr = 1 - Re;


        if (random[0] < Re) // Russian roulette
        {
            // Compute specular reflection for _FresnelSpecular_

            sample.wi = reflectDirection;
            sample.pdf = Re;
            sample.f = (R * Re) / AbsCosTheta(sample.wi);
        }
        else
        {
            // Compute specular transmission for _FresnelSpecular_

            sample.wi = refractDirection;
            sample.pdf = Tr;
            sample.f = (T * Tr) / AbsCosTheta(sample.wi);
        }

        return sample;
    }


private:
    Color R;
    Color T;
    Float etaI;
    Float etaT;
};

#pragma endregion

#pragma region Texture

// https://www.pbr-book.org/3ed-2018/Texture
// https://github.com/mmp/pbrt-v3/blob/master/src/core/texture.h

#pragma endregion

#pragma region Material

// https://www.pbr-book.org/3ed-2018/Materials
// https://github.com/mmp/pbrt-v3/blob/master/src/core/material.h

class Material
{
public:
    virtual ~Material() = default;

    virtual std::unique_ptr<BSDF> Scattering(const Isect& isect) const = 0;
};

class MatteMaterial : public Material
{
public:
    MatteMaterial(const Color Kd) :
        Kd{ Kd }
    {
    }

    std::unique_ptr<BSDF> Scattering(const Isect& isect) const override
    {
        return std::make_unique<LambertionReflection>(Frame(isect.normal), Kd);
    }

private:
    Color Kd;
};

class MirrorMaterial : public Material
{
public:
    MirrorMaterial(const Color Kr) :
        Kr{ Kr }
    {
    }

    std::unique_ptr<BSDF> Scattering(const Isect& isect) const override
    {
        return std::make_unique<SpecularReflection>(Frame(isect.normal), Kr);
    }

private:
    Color Kr;
};

class GlassMaterial : public Material
{
public:
    GlassMaterial(const Color& Kr, const Color& Kt, Float eta) :
        Kr{ Kr }, Kt{ Kt }, eta{ eta }
    {
    }

    std::unique_ptr<BSDF> Scattering(const Isect& isect) const override
    {
        return std::make_unique<FresnelSpecular>(Frame(isect.normal), Kr, Kt, 1, eta);
    }

private:
    Color Kr;
    Color Kt;
    Float eta;
};

#pragma endregion



#pragma region Light

// https://www.pbr-book.org/3ed-2018/Light_Sources
// https://github.com/mmp/pbrt-v3/blob/master/src/core/light.h

class Light
{
};

class AreaLight : public Light
{
public:
    AreaLight(Color radiance, const Shape* shape) :
        radiance{ radiance },
        shape{ shape }
    {
    }

    Color Le(const Isect& lightIsect, const Vector3& wo) const
    {
        return (Dot(lightIsect.normal, wo) > 0) ? radiance : Color();
    }

private:
    Color radiance;
    const Shape* shape;
};

#pragma endregion



#pragma region Primitive

// https://www.pbr-book.org/3ed-2018/Primitives_and_Intersection_Acceleration
// https://github.com/infancy/pbrt-v3/blob/master/src/core/primitive.h

struct Primitive
{
    const Shape* shape;
    const Material* material;
    const AreaLight* areaLight;

    bool Intersect(Ray& ray, Isect* isect) const
    {
        bool hit = shape->Intersect(ray, isect);
        if (hit)
        {
            isect->bsdfPtr = material->Scattering(*isect);
            isect->emission = areaLight ? areaLight->Le(*isect, isect->wo) : Color();
        }

        return hit;
    }
};

#pragma endregion

#pragma region Accelerator

// https://www.pbr-book.org/3ed-2018/Primitives_and_Intersection_Acceleration/Aggregates

// https://github.com/infancy/pbrt-v3/blob/master/src/core/primitive.h#L158-L183
    // https://github.com/infancy/pbrt-v3/blob/master/src/accelerators/bvh.h
    // https://github.com/infancy/pbrt-v3/blob/master/src/accelerators/kdtreeaccel.h

#pragma endregion

#pragma region Scene

// https://www.pbr-book.org/3ed-2018/Scene_Description_Interface
// https://github.com/infancy/pbrt-v3/blob/master/src/core/scene.h

class Scene
{
public:
    Scene() = default;
    Scene(
        std::vector<std::shared_ptr<Shape>> shapeList, std::vector<std::shared_ptr<Material>> materialList, std::vector<std::shared_ptr<Light>> lightList, std::vector<Primitive> primitiveList) :
        shapeList{ shapeList },
        materialList{ materialList },
        lightList{ lightList },
        primitiveList{ primitiveList }
    {
    }

public:
    bool Intersect(Ray& ray, Isect* isect) const
    {
        bool bHit = false;

        for (const auto& primitive : primitiveList)
        {
            if (primitive.Intersect(ray, isect))
            {
                bHit = true;
            }
        }

        return bHit;
    }

public:
    static Scene CreateSmallptScene()
    {
        std::shared_ptr<Shape> left   = std::make_shared<Sphere>(1e5, Vector3(1e5 + 1, 40.8, -81.6));
        std::shared_ptr<Shape> right  = std::make_shared<Sphere>(1e5, Vector3(-1e5 + 99, 40.8, -81.6));
        std::shared_ptr<Shape> back   = std::make_shared<Sphere>(1e5, Vector3(50, 40.8, -1e5));
        std::shared_ptr<Shape> front  = std::make_shared<Sphere>(1e5, Vector3(50, 40.8, 1e5 - 170));
        std::shared_ptr<Shape> bottom = std::make_shared<Sphere>(1e5, Vector3(50, 1e5, -81.6));
        std::shared_ptr<Shape> top    = std::make_shared<Sphere>(1e5, Vector3(50, -1e5 + 81.6, -81.6));

        std::shared_ptr<Shape> mirror = std::make_shared<Sphere>(16.5, Vector3(27, 16.5, -47));
        std::shared_ptr<Shape> glass  = std::make_shared<Sphere>(16.5, Vector3(73, 16.5, -78));
        std::shared_ptr<Shape> light  = std::make_shared<Sphere>(600, Vector3(50, 681.6 - .27, -81.6));
        std::vector<std::shared_ptr<Shape>> shapeList{ left, right, back, front, bottom, top, mirror, glass, light };


        std::shared_ptr<Material> red   = std::make_shared<MatteMaterial>(Color(.75, .25, .25));
        std::shared_ptr<Material> blue  = std::make_shared<MatteMaterial>(Color(.25, .25, .75));
        std::shared_ptr<Material> gray  = std::make_shared<MatteMaterial>(Color(.75, .75, .75));
        std::shared_ptr<Material> black = std::make_shared<MatteMaterial>(Color());

        std::shared_ptr<Material> mirror_mat = std::make_shared<MirrorMaterial>(Color(1, 1, 1) * 0.999);
        std::shared_ptr<Material> glass_mat  = std::make_shared<GlassMaterial>(Color(1, 1, 1) * 0.999, Color(1, 1, 1) * 0.999, 1.5);
        std::vector<std::shared_ptr<Material>> materialList{ red, blue, gray, black, mirror_mat, glass_mat };


        std::shared_ptr<AreaLight> area_light = std::make_shared<AreaLight>(Color(12, 12, 12), light.get());
        std::vector<std::shared_ptr<Light>> lightList{ area_light };


        std::vector<Primitive> primitiveList
        {
            {   left.get(),   red.get(), nullptr},
            {  right.get(),  blue.get(), nullptr },
            {   back.get(),  gray.get(), nullptr },
            {  front.get(), black.get(), nullptr },
            { bottom.get(),  gray.get(), nullptr },
            {    top.get(),  gray.get(), nullptr },

            { mirror.get(), mirror_mat.get(), nullptr },
            {  glass.get(),  glass_mat.get(), nullptr },

            {  light.get(), black.get(), area_light.get() },
        };

        return Scene{ shapeList, materialList, lightList, primitiveList };
    }

private:
    std::vector<std::shared_ptr<Shape>> shapeList;
    std::vector<std::shared_ptr<Material>> materialList;
    std::vector<std::shared_ptr<Light>> lightList;

    std::vector<Primitive> primitiveList;
};


#pragma endregion



#pragma region Integrater

// https://www.pbr-book.org/3ed-2018/Color_and_Radiometry
    // https://www.pbr-book.org/3ed-2018/Color_and_Radiometry/Radiometry
    // https://www.pbr-book.org/3ed-2018/Color_and_Radiometry/Working_with_Radiometric_Integrals
    // https://www.pbr-book.org/3ed-2018/Color_and_Radiometry/Surface_Reflection

// https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration
    // https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/The_Monte_Carlo_Estimator
    // https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/Russian_Roulette_and_Splitting 
    // https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/Importance_Sampling

// https://www.pbr-book.org/3ed-2018/Light_Transport_I_Surface_Reflection
    // https://www.pbr-book.org/3ed-2018/Light_Transport_I_Surface_Reflection/Sampling_Reflection_Functions

// https://github.com/infancy/pbrt-v3/blob/master/src/core/integrator.h
    // https://github.com/infancy/pbrt-v3/blob/master/src/integrators/directlighting.h
    // https://github.com/infancy/pbrt-v3/blob/master/src/integrators/path.h

class Integrater
{
public:
    ~Integrater() = default;
    Integrater()
    {
    }

public:
    void Render(Scene& scene, Camera& camera, Sampler& originalSampler, Film& film)
    {
        auto resolution = film.Resolution();
        int width = (int)resolution.x;
        int height = (int)resolution.y;
        
    #ifndef _DEBUG
        #pragma omp parallel for schedule(dynamic, 1) // OpenMP
    #endif
        for (int y = 0; y < height; y++) // Loop over image rows
        {
            std::unique_ptr<Sampler> sampler = originalSampler.Clone();
            fprintf(stderr, "\rRendering (%d spp) %5.2f%%", sampler->SamplesPerPixel(), 100. * y / (height - 1));

            for (int x = 0; x < width; x++) // Loop cols
            {
                Color L{};

                sampler->StartPixel();
                do
                {
                    auto cameraSample = sampler->GetCameraSample({ (Float)x, (Float)y });
                    auto ray = camera.GenerateRay(cameraSample);

                    L = L + Li(ray, scene, *sampler) * (1. / sampler->SamplesPerPixel());
                }
                while (sampler->StartNextSample());

                film.add_color(x, y, Clamp(L));
            }
        }
    }
    
    // estimate input radiance
    virtual Color Li(Ray ray, Scene& scene, Sampler& sampler) = 0;
};

class PathIntegrater : public Integrater
{
public:
    PathIntegrater(int maxPathDepth) : maxPathDepth{ maxPathDepth }
    {
    }

protected:
    int maxPathDepth;
};

class RecursionPathIntegrater : public PathIntegrater
{
public:
    using PathIntegrater::PathIntegrater;

    Color Li(Ray ray, Scene& scene, Sampler& sampler) override
    {
        return Li(ray, scene, sampler, 0);
    }

    Color Li(Ray ray, Scene& scene, Sampler& sampler, int depth)
    {
        Isect isect;
        if (!scene.Intersect(ray, &isect))
            return Color(); // if miss, return black

        if (depth > maxPathDepth)
            return isect.Le();

        auto bs = isect.bsdf()->Sample_f(isect.wo, sampler.Get2D());
        if (bs.f.IsBlack() || bs.pdf == 0.f) // pdf == 0 => NaN
            return isect.Le();

        //russian roulette
        if (++depth > 5)
        {
            Float maxComponent = bs.f.MaxComponentValue();
            if (sampler.Get1D() < maxComponent) // continue
                bs.f = bs.f * (1 / maxComponent);
            else
                return isect.Le();
        }

        Ray wi(isect.position, bs.wi);
        return isect.Le() + (bs.f * Li(wi, scene, sampler, depth) * AbsDot(bs.wi, isect.normal) / bs.pdf);
    }
};

#pragma endregion

// https://www.pbr-book.org/3ed-2018/Introduction/pbrt_System_Overview
// https://www.pbr-book.org/3ed-2018/Scene_Description_Interface

// https://github.com/infancy/pbrt-v3/blob/master/src/main/pbrt.cpp#L161-L171
// https://github.com/infancy/pbrt-v3/blob/master/src/core/parser.cpp#L1080-L1087
// https://github.com/infancy/pbrt-v3/blob/master/src/core/api.cpp#L1604-L1624

int main(int argc, char* argv[])
{
    int width = 1024, height = 768;
    Film film({ (Float)width, (Float)height }, "image.bmp"s);

    int samplesPerPixel = argc == 2 ? atoi(argv[1]) / 4 : 100;
    std::unique_ptr<Sampler> originalSampler = std::make_unique<RandomSampler>(samplesPerPixel);

    std::unique_ptr<Camera> camera = std::make_unique<PerspectiveCamera>(
        Vector3{ 50, 52, -295.6 }, Vector3{ 0, -0.042612, 1 }.Normalize(), Vector3{ 0, 1, 0 }, 53, film.Resolution());

    auto scene = Scene::CreateSmallptScene();

    std::unique_ptr<Integrater> integrater = std::make_unique<RecursionPathIntegrater>(10);
    integrater->Render(scene, *camera, *originalSampler, film);

    film.store_image();
#if defined(_WIN32) || defined(_WIN64)
    system("mspaint image.bmp");
#endif

    return 0;
}