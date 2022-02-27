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

    Vector3 Normalize() const { return *this * (1 / sqrt(x * x + y * y + z * z)); }

    Float Dot(const Vector3& b) const { return x * b.x + y * b.y + z * b.z; }
    Vector3 Cross(const Vector3& b) const { return Vector3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x); }

    friend Vector3 operator*(Float b, Vector3 v) { return v * b; }
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
class BSDF;

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

    MaterialType materialType{}; // TODO: remove
    Color bsdfValue{}; // TODO: remove

    std::unique_ptr<BSDF> bsdf_{};
    Color emission_{};

    const BSDF* bsdf() const { return bsdf_.get(); }

    // prev <- isect, against ray's direction
    Color Le() const { return emission_; }

private:
    // TODO: remove
    friend Sphere;
};

#pragma endregion



#pragma region Sampling

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

inline Float CosineHemispherePdf(Float cosTheta) { return cosTheta * InvPi; }

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
        }

        return hit;
    }
};

#pragma endregion



#pragma region BSDF

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

            isect->materialType = Scene[i].materialType;
            isect->bsdfValue = Scene[i].color;

            if(isect->materialType == MaterialType::Diffuse)
                isect->bsdf_ = std::make_unique<LambertionReflection>(Frame(isect->normal), isect->bsdfValue);
            else if(isect->materialType == MaterialType::Specular)
                isect->bsdf_ = std::make_unique<SpecularReflection>(Frame(isect->normal), isect->bsdfValue);
            else
                isect->bsdf_ = std::make_unique<FresnelSpecular>(Frame(isect->normal), isect->bsdfValue, isect->bsdfValue, 1, 1.5);

            isect->emission_ = Scene[i].emission;
            
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

    auto bs = isect.bsdf()->Sample_f(isect.wo, sampler.Get2D());
    if (bs.f.IsBlack() || bs.pdf == 0.f) // pdf == 0 => NaN
        return isect.Le();

    //russian roulette
    if (++depth > 5)
    {
        Float bsdf_max_comp = bs.f.MaxComponentValue();
        if (sampler.Get1D() < bsdf_max_comp) // continue
            bs.f = bs.f * (1 / bsdf_max_comp);
        else
            return isect.Le();
    }

    Ray wi(isect.position, bs.wi);
    return isect.Le() + (bs.f * Radiance(wi, depth, sampler) * AbsDot(bs.wi, isect.normal) / bs.pdf);
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