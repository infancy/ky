#include <algorithm>
#include <array>
#include <cmath>   // smallpt, a Path Tracer by Kevin Beason, 2008
#include <cstdlib> // Make : g++ -O3 -fopenmp smallpt.cpp -o smallpt
#include <cstdio>
#include <ctime>
#include <exception>
#include <format>
#include <numbers>
#include <string>
#include <string_view>



#pragma region macro

#pragma endregion

// namespace ky

#pragma region using/cosntant

using Float = double;
constexpr Float Pi = std::numbers::pi;

#pragma endregion



#pragma region utility

template <typename... Ts>
inline void LOG(const std::string_view fmt, const Ts&... args)
{
    auto msg = std::vformat(fmt, std::make_format_args(args...));
    std::printf("%s", msg.c_str());
}

template <typename... Ts>
inline void LOG_ERROR(const std::string_view fmt, const Ts&... args)
{
    auto msg = std::vformat(fmt, std::make_format_args(args...));
    std::printf("%s", msg.c_str());
    throw std::exception(msg.c_str());
}

#ifdef _DEBUG
    #ifndef LOG_DEBUG
        #define LOG_DEBUG(...) LOG(__VA_ARGS__)
    #endif

    #ifndef DCHECK
        #define DCHECK1(condition)      if(!(condition)) LOG_ERROR("{}", #condition)
        #define DCHECK2(condition, msg) if(!(condition)) LOG_ERROR("{}", msg)
        #define DCHECK3(condition, ...) if(!(condition)) LOG_ERROR(__VA_ARGS__)

        #define GET_MACRO_(_1, _2, _3, NAME, ...) NAME
        #define DCHECK(...) GET_MACRO_(__VA_ARGS__, DCHECK3, DCHECK2, DCHECK1, UNUSED)(__VA_ARGS__)
    #endif
#else
    #ifndef LOG_DEBUG
        #define LOG_DEBUG(...) 
    #endif

    #ifndef DCHECK
        #define DCHECK(...)
    #endif
#endif


#pragma endregion



#pragma region geometry

// https://stackoverflow.com/questions/17333/what-is-the-most-effective-way-for-float-and-double-comparison
// http://realtimecollisiondetection.net/blog/?p=89

template <typename T>
struct equal_epsilon
{
    static constexpr T absolute_epsilon = std::numeric_limits<T>::epsilon();
    static constexpr T relative_epsilon = std::numeric_limits<T>::epsilon();
};

template <typename T>
constexpr bool is_equal(T x, T y, T epsilon = equal_epsilon<T>::absolute_epsilon)
{
    if constexpr (std::is_floating_point_v<T>)
        // return std::fabs(x - y) <= std::max(absolute_epsilon, relative_epsilon * std::max(fabs(x), fabs(y)) );
        return std::abs(x - y) <= epsilon * std::max({ T(1), std::abs(x), std::abs(y) });
    else
        return x == y;
}



struct vec3_t
{        // Usage: time ./smallpt 5000 && xv image.ppm
    union
    {
        struct { Float x, y, z; };
        struct { Float r, g, b; };
        //std::array<Float, 3> array_{};
    };

    vec3_t(Float x = 0, Float y = 0, Float z = 0) { this->x = x; this->y = y; this->z = z; }

    Float operator[](int i) const { DCHECK(i >= 0 && i < 3); return (&x)[i]; }
    vec3_t operator+(const vec3_t& vec3) const { return vec3_t(x + vec3.x, y + vec3.y, z + vec3.z); }
    vec3_t operator-(const vec3_t& vec3) const { return vec3_t(x - vec3.x, y - vec3.y, z - vec3.z); }
    vec3_t operator*(Float scalar) const { return vec3_t(x * scalar, y * scalar, z * scalar); }
    friend vec3_t operator*(Float scalar, vec3_t vec3) { return vec3_t(vec3.x * scalar, vec3.y * scalar, vec3.z * scalar); }


    // ???
    vec3_t multiply(const vec3_t& vec3) const { return vec3_t(x * vec3.x, y * vec3.y, z * vec3.z); }

    friend Float dot(const vec3_t& u, const vec3_t& v) { return u.x * v.x + u.y * v.y + u.z * v.z; }
    Float dot(const vec3_t& vec3) const { return x * vec3.x + y * vec3.y + z * vec3.z; }
    vec3_t cross(const vec3_t& v) 
    {
        /*
            |  i  j  k |
            | ax ay az |
            | bx by bz |
        */
        return vec3_t(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x);
    }

    // or length(), abs(), absolute_value()
    Float magnitude() const { return sqrt(magnitude_sq()); }
    Float magnitude_sq() const { return x * x + y * y + z * z; }

    // unit_vec3_t& normlize() const { return *this * (1 / sqrt(x * x + y * y + z * z)); }
    vec3_t& normlize() { return *this = *this * (1 / sqrt(x * x + y * y + z * z)); } // TODO: const
    bool is_unit() const { return is_equal(magnitude(), 1.); }
};

/*
class unit_vector_t;
using direction_t = unit_vector_t;
using normal_t = unit_vector_t
*/

using point3_t = vec3_t; // object_point, world_point... we need a frame
using normal_t = vec3_t;
using color_t = vec3_t;
using unit_vec3_t = vec3_t;

struct ray_t
{
    point3_t origin_;
    unit_vec3_t direction_; // confirm it is Unit Vector

    ray_t(const point3_t& origin, const unit_vec3_t& direction) : 
        origin_(origin), 
        direction_(direction) 
    {
        /* TODO
        DCHECK(direction_.is_unit());
        DCHECK(direction_.is_unit(), "aabb");
        DCHECK(direction_.is_unit(), "not unit vector magnitude is {}", direction_.magnitude());

        GET_MACRO_(direction_.is_unit(), DCHECK3, DCHECK2, DCHECK1) (direction_.is_unit());
        GET_MACRO_(direction_.is_unit(), "aabb", DCHECK3, DCHECK2, DCHECK1) (direction_.is_unit(), "aabb");
        GET_MACRO_(__VA_ARGS__, DCHECK3, DCHECK2, DCHECK1, UNUSED)(__VA_ARGS__);
        */
    }

    vec3_t origin() const { return origin_; }
    vec3_t direction() const { return direction_; }

    vec3_t operator()(Float t) const
    {
        DCHECK(t >= 0);
        return origin_ + t * direction_;
    }
};

#pragma endregion



#pragma region sampling

struct RandomLCG {
    unsigned mSeed;
    RandomLCG(unsigned seed = 0) : mSeed(seed) {}
    double operator()() { mSeed = 214013 * mSeed + 2531011; return mSeed * (1.0 / 4294967296); }
};
#define RANDOM(Xi) Xi()
#define RANDOM_INIT(Xi) RandomLCG Xi;
#define RANDOM_PARAM(Xi) RandomLCG& Xi

// TODO: Pseudo or Quasi
struct RNG
{

};

#pragma endregion



#pragma region shape

enum class surface_scattering_e { diffuse, specular, refractive };  // material types, used in radiance()

struct shape_t
{
    color_t color_;
    surface_scattering_e surface_scattering_;
};

struct intersection_t
{
    point3_t position; // world position

};

struct sphere_t
{
    vec3_t center_;
    Float radius_;
    Float radius_sq_;

    color_t emission_;
    color_t color_;
    surface_scattering_e surface_scattering_;

    sphere_t(Float radius, vec3_t center, vec3_t e_, vec3_t c_, surface_scattering_e refl_) :
        center_(center),
        radius_(radius),
        radius_sq_(radius * radius),
        emission_(e_),
        color_(c_),
        surface_scattering_(refl_)
    {
    }

    Float intersect(const ray_t& r) const
    { 
        // returns distance, 0 if nohit

        /*
            ||o + t*d - c||^2 = r^2
            (t*d + o - c).(t*d + o - c) = r^2

            t^2*d.d + 2t*d.(o-c) + (o-c).(o-c)-r^2 = 0
            at^2 + bt + c = 0

            oc = o - c
            a = dot(d, d) = 1;
            b = 2 * dot(d, oc);
            c = dot(oc, oc) - r^2;

            t = (-b +/- sqrt(b^2 - 4ac)) / 2a
              = (-b +/- sqrt(b^2 - 4c)) / 2
              = ((-2 * dot(d, oc) +/- sqrt(4 * dot(d, oc)^2 - 4 * (dot(oc, oc) - r^2))) / 2
              = -dot(d, oc) +/- sqrt( dot(d, oc)^2 - dot(oc, oc) + r^2 )
              = -b' +/- sqrt(det)
        */

        vec3_t co = center_ - r.origin();
        Float neg_b = dot(co, r.direction());
        Float discr = neg_b * neg_b - dot(co, co) + radius_sq_;

        if (discr >= 0)
         {
            Float sqrt_discr = sqrt(discr);

            Float t = 0;
            Float eps = 1e-4;

            if (t = neg_b - sqrt_discr; t > eps)
            {
                return t;
            }
            else if (t = neg_b + sqrt_discr; t > eps)
            {
                return t;
            }
        }

        return 0;
    }
};

#pragma endregion



#pragma region accelerator(optional)

enum class accelerator_e
{
    none,
};

// accel_t

#pragma endregion



#pragma region scene

// TODO: std::vector<std::function<intersect(ray_t ray), result_t> primitives_;

sphere_t scene[] = {//Scene: radius, center, emission, color, material
  sphere_t(1e5, vec3_t(1e5 + 1,40.8,81.6), vec3_t(),vec3_t(.75,.25,.25),surface_scattering_e::diffuse),//Left
  sphere_t(1e5, vec3_t(-1e5 + 99,40.8,81.6),vec3_t(),vec3_t(.25,.25,.75),surface_scattering_e::diffuse),//Rght
  sphere_t(1e5, vec3_t(50,40.8, 1e5),     vec3_t(),vec3_t(.75,.75,.75),surface_scattering_e::diffuse),//Back
  sphere_t(1e5, vec3_t(50,40.8,-1e5 + 170), vec3_t(),vec3_t(),           surface_scattering_e::diffuse),//Frnt
  sphere_t(1e5, vec3_t(50, 1e5, 81.6),    vec3_t(),vec3_t(.75,.75,.75),surface_scattering_e::diffuse),//Botm
  sphere_t(1e5, vec3_t(50,-1e5 + 81.6,81.6),vec3_t(),vec3_t(.75,.75,.75),surface_scattering_e::diffuse),//Top
  sphere_t(16.5,vec3_t(27,16.5,47),       vec3_t(),vec3_t(1,1,1) * .999, surface_scattering_e::specular),//Mirr
  sphere_t(16.5,vec3_t(73,16.5,78),       vec3_t(),vec3_t(1,1,1) * .999, surface_scattering_e::refractive),//Glas
  sphere_t(600, vec3_t(50,681.6 - .27,81.6),vec3_t(12,12,12),  vec3_t(), surface_scattering_e::diffuse) //Lite
};

bool intersect(const ray_t& r, double& t, int& id)
{
    int n = sizeof(scene) / sizeof(sphere_t);
    Float direction_, inf = t = 1e20;
    for (int i = int(n); i--;)
        if ((direction_ = scene[i].intersect(r)) && direction_ < t)
        {
            t = direction_; id = i;
        }

    return t < inf;
}

#pragma endregion



#pragma region camera, image

constexpr double clamp01(double x) { return std::clamp(x, 0., 1.); }
int gamma_encoding(double x) { return int(pow(clamp01(x), 1 / 2.2) * 255 + .5); }

class image_t
{

};

void save_bmp()
{

}

class camera_t
{

};

#pragma endregion



#pragma region material, bsdf

#pragma endregion



#pragma region light

#pragma endregion



#pragma region integrater

enum class integrater_e
{
    // debug
    depth,
    normal,

    // discrete
    direct_lighting_point, 

    // ray casting/direct lighting
    direct_lighting_bsdf,
    direct_lighting_light,
    direct_lighting_mis,

    // stochastic ray tracing
    stochastic_ray_tracing,

    // path tracing
    path_tracing_recursion,
    path_tracing_iteration
};

vec3_t radiance(const ray_t& r, int depth, RANDOM_PARAM(Xi))
{
    double t;                               // distance to intersection
    int id = 0;                               // id of intersected object

    if (!intersect(r, t, id))
        return vec3_t(); // if miss, return black

    const sphere_t& obj = scene[id];        // the hit object
    vec3_t x = r.origin_ + r.direction_ * t, n = (x - obj.position_).normlize(), nl = n.dot(r.direction_) < 0 ? n : n * -1, f = obj.color_;
    double position_ = f.x > f.y && f.x > f.z ? f.x : f.y > f.z ? f.y : f.z; // max refl

    if (++depth > 5)
    {
        if (RANDOM(Xi) < position_)
            f = f * (1 / position_);
        else
            return obj.emission_; //R.R.
    }

    if (depth > 100) return obj.emission_; // MILO

    if (obj.surface_scattering_ == surface_scattering_e::diffuse)
    {                  // Ideal DIFFUSE reflection
        double r1 = 2 * Pi * RANDOM(Xi), r2 = RANDOM(Xi), r2s = sqrt(r2);
        vec3_t w = nl;
        vec3_t u = ((fabs(w.x) > .1 ? vec3_t(0, 1) : vec3_t(1)).cross(w)).normlize();
        vec3_t v = w.cross(u);

        vec3_t direction_ = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normlize();
        return obj.emission_ + f.multiply(radiance(ray_t(x, direction_), depth, Xi));
    }
    else if (obj.surface_scattering_ == surface_scattering_e::specular)            // Ideal SPECULAR reflection
        return obj.emission_ + f.multiply(radiance(ray_t(x, r.direction_ - n * 2 * n.dot(r.direction_)), depth, Xi));

    ray_t reflRay(x, r.direction_ - n * 2 * n.dot(r.direction_));     // Ideal dielectric REFRACTION
    bool into = n.dot(nl) > 0;                // Ray from outside going in?
    double nc = 1, nt = 1.5, nnt = into ? nc / nt : nt / nc, ddn = r.direction_.dot(nl), cos2t;
    if ((cos2t = 1 - nnt * nnt * (1 - ddn * ddn)) < 0)    // Total internal reflection
        return obj.emission_ + f.multiply(radiance(reflRay, depth, Xi));

    vec3_t tdir = (r.direction_ * nnt - n * ((into ? 1 : -1) * (ddn * nnt + sqrt(cos2t)))).normlize();
    double a = nt - nc, b = nt + nc, R0 = a * a / (b * b), color_ = 1 - (into ? -ddn : tdir.dot(n));
    double Re = R0 + (1 - R0) * color_ * color_ * color_ * color_ * color_, Tr = 1 - Re, P = .25 + .5 * Re, RP = Re / P, TP = Tr / (1 - P);

    return obj.emission_ + f.multiply(depth > 2 ? (RANDOM(Xi) < P ?   // Russian roulette
        radiance(reflRay, depth, Xi) * RP : radiance(ray_t(x, tdir), depth, Xi) * TP) :
        radiance(reflRay, depth, Xi) * Re + radiance(ray_t(x, tdir), depth, Xi) * Tr);
}

#pragma endregion



#pragma region interactive/debug

// single ray debug

#pragma endregion


#pragma region main

int main(int argc, char* argv[])
{
    clock_t start = clock(); // MILO

    int w = 256, h = 256, samps = argc == 2 ? atoi(argv[1]) / 4 : 10; // # samples

    ray_t cam(vec3_t(50, 52, 295.6), vec3_t(0, -0.042612, -1).normlize()); // cam pos, dir

    vec3_t cx = vec3_t(w * .5135 / h);
    vec3_t cy = (cx.cross(cam.direction_)).normlize() * .5135;

    vec3_t Li;
    vec3_t* color_ = new vec3_t[w * h];

#pragma omp parallel for schedule(dynamic, 1) private(Li)       // OpenMP
    for (int y = 0; y < h; y++) 
    {                       // Loop over image rows
        LOG("\rRendering ({} spp) {}", samps * 4, 100. * y / (h - 1));

        RANDOM_INIT(Xi);// MILO
        for (unsigned short x = 0; x < w; x++)   // Loop cols

            for (int sy = 0, i = (h - y - 1) * w + x; sy < 2; sy++)     // 2x2 subpixel rows
                for (int sx = 0; sx < 2; sx++, Li = vec3_t())
                {        // 2x2 subpixel cols
                    for (int s = 0; s < samps; s++) {
                        double r1 = 2 * RANDOM(Xi), dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
                        double r2 = 2 * RANDOM(Xi), dy = r2 < 1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2);
                        vec3_t direction_ = cx * (((sx + .5 + dx) / 2 + x) / w - .5) +
                            cy * (((sy + .5 + dy) / 2 + y) / h - .5) + cam.direction_;
                        Li = Li + radiance(ray_t(cam.origin_ + direction_ * 140, direction_.normlize()), 0, Xi) * (1. / samps);
                    } // Camera rays are pushed ^^^^^ forward to start in interior
                    color_[i] = color_[i] + vec3_t(clamp01(Li.x), clamp01(Li.y), clamp01(Li.z)) * .25;
                }
    }

    LOG("\n{} sec\n", (float)(clock() - start) / CLOCKS_PER_SEC); // MILO

    FILE* file = fopen("image.ppm", "w");         // Write image to PPM file.
    fprintf(file, "P3\n%d %d\n%d\n", w, h, 255);
    for (int i = 0; i < w * h; i++)
        fprintf(file, "%d %d %d ", gamma_encoding(color_[i].x), gamma_encoding(color_[i].y), gamma_encoding(color_[i].z));
    fclose(file);
}

#pragma endregion






#pragma region assert io

#pragma endregion
