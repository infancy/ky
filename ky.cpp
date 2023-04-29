//#define KY_OUTPUT_HDR // default output .bmp image, whether need to output .hdr image
//#define KY_LOG_VAST

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <ctime>

#include <algorithm>
#include <array>
#include <concepts>
#include <exception>
#include <format>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <source_location>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

using namespace std::literals::string_literals;



#pragma region macro

#if defined(_DEBUG)
    #define KY_DEBUG
#else
    #define KY_RELEASE
#endif

#if defined(_WIN32) || defined(_WIN64)
    #define KY_WINDOWS

    #if defined(_MSC_VER)
        #define KY_MSVC
    #elif defined(__MINGW32__)  
        #define KY_MINGW
    #endif
#elif defined(__linux__)
    #define KY_LINUX
#elif defined(__APPLE__)
    #define KY_MACOS
#endif



template <typename... Ts>
inline void _LOG(const std::source_location& location, const std::string& fmt, Ts&&... args)
{
    std::string msg = std::vformat("{}(...) line{}: " + fmt,
        std::make_format_args(location.function_name(), location.line(), std::forward<Ts>(args)...));
    std::printf("%s", msg.c_str());
}

template <typename... Ts>
inline void _LOG_ERROR(const std::source_location& location, const std::string& fmt, Ts&&... args)
{
    std::string msg = std::vformat("{}(...) line{}: " + fmt,
        std::make_format_args(location.function_name(), location.line(), std::forward<Ts>(args)...));
    std::printf("%s", msg.c_str());

    throw std::exception(msg.c_str());
}

#define LOG(...) _LOG(std::source_location::current(), __VA_ARGS__)
#define LOG_ERROR(...) _LOG_ERROR(std::source_location::current(), __VA_ARGS__)



#define _CHECK1(condition)      if(!(condition)) LOG_ERROR("{}", #condition)
#define _CHECK2(condition, msg) if(!(condition)) LOG_ERROR("{}", msg)
#define _CHECK3(condition, ...) _EXPAND( if(!(condition)) LOG_ERROR(__VA_ARGS__) )

#define _GET_MACRO(_1, _2, _3, _4, _5, NAME, ...) NAME
#define _EXPAND( x ) x

#define CHECK(...) _EXPAND( _GET_MACRO(__VA_ARGS__, _CHECK3, _CHECK3, _CHECK3, _CHECK2, _CHECK1, UNUSED) (__VA_ARGS__) )



#ifdef KY_DEBUG
    #ifndef LOG_DEBUG
        #define LOG_DEBUG(...) LOG(__VA_ARGS__)
    #endif

    #ifndef CHECK_DEBUG
        #define CHECK_DEBUG(...) _EXPAND( CHECK(__VA_ARGS__) )
    #endif
#else
    #ifndef LOG_DEBUG
        #define LOG_DEBUG(...) 
    #endif

    #ifndef CHECK_DEBUG
        #define CHECK_DEBUG(...)
    #endif
#endif

#ifdef KY_LOG_VAST
    #ifndef LOG_VAST
        #define LOG_VAST(...) LOG(__VA_ARGS__)
    #endif
#else
    #ifndef LOG_VAST
        #define LOG_VAST(...) 
    #endif
#endif



#define KY_ENUM_OPERATORS(enum_t)                                                           \
constexpr enum_t  operator~(enum_t a)                 { return (enum_t)(~(int)a); }         \
constexpr enum_t  operator|(enum_t a, const enum_t b) { return (enum_t)((int)a | (int)b); } \
constexpr enum_t  operator&(enum_t a, const enum_t b) { return (enum_t)((int)a & (int)b); } \
constexpr enum_t& operator|=(enum_t& a, const enum_t b) { a = a | b; return a; };           \
constexpr enum_t& operator&=(enum_t& a, const enum_t b) { a = a & b; return a; };           \
constexpr bool enum_have(enum_t group, enum_t value) { return (group & value) != (enum_t)0; }

#pragma endregion

#pragma region utility

class nocopyable_t
{
protected:
    constexpr nocopyable_t() = default;
    ~nocopyable_t() = default;

    nocopyable_t(nocopyable_t&&) = default;
    nocopyable_t& operator=(nocopyable_t&&) = default;

private:
    nocopyable_t(const nocopyable_t&);
    nocopyable_t& operator=(const nocopyable_t&);
};

float timing_seconds(std::invocable auto function)
{
    clock_t start = clock();

    function();

    return (float_t)(clock() - start) / CLOCKS_PER_SEC;
}

#pragma endregion



#pragma region math

//using float_t = float; // already defined in math.h
static_assert(sizeof(float_t) == 4);

using float01_t    = float_t; // from 0 to 1
using unit_float_t = float_t; // 'normalized float_t', from -1 to 1

using radian_t = float_t;
using degree_t = float_t;

constexpr float_t k_epsilon  = std::numeric_limits<float_t>::epsilon();
constexpr float_t k_infinity = std::numeric_limits<float_t>::infinity();
constexpr float_t k_pi  = std::numbers::pi;
constexpr float_t k_2pi = 2.f * k_pi;
constexpr float_t k_pi_over2 = k_pi / 2.f;
constexpr float_t k_pi_over4 = k_pi / 4.f;
constexpr float_t k_inv_pi  = std::numbers::inv_pi;
constexpr float_t k_inv_2pi = k_inv_pi / 2.f;
constexpr float_t k_inv_4pi = k_inv_pi / 4.f;

constexpr radian_t radians(degree_t degree) { return (k_pi / 180.f) * degree; }
constexpr degree_t degrees(radian_t radian) { return (180.f / k_pi) * radian; }

constexpr float_t lerp(float_t a, float_t b, float_t t) { return a + t * (b - a); }

inline bool is_infinity(std::floating_point auto x) { return std::isinf(x); }
inline bool is_nan(std::floating_point auto x) { return std::isnan(x); }

inline bool is_invalid(std::floating_point auto x) { return is_infinity(x) || is_nan(x); }
inline bool is_valid(std::floating_point auto x) { return !is_invalid(x); }


// https://stackoverflow.com/questions/17333/what-is-the-most-effective-way-for-float_t-and-double-comparison
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

#pragma endregion

#pragma region geometry

struct color_t
{
    float_t r{}, g{}, b{};

    color_t operator*(float_t s) const { return { r * s, g * s, b * s }; }
    color_t operator/(float_t s) const { return { r / s, g / s, b / s }; }

    color_t operator*=(float_t s) { r *= s, g *= s, b *= s; return *this; }
    color_t operator/=(float_t s) { r /= s, g /= s, b /= s; return *this; }

    color_t operator+(color_t c) const { return { r + c.r, g + c.g, b + c.b }; }
    color_t operator*(color_t c) const { return { r * c.r, g * c.g, b * c.b }; }

    color_t operator+=(color_t c) { r += c.r, g += c.g, b += c.b; return *this; }
    color_t operator*=(color_t c) { r *= c.r, g *= c.g, b *= c.b; return *this; }

    friend color_t operator*(float_t s, color_t c) { return { s * c.r, s * c.g, s * c.b }; }

    float_t max_component_value() const
    {
        return std::max({ r, g, b });
    }

    float_t luminance() const
    {
        return
            0.212671f * r +
            0.715160f * g +
            0.072169f * b;
    }

    // TODO: why allow nagetive value?
    bool is_black() const { return (r <= 0) && (g <= 0) && (b <= 0); }

// for debug:

    bool is_valid() const { return ::is_valid(r) && ::is_valid(g) && ::is_valid(b); }

    std::string to_string() const { return std::format("[{:.3f}, {:.3f}, {:.3f}]", r, g, b); }
    friend std::ostream& operator<<(std::ostream& console, color_t color)
    {
        console << color.to_string();
        return console;
    }
};



struct vec2_t
{
    float_t x{}, y{};

    float_t operator[](int i) const { CHECK_DEBUG(i >= 0 && i < 2); return (&x)[i]; }

    vec2_t& operator+=(vec2_t v) { x += v.x; y += v.y; return *this; }
    vec2_t& operator-=(vec2_t v) { x -= v.x; y -= v.y; return *this; }

    vec2_t operator+(vec2_t vec2) const { return vec2_t(x + vec2.x, y + vec2.y); }
    vec2_t operator-(vec2_t vec2) const { return vec2_t(x - vec2.x, y - vec2.y); }

    friend vec2_t operator*(float_t s, vec2_t v) { return vec2_t(v.x * s, v.y * s); }
};
using point2_t = vec2_t;
using float2_t = vec2_t;


struct vec3_t
{
    float_t x{}, y{}, z{};

    float_t operator[](int i) const { CHECK_DEBUG(i >= 0 && i < 3); return (&x)[i]; }
    vec3_t operator-() const { return vec3_t(-x, -y, -z); }

    vec3_t& operator+=(vec3_t v) { x += v.x; y += v.y; z += v.z; return *this; }
    vec3_t& operator-=(vec3_t v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    vec3_t& operator*=(float_t s) { x *= s; y *= s; z *= s; return *this; }
    vec3_t& operator/=(float_t s) { x /= s; y /= s; z /= s; return *this; }

    vec3_t operator+(vec3_t v) const { return vec3_t(x + v.x, y + v.y, z + v.z); }
    vec3_t operator-(vec3_t v) const { return vec3_t(x - v.x, y - v.y, z - v.z); }
    vec3_t operator*(float_t s)    const { return vec3_t(x * s, y * s, z * s); }
    vec3_t operator/(float_t s)    const { return vec3_t(x / s, y / s, z / s); }

    // or length(), abs(), absolute_value()
    float_t magnitude()    const { return sqrt(magnitude_squared()); }
    float_t magnitude_squared() const { return x * x + y * y + z * z; }

    // unit_vec3_t& normlize() const { return *this * (1 / sqrt(x * x + y * y + z * z)); }
    vec3_t normalize() const { return *this * (1 / sqrt(x * x + y * y + z * z)); }
    bool is_unit() const { return is_equal(magnitude(), (float_t)1); }

    float_t dot(vec3_t v) const { return x * v.x + y * v.y + z * v.z; }
    vec3_t cross(vec3_t v) const
    {
        /*
            |  i  j  k |
            |  x  y  z |
            | vx vy vz |
        */
        return vec3_t(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x);
    }

    operator color_t() { return { x, y, z }; }

public:
    friend vec3_t operator*(float_t s, vec3_t v) { return vec3_t(v.x * s, v.y * s, v.z * s); }

    friend float_t     dot(vec3_t u, vec3_t v) { return u.dot(v); }
    friend float_t abs_dot(vec3_t u, vec3_t v) { return std::abs(u.dot(v)); }
    friend vec3_t  cross(vec3_t u, vec3_t v) { return u.cross(v); }
    friend vec3_t normalize(vec3_t v) { return v.normalize(); }

    friend float_t     cos(vec3_t u, vec3_t v) { return u.dot(v); }
    friend float_t abs_cos(vec3_t u, vec3_t v) { return std::abs(u.dot(v)); }
    friend vec3_t   lerp(vec3_t u, vec3_t v, float_t t) { return u + t * (v - u); }

    // per-component
    friend vec3_t min(vec3_t a, vec3_t b)
    {
        return vec3_t(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
    }
    // per-component
    friend vec3_t max(vec3_t a, vec3_t b)
    {
        return vec3_t(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
    }

public:
    // deubg
    bool is_valid() const { return ::is_valid(x) && ::is_valid(y) && ::is_valid(z); }
    bool is_zero() const { return (x == 0) && (y == 0) && (z == 0); }
    bool has_negative() const { return (x < 0) || (y < 0) || (z < 0); }
    bool small_than(vec3_t vec3) const { return (x < vec3.x) || (y < vec3.y) || (z < vec3.z); }

    std::string to_string() const { return std::format("[{:.6f}, {:.6f}, {:.6f}]", x, y, z);  }
    friend std::ostream& operator<<(std::ostream& console, vec3_t vec3)
    {
        console << vec3.to_string();
        return console;
    }
};
/*
class unit_vector_t;
using direction_t = unit_vector_t;
using normal_t = unit_vector_t
*/
using point3_t    = vec3_t; // object_point, world_point... we need a frame
using normal_t    = vec3_t;
using unit_vec3_t = vec3_t;
using float3_t    = vec3_t;


inline float_t distance(point3_t p1, point3_t p2)
{
    return (p1 - p2).magnitude();
}
inline float_t distance_squared(point3_t p1, point3_t p2)
{
    return (p1 - p2).magnitude_squared();
}



// TODO: confirm
/*
     z(0, 0, 1)
          |
          | theta/
          |    /
          |  /
          |/_ _ _ _ _ _ x(1, 0, 0)
         / \
        / phi\
       /       \
      /          \
 y(0, 1, 0)

   https://www.pbr-book.org/3ed-2018/Shapes/Spheres
*/

// unit direction vector -> spherical coordinate
inline radian_t spherical_theta(unit_vec3_t v)
{
    return std::acos(std::clamp(v.z, (float_t)-1, (float_t)1));
}
inline radian_t spherical_phi(unit_vec3_t v)
{
    float_t phi = std::atan2(v.y, v.x);
    return (phi < 0) ? (phi + k_2pi) : phi;
}


// convert spherical coordinate (θ theta, φ phi) into direction vector (x, y, z)
inline vec3_t spherical_to_direction(float_t sin_theta, float_t cos_theta, float_t phi)
{
    return vec3_t(
        sin_theta * std::cos(phi),
        sin_theta * std::sin(phi),
        cos_theta);
}
// takes three basis vectors representing the x, y, and z axis and
// returns the appropriate direction vector with respect to the coordinate frame defined by them
inline vec3_t spherical_to_direction(
    float_t sin_theta, float_t cos_theta, float_t phi,
    vec3_t x, vec3_t y, vec3_t z)
{
    return
        sin_theta * std::cos(phi) * x + 
        sin_theta * std::sin(phi) * y +
        cos_theta * z;
}



/* TODO
              y         z
              |       /
              |     /
              |   /
              | /
              o - - - - x
              
                       bounds3_t.max
                  3-------2
                 /|      /|
                4-------1 |
                | |     | |
                | 7-----|-6
                |/      |/
                8-------5
        bounds3_t.min
*/
class bounds3_t
{
public:
    bounds3_t() 
    {
        constexpr float_t min = std::numeric_limits<float_t>::lowest();
        constexpr float_t max = std::numeric_limits<float_t>::max();

        min_ = point3_t(max, max, max);
        max_ = point3_t(min, min, min);
    }

    explicit bounds3_t(point3_t p) : min_(p), max_(p)
    {
    }

    bounds3_t(point3_t p1, point3_t p2): 
        min_(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z)),
        max_(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z))
    {
    }

public:
    // union, merge
    bounds3_t join(point3_t p) const
    {
        return bounds3_t(min(min_, p), max(max_, p));
    }
    bounds3_t join(const bounds3_t& b) const
    {
        return bounds3_t(min(min_, b.min_), max(max_, b.max_));
    }

    friend bounds3_t join(const bounds3_t& b, point3_t p) { return b.join(p); }
    friend bounds3_t join(const bounds3_t& b1, const bounds3_t& b2) { return b1.join(b2); }

public:
    bool contain(point3_t p) const
    {
        return
            p.x >= min_.x && p.x <= max_.x &&
            p.y >= min_.y && p.y <= max_.y &&
            p.z >= min_.z && p.z <= max_.z;
    }

public:
    // return a sphere that hold this bounding box
    void bounding_sphere(point3_t* center, float_t* radius_) const
    {
        *center = lerp(min_, max_, (float_t)0.5);
        *radius_ = contain(*center) ? distance(*center, max_) : 0;
    }

private:
    point3_t min_, max_;
};


struct mat4_t
{
};



// https://github.com/SmallVCM/SmallVCM/blob/master/src/frame.hxx
class frame_t
{
public:
    frame_t() = default;
    frame_t(vec3_t s, vec3_t t, normal_t n) :
        s_{ s.normalize() },
        t_{ t.normalize() },
        n_{ n.normalize() }
    {
    }

    frame_t(normal_t n) :
        n_{ n.normalize()}
    {
        set_from_z();
    }

public:
    // think if {s, t, n} is (1, 0, 0), (0, 1, 0), (0, 0, 1)
    vec3_t to_local(vec3_t world_vec3) const
    {
        return vec3_t(
            dot(s_, world_vec3),
            dot(t_, world_vec3),
            dot(n_, world_vec3));
    }

    vec3_t to_world(vec3_t local_vec3) const
    {
        return 
            s_ * local_vec3.x + 
            t_ * local_vec3.y + 
            n_ * local_vec3.z;
    }

    vec3_t binormal() const { return s_; }
    vec3_t tangent() const { return t_; }
    vec3_t normal() const { return n_; }

private:
    void set_from_z()
    {
        vec3_t tmp_s = (std::abs(n_.x) > 0.99f) ? vec3_t(0, 1, 0) : vec3_t(1, 0, 0);
        t_ = normalize(cross(n_, tmp_s));
        s_ = normalize(cross(t_, n_));
    }

private:
    // world frame basic vector
    vec3_t   s_{ 1, 0, 0 }; // x
    vec3_t   t_{ 0, 1, 0 }; // y
    normal_t n_{ 0, 0, 1 }; // z
};



class ray_t
{
public:
    ray_t(point3_t origin, unit_vec3_t direction, const float_t distance = k_infinity) :
        origin_{ origin },
        direction_{ direction },
        distance_{ distance }
    {
        // TODO: move to unit_vec3_t
        // CHECK_DEBUG(direction_.is_unit(), "ray.direction {} magnitude {:.6f} not a unit vector", 
        //     direction_.to_string(), direction_.magnitude());
    }

    point3_t origin() const { return origin_; }
    unit_vec3_t direction() const { return direction_; }
    float_t distance() const { return distance_; }

    void set_distance(float_t distance) const { distance_ = distance; }

    point3_t operator()(float_t t) const
    {
        CHECK_DEBUG(t >= 0);
        return origin_ + t * direction_;
    }

private:
    point3_t origin_;
    unit_vec3_t direction_; // confirm it is a unit vector
    mutable float_t distance_; // distance from ray to intersection
};



class bsdf_t;
class material_t;
class area_light_t;
class surface_t;
using bsdf_uptr_t = std::unique_ptr<bsdf_t>;

// avoid self intersection
point3_t offset_ray_origin(point3_t position, normal_t normal, unit_vec3_t direction)
{
    vec3_t offset = normal * 1e-2; // TODO: 1e-2
    if (dot(normal, direction) < 0)
        offset = -offset;
    return position + offset;
}

/*
  prev   n   light
  ----   ^   -----
    ^    |    ^
     \   | θ /
   wo \  |  / wi is unknown, sampling from bsdf or light
       \ | / 
        \|/
      -------
       isect
*/

// surface intersection
class isect_t : public nocopyable_t
{
public:
    isect_t() = default;
    isect_t(point3_t position, normal_t normal, unit_vec3_t wo) :
        position{ position },
        normal{ normal },
        wo{ wo }
    {
    }

public:
    //void scattering();

    const surface_t* surface() const { return surface_; }

    const bsdf_t* bsdf() const { return bsdf_.get(); }

    // prev <- isect, against ray's direction
    color_t Le() const { return emission_; }

public:
    ray_t spawn_ray(unit_vec3_t direction) const
    {
        return ray_t{ offset_ray_origin(position, normal, direction), direction };
    }

    ray_t spawn_ray_to(point3_t target) const
    {
        return spawn_ray(normalize(target - position));
    }
    ray_t spawn_ray_to(const isect_t& isect) const
    {
        return spawn_ray(normalize(isect.position - position));
    }

public:
    point3_t position{}; // world position of intersection
    normal_t normal{};
    unit_vec3_t wo{};

private:
    const surface_t* surface_{};
    bsdf_uptr_t bsdf_{};
    color_t emission_{};

    friend surface_t;
};
using light_isect_t = isect_t;

#pragma endregion



#pragma region sampling

// https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations
// https://github.com/mmp/pbrt-v3/blob/master/src/core/sampling.cpp

inline point2_t uniform_disk_sample(float2_t random)
{
    float_t radius = std::sqrt(random[0]);
    float_t theta = 2 * k_pi * random[1];
    return radius * point2_t(std::cos(theta), std::sin(theta));
}

inline point2_t concentric_disk_sample(float2_t random)
{
    // map uniform random numbers to $[-1,1]^2$
    random = 2.f * random - vec2_t(1, 1);

    // handle degeneracy at the origin
    if (random.x == 0 && random.y == 0)
        return point2_t(0, 0);

    // apply concentric mapping to point
    float_t radius{}, theta{};
    if (std::abs(random.x) > std::abs(random.y))
    {
        radius = random.x;
        theta = k_pi_over4 * (random.y / random.x);
    }
    else
    {
        radius = random.y;
        theta = k_pi_over2 - k_pi_over4 * (random.x / random.y);
    }

    return radius * point2_t(std::cos(theta), std::sin(theta));
}


// cosine-weighted hemisphere sampling
inline vec3_t cosine_hemisphere_sample(float2_t random)
{
    point2_t p = concentric_disk_sample(random);
    float_t z = std::sqrt(std::max((float_t)0, 1 - p.x * p.x - p.y * p.y));

    return vec3_t(p.x, p.y, z);
}

inline float_t cosine_hemisphere_pdf(float_t cos_theta) { return cos_theta * k_inv_pi; }


inline vec3_t uniform_hemisphere_sample(float2_t random)
{
    float_t z = random[0]; // [0, 1)
    float_t radius = std::sqrt(std::max((float_t)0, (float_t)1. - z * z));

    float_t phi = 2 * k_pi * random[1];

    return vec3_t(radius * std::cos(phi), radius * std::sin(phi), z);
}

inline float_t uniform_hemisphere_pdf() { return k_inv_2pi; }


inline vec3_t uniform_sphere_sample(float2_t random)
{
    float_t z = 1 - 2 * random[0]; // (-1, 1)
    float_t radius = std::sqrt(std::max((float_t)0, (float_t)1 - z * z));

    float_t phi = 2 * k_pi * random[1];

    return vec3_t(radius * std::cos(phi), radius * std::sin(phi), z);
}

inline float_t uniform_sphere_pdf() { return k_inv_4pi; }


/*

        /         _
       /        / O \
      /         O O O (a sphere)
     /       .  \ O /
    /    .
   / .     theta
  . _ _ _ _ _ _ _ _

*/
vec3_t uniform_cone_sample(float2_t random, float_t cos_theta_max)
{
    float_t cos_theta = (1 - random[0]) + random[0] * cos_theta_max;
    float_t sin_theta = std::sqrt((float_t)1 - cos_theta * cos_theta);

    float_t phi = random[1] * 2 * k_pi;

    return vec3_t(
        std::cos(phi) * sin_theta,
        std::sin(phi) * sin_theta,
        cos_theta);
}

float_t uniform_cone_pdf(float_t cos_theta_max)
{
    return 1 / (2 * k_pi * (1 - cos_theta_max));
}


point2_t uniform_triangle_sample(float2_t random)
{
    float_t su0 = std::sqrt(random[0]);
    return point2_t(1 - su0, random[1] * su0);
}


inline float_t balance_heuristic(int f_num, float_t f_pdf, int g_num, float_t g_pdf)
{
    return (f_num * f_pdf) / (f_num * f_pdf + g_num * g_pdf);
}

inline float_t power_heuristic(int f_num, float_t f_pdf, int g_num, float_t g_pdf)
{
    float_t f = f_num * f_pdf, g = g_num * g_pdf;
    return (f * f) / (f * f + g * g);
}

#pragma endregion

#pragma region sampler

// random number generator
// https://github.com/SmallVCM/SmallVCM/blob/master/src/rng.hxx
class rng_t
{
public:
    // TODO
    rng_t(int seed = 1234) : rng_engine_(seed)
    {
    }

    // [0, int_max]
    int uniform_int()
    {
        return int_dist_(rng_engine_);
    }

    // [0, uint_max]
    uint32_t uniform_uint()
    {
        return uint_dist_(rng_engine_);
    }

    // [0, 1)
    float_t uniform_float()
    {
        return float_dist_(rng_engine_);
    }

    // [0, 1), [0, 1)
    vec2_t uniform_float2()
    {
        return vec2_t(uniform_float(), uniform_float());
    }

private:
    std::mt19937_64 rng_engine_;

    std::uniform_int_distribution<int> int_dist_;
    std::uniform_int_distribution<uint32_t> uint_dist_;
    std::uniform_real_distribution<float_t> float_dist_{ (float_t)0, (float_t)1 };
};


struct camera_sample_t
{
    point2_t p_film{}; // sample point on film
    // point2_t p_lens{};
};


class sampler_t
{
public:
    virtual ~sampler_t() {}

    sampler_t(int samples_per_pixel) :
        samples_per_pixel_{ samples_per_pixel }
    {
    }

    virtual std::unique_ptr<sampler_t> clone() = 0;

public:
    virtual int ge_samples_per_pixel()
    {
        return samples_per_pixel_;
    }
    virtual void set_samples_per_pixel(int samples_per_pixel)
    {
        samples_per_pixel_ = samples_per_pixel;
    }

public:
    virtual void start_pixel()
    {
        current_sample_index_ = 0;
    }
    virtual bool next_sample()
    {
        current_sample_index_ += 1;
        return current_sample_index_ < samples_per_pixel_;
    }

public:
    virtual float_t get_float() = 0;
    virtual vec2_t get_float2() = 0;
    virtual camera_sample_t get_camera_sample(point2_t p_film) = 0;

protected:
    rng_t rng_{};

    int samples_per_pixel_{};
    int current_sample_index_{};
};

class debug_sampler_t : public sampler_t
{
public:
    using sampler_t::sampler_t;

    std::unique_ptr<sampler_t> clone() override
    {
        return std::make_unique<debug_sampler_t>(samples_per_pixel_);
    }

public:
    float_t get_float() override
    {
        return 0.5f;
    }

    vec2_t get_float2() override
    {
        return { 0.5f, 0.5f };
    }

    camera_sample_t get_camera_sample(point2_t p_film) override
    {
        return { p_film + vec2_t{0.5f, 0.5f} };
    }
};

class random_sampler_t : public sampler_t
{
public:
    using sampler_t::sampler_t;

    std::unique_ptr<sampler_t> clone() override
    {
        return std::make_unique<random_sampler_t>(samples_per_pixel_);
    }

public:
    float_t get_float() override
    {
        return rng_.uniform_float();
    }

    vec2_t get_float2() override
    {
        return rng_.uniform_float2();
    }

    // TODO: coroutine
    camera_sample_t get_camera_sample(point2_t p_film) override
    {
        return { p_film + rng_.uniform_float2() };
    }
};

// TODO
class stratified_sampler_t : public sampler_t
{
public:
    camera_sample_t get_camera_sample(point2_t p_film) override
    {
        return { p_film + rng_.uniform_float2() };
    }
};

#pragma endregion



#pragma region shape

/*
     z(0, 0, 1)
          |
          | theta/
          |    /
          |  /
          |/_ _ _ _ _ _ x(1, 0, 0)
         / \
        / phi\
       /       \
      /          \
 y(0, 1, 0)

   https://www.pbr-book.org/3ed-2018/Shapes/Spheres
*/

class shape_t
{
public:
    virtual ~shape_t() = default;

    virtual bool intersect(const ray_t& ray, isect_t* out_isect) const = 0;

    virtual bounds3_t world_bound() const = 0;
    virtual float_t area() const = 0;

public:
    // these methods below only used for `area_light_t`

    // TODO: return position_sample_t
    virtual light_isect_t sample_position(float2_t random, float_t* out_pdf_position) const = 0;


    // TODO: return direction_sample_t
    // default compute `*_direction` by `*_position` 
    virtual light_isect_t sample_direction(const isect_t& isect, float2_t random, float_t* out_pdf_direction) const
    {
        isect_t light_isect = sample_position(random, out_pdf_direction);
        vec3_t wi = light_isect.position - isect.position;

        if (wi.magnitude_squared() == 0)
        {
            *out_pdf_direction = 0;
        }
        else
        {
            wi = normalize(wi);
            // look comments in `pdf_direction()` below
            *out_pdf_direction *= distance_squared(light_isect.position, isect.position) / abs_dot(light_isect.normal, -wi);

            if (std::isinf(*out_pdf_direction))
                *out_pdf_direction = 0.f;
        }

        return light_isect;
    }
    virtual float_t pdf_direction(const isect_t& isect, unit_vec3_t world_wi) const
    {
        ray_t ray = isect.spawn_ray(world_wi);
        isect_t light_isect;

        if (!intersect(ray, &light_isect))
            return 0;

        /*
          convert light sample point to solid angle:

              $$\mathrm{d} \omega = \fact{\mathrm{d} A^{\perp} }{l^{2}} $$

          because:
              unit_solid_angle = 1 / distance_squared(...)
              projected_light_area = abs_dot(...) * area()
              projected_solid_angle = projected_light_area / distance_squared

              pdf = distance_squared(...) / (abs_dot(...) * area()) = inverse_projected_solid_angle
              1 / pdf 
                      = (abs_dot(...) * area()) / distance_squared(...) = projected_solid_angle
                      = (1 / distance_squared(...)) * (abs_dot(...) * area()) = unit_solid_angle * projected_light_area

          so:
              (f * Li * cos_theta) / pdf = f * (Li * cos_theta * projected_solid_angle)

          or:
              (f * Li * cos_theta) / pdf = f * (Li * cos_theta * unit_solid_angle * projected_light_area)
        */
        float_t pdf = distance_squared(isect.position, light_isect.position) / (abs_dot(light_isect.normal, -world_wi) * area());
        if (std::isinf(pdf))
            pdf = 0.f;

        return pdf;
    }

public:
    static constexpr float_t epsilon = 1e-3;// TODO
};

using shape_sptr_t = std::shared_ptr<shape_t>; 
using shape_list_t = std::vector<shape_sptr_t>;



class disk_t : public shape_t
{
public:
    disk_t(point3_t position, normal_t normal, float_t radius):
        position_{ position },
        normal_{ normalize(normal) },
        radius_{ radius }

    {
    }

    bool intersect(const ray_t& ray, isect_t* out_isect) const override
    {
        if (is_equal(dot(ray.direction(), normal_), (float_t)0))
            return false;
 
        const vec3_t op = position_ - ray.origin();
        const float_t distance = dot(normal_, op) / dot(normal_, ray.direction());

        if ((distance > epsilon) && (distance < ray.distance()))
        {
            point3_t hit_point = ray(distance);
            if (::distance(position_, hit_point) <= radius_)
            {
                ray.set_distance(distance);
                *out_isect = isect_t(hit_point, normal_, -ray.direction());

                return true;
            }
        }

        return false;
    }

    bounds3_t world_bound() const override
    {
        frame_t frame{ normal_ };
        vec3_t offset = frame.binormal() * radius_ + frame.tangent() * radius_;
        return bounds3_t(position_ - offset, position_ + offset);
    }

    float_t area() const override { return k_pi * radius_ * radius_; }

public:
    light_isect_t sample_position(float2_t random, float_t* pdf) const override
    {
        isect_t light_isect;

        frame_t frame{ normal_ };
        point2_t sample_point = concentric_disk_sample(random);
        light_isect.position = position_ + radius_ * (frame.binormal() * sample_point.x + frame.tangent() * sample_point.y);

        light_isect.normal = normalize(normal_);

        *pdf = 1 / area();
        return light_isect;
    }

public:
    point3_t position_;
    normal_t normal_;
    float_t radius_;
};

class triangle_t : public shape_t
{
public:
    triangle_t(point3_t p0, point3_t p1, point3_t p2, bool flip_normal = false)
    {
        p0_ = p0;
        p1_ = p1;
        p2_ = p2;

        normal_ = normalize(cross(p1_ - p0_, p2_ - p0_));
        if (flip_normal)
            normal_ = -normal_;
    }

    bool intersect(const ray_t& ray, isect_t* out_isect) const override
    {
        // https://github.com/SmallVCM/SmallVCM/blob/master/src/geometry.hxx#L125-L156

        const vec3_t oa = p0_ - ray.origin();
        const vec3_t ob = p1_ - ray.origin();
        const vec3_t oc = p2_ - ray.origin();

        const vec3_t v0 = cross(oc, ob);
        const vec3_t v1 = cross(ob, oa);
        const vec3_t v2 = cross(oa, oc);

        const float_t v0d = dot(v0, ray.direction());
        const float_t v1d = dot(v1, ray.direction());
        const float_t v2d = dot(v2, ray.direction());

        if (((v0d <  0.f) && (v1d <  0.f) && (v2d <  0.f)) ||
            ((v0d >= 0.f) && (v1d >= 0.f) && (v2d >= 0.f)))
        {
            // 1. first calculate the vertical distance from ray.origin to the plane,
            //    by `dot(normal, op)` (or `bo`, `co`)
            // 2. then calculate the distance from ray.origin to the plane alone ray.direction, 
            //    by `distance * dot(normal, ray.direction()) = vertical_distance`
            const float_t distance = dot(normal_, oa) / dot(normal_, ray.direction());

            if ((distance > epsilon) && (distance < ray.distance()))
            {
                ray.set_distance(distance);
                point3_t hit_point = ray(distance);
                *out_isect = isect_t(hit_point, normal_, -ray.direction());

                return true;
            }
        }

        return false;
    }

    bounds3_t world_bound() const override
    {
        return bounds3_t(p0_, p1_).join(p2_);
    }

    float_t area() const override { return 0.5 * cross(p1_ - p0_, p2_ - p0_).magnitude(); }

public:
    light_isect_t sample_position(float2_t random, float_t* pdf) const override
    {
        point2_t b = uniform_triangle_sample(random);

        isect_t light_isect;
        light_isect.position = b.x * p0_ + b.y * p1_ + (1 - b.x - b.y) * p2_;
        light_isect.normal = normal_;

        *pdf = 1 / area();
        return light_isect;
    }

public:
    point3_t p0_;
    point3_t p1_;
    point3_t p2_;
    normal_t normal_;
};

class rectangle_t : public shape_t
{
public:
    rectangle_t(point3_t p0, point3_t p1, point3_t p2, point3_t p3, bool flip_normal = false)
    {
        p0_ = p0;
        p1_ = p1;
        p2_ = p2;
        p3_ = p3;
        // TODO: CHECK_DEBUG

        normal_ = normalize(cross(p1_ - p0_, p2_ - p0_));
        if (flip_normal)
            normal_ = -normal_;
    }

    bool intersect(const ray_t& ray, isect_t* out_isect) const override
    {
        // https://github.com/SmallVCM/SmallVCM/blob/master/src/geometry.hxx#L125-L156

        const vec3_t oa = p0_ - ray.origin();
        const vec3_t ob = p1_ - ray.origin();
        const vec3_t oc = p2_ - ray.origin();
        const vec3_t od = p3_ - ray.origin();

        const vec3_t v0 = cross(oc, ob);
        const vec3_t v1 = cross(ob, oa);
        const vec3_t v2 = cross(oa, od);
        const vec3_t v3 = cross(od, oc);

        const float_t v0d = dot(v0, ray.direction());
        const float_t v1d = dot(v1, ray.direction());
        const float_t v2d = dot(v2, ray.direction());
        const float_t v3d = dot(v3, ray.direction());

        if (((v0d <  0.f) && (v1d <  0.f) && (v2d <  0.f) && (v3d <  0.f)) ||
            ((v0d >= 0.f) && (v1d >= 0.f) && (v2d >= 0.f) && (v3d >= 0.f)))
        {
            const float_t distance = dot(normal_, oa) / dot(normal_, ray.direction());

            if ((distance > epsilon) && (distance < ray.distance()))
            {
                ray.set_distance(distance);
                point3_t hit_point = ray(distance);
                normal_t normal = dot(normal_, ray.direction()) <= 0 ? normal_ : -normal_;
                *out_isect = isect_t(hit_point, normal, -ray.direction());

                return true;
            }
        }

        return false;
    }

    bounds3_t world_bound() const override
    {
        return bounds3_t(p0_, p1_).join(p2_).join(p3_);
    }

    float_t area() const override { return cross(p0_ - p1_, p2_ - p1_).magnitude(); }

public:
    light_isect_t sample_position(float2_t random, float_t* pdf) const override
    {
        isect_t light_isect;
        light_isect.position = p1_ + (p0_ - p1_) * random[0] + (p2_ - p1_) * random[1];
        light_isect.normal = normalize(normal_);

        *pdf = 1 / area();
        return light_isect;
    }

public:
    point3_t p0_;
    point3_t p1_;
    point3_t p2_;
    point3_t p3_;
    normal_t normal_;
};

class sphere_t : public shape_t
{
public:
    sphere_t(vec3_t center, float_t radius) :
        center_(center),
        radius_(radius),
        radius_sq_(radius * radius)
    {
    }

    bool intersect(const ray_t& ray, isect_t* out_isect) const override
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
               = neg_b' +/- sqrt(discr)
        */

        vec3_t oc = center_ - ray.origin();
        float_t neg_b = dot(oc, ray.direction());
        float_t discr = neg_b * neg_b - dot(oc, oc) + radius_sq_;

        float_t distance = 0;
        bool hit = false;
        if (discr >= 0)
         {
            float_t sqrt_discr = sqrt(discr);

            if (distance = neg_b - sqrt_discr; distance > epsilon && distance < ray.distance())
            {
                hit = true;
            }
            else if (distance = neg_b + sqrt_discr; distance > epsilon && distance < ray.distance())
            {
                hit = true;
            }
        }

        if (hit)
        {
            ray.set_distance(distance);
            point3_t hit_point = ray(distance);
            *out_isect = isect_t(hit_point, (hit_point - center_).normalize(), -ray.direction());
        }

        return hit;
    }

    bounds3_t world_bound() const override
    {
        vec3_t half(radius_, radius_, radius_);
        return bounds3_t(center_ + half, center_ - half);
    }

    float_t area() const override { return 4 * k_pi * radius_sq_; }

public:
    light_isect_t sample_position(float2_t random, float_t* pdf) const override
    {
        unit_vec3_t direction = uniform_sphere_sample(random);
        point3_t position = center_ + radius_ * direction;

        isect_t light_isect;
        light_isect.position = position;
        light_isect.normal = normalize(direction);

        *pdf = 1 / area();

        return light_isect;
    }

    // TODO: confirm
    light_isect_t sample_direction(const isect_t& isect, float2_t random, float_t* pdf) const override
    {
        if (distance_squared(isect.position, center_) <= radius_ * radius_)
        {
            isect_t light_isect = sample_position(random, pdf);
            vec3_t wi = light_isect.position - isect.position;

            if (wi.magnitude_squared() == 0)
                *pdf = 0;
            else
            {
                // convert from area measure returned by Sample() call above to solid angle measure.
                wi = normalize(wi);
                *pdf *= distance_squared(light_isect.position, isect.position) / abs_dot(isect.normal, -wi);
            }

            if (std::isinf(*pdf))
                *pdf = 0.f;

            return light_isect;
        }

        // sample sphere uniformly inside subtended cone

        /*
                /         _
               /        / O \
              /         O O O (a sphere)
             /       .  \ O /
            /    .
           / .     theta
          . _ _ _ _ _ _ _ _

        */

        float_t dist = distance(isect.position, center_);
        float_t inv_dist = 1 / dist;

        // compute $\theta$ and $\phi$ values for sample in cone
        float_t sin_theta_max = radius_ * inv_dist;
        float_t sin_theta_max_sq = sin_theta_max * sin_theta_max;
        float_t inv_sin_theta_max = 1 / sin_theta_max;
        float_t cos_theta_max = std::sqrt(std::max((float_t)0.f, 1 - sin_theta_max_sq));

        float_t cos_theta = (cos_theta_max - 1) * random[0] + 1;
        float_t sin_theta_sq = 1 - cos_theta * cos_theta;

        if (sin_theta_max_sq < 0.00068523f /* sin^2(1.5 deg) */)
        {
            /* fall back to a Taylor series expansion for small angles, where
               the standard approach suffers from severe cancellation errors */
            sin_theta_sq = sin_theta_max_sq * random[0];
            cos_theta = std::sqrt(1 - sin_theta_sq);
        }

        // compute angle $\alpha$ from center of sphere to sampled point on surface
        float_t cos_alpha = sin_theta_sq * inv_sin_theta_max +
            cos_theta * std::sqrt(std::max((float_t)0.f, 1.f - sin_theta_sq * inv_sin_theta_max * inv_sin_theta_max));
        float_t sin_alpha = std::sqrt(std::max((float_t)0.f, 1.f - cos_alpha * cos_alpha));
        float_t phi = random[1] * 2 * k_pi;

        // compute coordinate system for sphere sampling
        vec3_t normal = (center_ - isect.position) * inv_dist;
        frame_t frame{ normal };

        // compute surface normal and sampled point on sphere
        vec3_t world_normal =
            spherical_to_direction(sin_alpha, cos_alpha, phi, -frame.binormal(), -frame.tangent(), -frame.normal());
        point3_t world_position = center_ + radius_ * point3_t(world_normal.x, world_normal.y, world_normal.z);

        isect_t light_isect;
        light_isect.position = world_position;
        light_isect.normal = world_normal;

        // uniform cone PDF.
        *pdf = 1 / (2 * k_pi * (1 - cos_theta_max));

        return light_isect;
    }

    float_t pdf_direction(const isect_t& isect, vec3_t world_wi) const override
    {
        // return uniform PDF if point is inside sphere
        if (distance_squared(isect.position, center_) <= radius_ * radius_)
            return shape_t::pdf_direction(isect, world_wi);

        // compute general sphere PDF
        float_t sin_theta_max_sq = radius_ * radius_ / distance_squared(isect.position, center_);
        float_t cos_theta_max = std::sqrt(std::max((float_t)0, 1 - sin_theta_max_sq));
        return uniform_cone_pdf(cos_theta_max);
    }

private:
    vec3_t center_;
    float_t radius_;
    float_t radius_sq_;
};

#pragma endregion



#pragma region filter

#pragma endregion

#pragma region film

enum class image_enum_t
{
    ppm,
    bmp,
    hdr
};

// film_option_t
struct film_desc_t
{
    int width;
    int height;
};

constexpr float_t clamp01(float_t x) { return std::clamp(x, (float_t)0, (float_t)1); }
inline color_t clamp01(color_t c) { return color_t(clamp01(c.r), clamp01(c.g), clamp01(c.b)); }

inline uint8_t gamma_encoding(float_t x) { return pow(clamp01(x), 1 / 2.2) * 255 + .5; }

// warpper of `color_t pixels[]`
class film_t : public nocopyable_t
{
public:
    film_t(int width, int height) :
        width_{ width },
        height_{ height },
        pixels_{ std::make_unique<color_t[]>(get_pixel_num()) }
    {
    }

public:
    int get_width() const { return width_; }
    int get_height() const { return height_; }
    int get_pixel_num() const { return width_ * height_; }
    int get_channels() const { return 3; }
    
    virtual vec2_t get_resolution() const { return { (float_t)width_, (float_t)height_ }; }
    virtual color_t& operator()(int x, int y)
    {
        CHECK_DEBUG(x >= 0 && x < width_ && y >= 0 && y < height_, 
            "out of bound: {}, {}", x, y);
        return *(pixels_.get() + get_width() * y + x);
    }

    void set_color(int x, int y, color_t color)
    {
        operator()(x, y) = color;
    }
    void clear_color(int x, int y)
    {
        set_color(x, y, color_t{});
    }

    void add_color(int x, int y, color_t delta)
    {
        color_t& color = operator()(x, y);
        color = color + delta;
    }

    void clear(color_t color)
    {
        for (int i = 0; i < get_pixel_num(); ++i)
        {
            pixels_[i] = color;
        }
    }

public:

#pragma region store

    // TODO
    //add another virtual bool store_image();
    virtual bool store_image(std::string filename /*bool with_alpha = false*/) const
    {
        CHECK_DEBUG(get_channels() == 3, "Now only support RGB format");

        std::string command{};
#ifdef KY_OUTPUT_HDR
        store_hdr_impl(filename += ".hdr", get_width(), get_height(), get_channels(), (float_t*)pixels_.get());
        // https://github.com/Tom94/tev
        command = "tev " + filename;
#else
        store_bmp_impl(filename += ".bmp", get_width(), get_height(), get_channels(), (float_t*)pixels_.get());
        command = "mspaint " + filename;
#endif

#ifdef KY_WINDOWS
        system(command.c_str());
        /*
        std::thread([]()
        {
            system("mspaint single.bmp");
        })
        .detach();
        */
#endif

        return true;

        /*
        switch (image_type)
        {
        case image_enum_t::ppm:
            return store_ppm_impl(filename, get_width(), get_height(), get_channels(), (float_t*)pixels_.get());
        case image_enum_t::bmp:
            return store_bmp_impl(filename, get_width(), get_height(), get_channels(), (float_t*)pixels_.get());
        default:
            break;
        }
        */
    }

    static bool store_ppm_impl(const std::string& filename, int width, int height, int channel, const float_t* floats)
    {
        std::fstream img_file(filename, std::ios::binary | std::ios::out);

        img_file << std::format("P3\n{} {}\n{}\n", width, height, 255);

        int float_num = width * height * channel;
        for (int index = 0; index < float_num; ++index)
        {
            img_file << std::format("{} ", gamma_encoding(floats[index]));
        }

        return true;
    }
   
    static bool store_bmp_impl(const std::string& filename, int width, int height, int channel, const float_t* floats)
    {
        // https://github.com/SmallVCM/SmallVCM/blob/master/src/framebuffer.hxx#L149-L215
        // https://github.com/skywind3000/RenderHelp/blob/master/RenderHelp.h#L937-L1018

        std::fstream img_file(filename, std::ios::binary | std::ios::out);


        // 1.write file header & 2.write info header

        uint32_t padding_line_bytes = (width * channel + 3) & (~3);
        uint32_t padding_image_bytes = padding_line_bytes * height;

        const uint32_t FILE_HEADER_SIZE = 14;
        const uint32_t INFO_HEADER_SIZE = 40;

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


        // 3.without color table


        // 4.write data body 

        // gamma encoding
        int byte_num = width * height * channel;
        auto bytes = std::make_unique<uint8_t[]>(byte_num);
        for (int i = 0; i < byte_num; i += 3)
        {
            // BGR
            bytes[i]     = gamma_encoding(floats[i + 2]);
            bytes[i + 1] = gamma_encoding(floats[i + 1]);
            bytes[i + 2] = gamma_encoding(floats[i]);
        }

        int line_num = width * channel;
        // bmp is stored from bottom to up
        for (int y = height - 1; y >= 0; --y)
            img_file.write((const char*)(bytes.get() + y * line_num), line_num);


        return true;
    }

    static bool store_hdr_impl(const std::string& filename, int width, int height, int channel, const float_t* floats)
    {
        // https://github.com/SmallVCM/SmallVCM/blob/master/src/framebuffer.hxx#L218-L251

        std::ofstream img_file(filename, std::ios::binary | std::ios::out);

        img_file << std::format(
            "#?RADIANCE\n"
            "FORMAT=32-bit_rle_rgbe\n\n"
            "-Y {} +X {}\n", height, width);

        color_t* pixels = (color_t*)floats;
        int pixel_num = width * height;
        for (int index = 0; index < pixel_num; index++)
        {
            uint8_t rgbe[4]{};

            color_t color = pixels[index];
            float v = std::max({ color.r, color.g, color.b });

            if (v >= 1e-32f)
            {
                /*
                   write:
                        v = m * 2 ^ e ( 0 < m < 1)
                        r = R * m * 256.0/v
                   read:
                        R = r * 2^(e – 128 - 8);
                */

                int e;
                float m = float_t(frexp(v, &e) * 256.f / v);

                rgbe[0] = uint8_t(color.r * m);
                rgbe[1] = uint8_t(color.g * m);
                rgbe[2] = uint8_t(color.b * m);
                rgbe[3] = uint8_t(e + 128);
            }

            img_file.write((const char*)&rgbe[0], 4);
        }

        return true;
    }

#pragma endregion

private:
    int32_t width_{};
    int32_t height_{};

    std::unique_ptr<color_t[]> pixels_{};
};

/*
   put multi sub-film together in one film, for export multi images at once
   A_mn = 
       a_11, a_12 ... a_1n
       a_21, a_22 ... a_2n
       ...     ...     ...
       a_m1, a_m2 ... a_mn
   where a_ij is a sub-film, m/n is specified by row/column
*/
class film_grid_t : public film_t
{
public:
    film_grid_t(int row, int column, int sub_width, int sub_height) :
        film_t(column * sub_width, row * sub_height),
        row_{ row },
        column_{ column },
        sub_width_{ sub_width },
        sub_height_{ sub_height }
    {
    }

public:
    vec2_t get_resolution() const override { return { (float_t)sub_width_, (float_t)sub_height_ }; }

    color_t& operator()(int x, int y) override
    {
        int col_index = subfilm_index % column_;
        int row_index = subfilm_index / column_;
        return film_t::operator()(x + col_index * sub_width_ , y + row_index * sub_height_);
    }

    void next_subfilm()
    {
        ++subfilm_index;
    }

private:
    int row_{};
    int column_{};
    int subfilm_index{};

    int sub_width_{};
    int sub_height_{};
};

#pragma endregion

#pragma region camera

/*
  camera space:

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

class camera_t
{
public:
    virtual ~camera_t() {}

    camera_t(
        vec3_t position, vec3_t front, vec3_t up,
        degree_t fov, vec2_t resolution):
        position_{ position },
        front_{ front.normalize() },
        up_{ up.normalize()},
        resolution_{ resolution }
    {
        // TODO
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/transform.cpp#L394-L397
 
        float_t tan_fov = std::tan(radians(fov) / 2);

        // left hand, clockwise
        right_ = up_.cross(front_).normalize() * tan_fov * get_aspect();
        up_ = front_.cross(right_).normalize() * tan_fov;
    }

public:
    // generate primary ray from camera
    virtual ray_t generate_ray(const camera_sample_t& sample) const
    {
        vec3_t direction =
            front_ +
            right_ * (sample.p_film.x / resolution_.x - 0.5) +
               up_ * (0.5 - sample.p_film.y / resolution_.y);

        return ray_t{ position_, direction.normalize() };
    }

    // sample_ray(...)

private:
    float_t get_aspect() { return resolution_.x / resolution_.y; }

private:
    vec3_t position_;
    unit_vec3_t front_;
    unit_vec3_t right_;
    unit_vec3_t up_;

    vec2_t resolution_;
};

using const_camera_sptr_t = std::shared_ptr<const camera_t>;

#pragma endregion



#pragma region bsdf_utility

// the functions below are based on local shading coordinate

inline float_t cos_theta(vec3_t w) { return w.z; }
inline float_t abs_cos_theta(vec3_t w) { return std::abs(w.z); }

inline bool same_hemisphere(vec3_t w, vec3_t wp) { return w.z * wp.z > 0; }

inline vec3_t reflect(vec3_t wo, normal_t normal)
{
    // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#SpecularReflection

    return -wo + 2 * dot(wo, normal) * normal;
}

// eta = eta_i/eta_t
inline bool refract(vec3_t wi, normal_t normal, float_t eta, vec3_t* out_wt)
{
    // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#SpecularTransmission
    // https://github.com/mmp/pbrt-v3/blob/master/src/core/reflection.h#L97-L109

    // compute $\cos \theta_\mathrm{t}$ using Snell's law
    float_t cos_theta_i = dot(normal, wi);
    float_t sin_theta_i_sq = std::max(float_t(0), float_t(1 - cos_theta_i * cos_theta_i));
    float_t sin_theta_t_sq = eta * eta * sin_theta_i_sq;

    if (sin_theta_t_sq >= 1)
        return false; // handle total internal reflection for transmission

    float_t cos_theta_t = std::sqrt(1 - sin_theta_t_sq);
    *out_wt = eta * -wi + (eta * cos_theta_i - cos_theta_t) * vec3_t(normal);

    CHECK_DEBUG(out_wt->is_valid() && !out_wt->is_zero());
    return true;
}

#pragma endregion

#pragma region fresnel

float_t fresnel_dielectric(
    float_t cos_theta_i, 
    float_t eta_i, float_t eta_t)
{
    // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#FresnelReflectance
    // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.cpp#L66-L90

    cos_theta_i = std::clamp(cos_theta_i, (float_t)-1, (float_t)1);

    bool entering = cos_theta_i > 0.f;
    if (!entering)
    {
        std::swap(eta_i, eta_t);
        cos_theta_i = std::abs(cos_theta_i);
    }


    // compute $\cos \theta_\mathrm{t}$ using Snell's law
    float_t sin_theta_i = std::sqrt(std::max((float_t)0, 1 - cos_theta_i * cos_theta_i));
    float_t sin_theta_t = eta_i / eta_t * sin_theta_i;

    // Handle total internal reflection
    if (sin_theta_t >= 1)
        return 1;

    float_t cos_theta_t = std::sqrt(std::max((float_t)0, 1 - sin_theta_t * sin_theta_t));


    float_t r_para = ((eta_t * cos_theta_i) - (eta_i * cos_theta_t)) /
                   ((eta_t * cos_theta_i) + (eta_i * cos_theta_t));
    float_t r_perp = ((eta_i * cos_theta_i) - (eta_t * cos_theta_t)) /
                   ((eta_i * cos_theta_i) + (eta_t * cos_theta_t));
    return (r_para * r_para + r_perp * r_perp) / 2;
}

#pragma region schlick approximation 1994

float_t fresnel_dielectric_schlick(
    float_t cos_theta_i, float_t cos_theta_t, 
    float_t eta_i, float_t eta_t)
{
    /*
    cos_theta_i = std::clamp(cos_theta_i, -1.0, 1.0);
    cos_theta_t = std::clamp(cos_theta_t, -1.0, 1.0);

    bool entering = cos_theta_i > 0.f;
    if (!entering)
    {
        std::swap(eta_i, eta_t);
        cos_theta_i = std::abs(cos_theta_i);
        cos_theta_t = std::abs(cos_theta_t);
    }
    */

    float_t F0 = (eta_t - eta_i) / (eta_t + eta_i);
    F0 *= F0;

    //float_t cos_i = eta_i < eta_t ? cos_theta_i : cos_theta_t;
    float_t cos_i = cos_theta_i < 0 ? -cos_theta_i : cos_theta_t;

    return lerp(F0, 1.0f, std::pow(1 - cos_i, 5.0f) );
}

float_t fresnel_dielectric_schlick(
    float_t cos_theta_i,
    float_t eta_i, float_t eta_t)
{
    float_t F0 = (eta_t - eta_i) / (eta_t + eta_i);
    F0 *= F0;

    return lerp(F0, 1.0f, std::pow(1 - cos_theta_i, 5.0f));
}

/*
   given
     * the cosine of the incidence angle `cos_theta_i`,
     * the fresnel reflectance at normal incidence `F0`
   compute reflectance
*/
float_t fresnel_dielectric_schlick(float_t cos_theta_i, float_t F0)
{
    return lerp(F0, 1.0f, std::pow((1.0f - cos_theta_i), 5.0f));
}

/*
color_t fresnel_dielectric_schlick(float_t cos_theta_i, color_t F0)
{
    return lerp(F0, vec3_t(1.0f), std::pow((1.0f - cos_theta_i), 5.0f));
}
*/

#pragma endregion

/*
class fresnel_t
{
public:
    virtual ~fresnel_t() = default;

    virtual float_t evaluate(float_t cosI) const = 0;
};

class fresnel_dielectric_t : public fresnel_t
{
public:
    fresnel_dielectric_t()
    {
    }

    float_t evaluate(float_t cosI) const override
    {
        return 0;
    }
};

// class fresnel_dummy_t : public fresnel_t
*/

#pragma endregion

#pragma region bsdf

/*
   reference:
     * LuxCoreRender Materials https://wiki.luxcorerender.org/LuxCoreRender_Materials
     * Shader — Blender Manual https://docs.blender.org/manual/en/latest/render/shader_nodes/shader/index.html
     * BSDFs - Mitsuba 3 https://mitsuba.readthedocs.io/en/latest/src/generated/plugins_bsdfs.html
*/

enum class bsdf_enum_t
{
    none = 0,
    reflection = 1,
    transmission = 2,
    scattering = reflection | transmission,

    diffuse = 4,
    glossy = 8,
    specluar = 16,
};
KY_ENUM_OPERATORS(bsdf_enum_t)

inline bool is_delta_bsdf(bsdf_enum_t bsdf_type)
{
    return enum_have(bsdf_type, bsdf_enum_t::specluar);
}

/* 
  local shading frame:

      z/n(0, 0, 1)
       |
       |
       |
       |
       |_ _ _ _ _ _ x/s(1, 0, 0)
      / p
     /
    /
  y/t(0, 1, 0)

  prev   n   light
  ----   ^   -----
    ^    |    ^
     \   | θ /
   wo \  |  / wi is unknown, sampling from bsdf or light
       \ | /
        \|/
      -------
       isect

   https://www.pbr-book.org/3ed-2018/Reflection_Models#x0-GeometricSetting
*/

struct bsdf_sample_t
{
    color_t f{}; // scattering rate 
    vec3_t wi{}; // world wi
    float_t pdf{};
    bsdf_enum_t bsdf_type{}; // flags
};

class bsdf_t
{
public:
    virtual ~bsdf_t() = default;

    bsdf_t(frame_t shading_frame) :
        shading_frame_{ shading_frame }
    {
    }

public:
    virtual bool is_delta() const = 0;

    // or called `f()`, `evaluate()`
    color_t eval(vec3_t world_wo, vec3_t world_wi) const
    {
        return eval_(to_local(world_wo), to_local(world_wi));
    }

    float_t pdf(vec3_t world_wo, vec3_t world_wi) const
    {
        return pdf_(to_local(world_wo), to_local(world_wi));
    }

    // or called `sample_f()`, `sample_direction()`, `sample_solid_angle()`
    bsdf_sample_t sample(vec3_t world_wo, float2_t random) const
    {
        bsdf_sample_t sample = sample_(to_local(world_wo), random);
        sample.wi = to_world(sample.wi); // <--- ATTENTION!!!

        return sample;
    }

    std::tuple<color_t, float_t> eval_and_pdf(vec3_t world_wo, vec3_t world_wi) const
    {
        vec3_t wo = to_local(world_wo), wi = to_local(world_wi);

        return { eval_(wo, wi), pdf_(wo, wi) };
    }

protected: 
    virtual color_t eval_(vec3_t wo, vec3_t wi) const = 0;
    virtual float_t pdf_(vec3_t wo, vec3_t wi) const = 0;

    virtual bsdf_sample_t sample_(vec3_t wo, float2_t random) const = 0;

private:
    vec3_t to_local(vec3_t world_vec3) const
    {
        return shading_frame_.to_local(world_vec3);
    }

    vec3_t to_world(vec3_t local_vec3) const
    {
        return shading_frame_.to_world(local_vec3);
    }

private:
    frame_t shading_frame_;

    // extension point:
    // std::array<bxdf_uptr, 2> bxdf_list_;
};



/*
   ideal diffuse reflection
*/
class lambertion_reflection_t : public bsdf_t
{
public:
    lambertion_reflection_t(const frame_t& shading_frame, color_t albedo) :
        bsdf_t(shading_frame), albedo_{ albedo }
    {
    }

    bool is_delta() const override { return false; }

    color_t eval_(vec3_t wo, vec3_t wi) const override
    {
        // TODO: confirm
        if (!same_hemisphere(wo, wi))
            return color_t{};

        // lambertion surface's albedo divided by $\pi$ is surface bidirectional reflectance
        return albedo_ * k_inv_pi;
    }

    float_t pdf_(vec3_t wo, vec3_t wi) const override
    {
        return same_hemisphere(wo, wi) ? cosine_hemisphere_pdf(abs_cos_theta(wi)) : 0;
    }

    bsdf_sample_t sample_(vec3_t wo, float2_t random) const override
    {
        bsdf_sample_t sample;

        // cosine-sample the hemisphere, flipping the direction if necessary
        sample.wi = cosine_hemisphere_sample(random);
        if (wo.z < 0) 
            sample.wi.z *= -1;

        sample.f = eval_(wo, sample.wi);
        sample.pdf = pdf_(wo, sample.wi);
        sample.bsdf_type = bsdf_enum_t::reflection | bsdf_enum_t::diffuse;

        CHECK_DEBUG(sample.f.is_valid());
        return sample;
    }

private:
    /*
       https://wiki.luxcorerender.org/LuxCoreRender_Materials_Matte
       https://mitsuba.readthedocs.io/en/latest/src/generated/plugins_bsdfs.html#smooth-diffuse-material-diffuse

       surface directional-hemispherical reflectance, usually called `albedo`
       symbol: $\rho_{\mathrm{hd}}$
    */
    color_t albedo_{};
};



/*
   ideal specular reflection, ignore fresnel effect,
   only suitable for some metal materials
    
   as a delta bsdf, it's `eval(...), pdf(...) sample(...)` functions requires special processing,
   same to `fresnel_specular_scattering_t`
*/
class perfect_specular_reflection_t : public bsdf_t
{
public:
    perfect_specular_reflection_t(const frame_t& shading_frame, color_t reflectance) :
        bsdf_t(shading_frame), reflectance_{ reflectance }
    {
    }

    bool is_delta() const override { return true; }

    color_t eval_(vec3_t wo, vec3_t wi) const override { return color_t(); }
    float_t pdf_(vec3_t wo, vec3_t wi) const override { return 0; }

    bsdf_sample_t sample_(vec3_t wo, float2_t random) const override
    {
        // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#SpecularReflection
        // https://github.com/infancy/pbrt-v3/blob/master/src/materials/mirror.cpp#L45-L57  mirror material use `FresnelNoOp`
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.h#L387-L408   class SpecularReflection;
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.cpp#L181-L191 SpecularReflection::Sample_f(...)
        
        bsdf_sample_t sample; 
        sample.wi = vec3_t(-wo.x, -wo.y, wo.z); // sample.wi = reflect(wo, vec3_t(0, 0, 1));
        sample.f = reflectance_ / abs_cos_theta(sample.wi); // (f / cos_theta) * Li * cos_theta / pdf => f * Li
        sample.pdf = 1;
        sample.bsdf_type = bsdf_enum_t::reflection | bsdf_enum_t::specluar;

        CHECK_DEBUG(sample.f.is_valid());
        return sample;
    }

private:
    /*
       https://wiki.luxcorerender.org/LuxCoreRender_Materials_Mirror

       optional factor that can be used to modulate the specular reflection component. 
    */
    color_t reflectance_{};
};



/*
   https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular%20transmission%20projections.svg

   ray            N
    *             |             *
       *     θ_i  |          *
          *       |       *
             *    |    *            outside ior: eta_i
                * | *
    - - - - - - - - - - - - - - - - interface
                  |*
                  | *               inside ior:  eta_t
                  |  *
                  |   *
                  |    *
                  | θ_t *
*/
class fresnel_specular_scattering_t : public bsdf_t
{
public:
    fresnel_specular_scattering_t(
    const frame_t& shading_frame, float_t eta_i, float_t eta_t, color_t reflectance, color_t transmittance) :
        bsdf_t(shading_frame),
        eta_i_{ eta_i },
        eta_t_{ eta_t },
        reflectance_{ reflectance },
        transmittance_{ transmittance }
    {
    }

    bool is_delta() const override { return true; }

    color_t eval_(vec3_t wo, vec3_t wi) const override { return color_t(); }
    float_t pdf_(vec3_t wo, vec3_t wi) const override { return 0; }

    bsdf_sample_t sample_(vec3_t wo, float2_t random) const override
    {
        // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#FresnelReflectance
        // 
        // https://github.com/infancy/pbrt-v3/blob/master/src/materials/glass.cpp#L64-L69   full smooth glass
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.h#L440-L463   class FresnelSpecular;
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.cpp#L627-L667 FresnelSpecular::Sample_f(...)


        bsdf_sample_t sample;

        // percentage of light's reflect and refract
        float_t reflect_percent = fresnel_dielectric(cos_theta(wo), eta_i_, eta_t_);
        float_t refract_percent = 1 - reflect_percent;

        // and probability of single ray is reflect or refract
        float_t Pr_reflect = reflect_percent, Pr_refract = refract_percent;

        // Russian roulette
        if (random[0] < Pr_reflect)
        {
            // specular reflection

            sample.wi = vec3_t(-wo.x, -wo.y, wo.z);

            sample.pdf = Pr_reflect;
            sample.f = (reflectance_ * reflect_percent) / abs_cos_theta(sample.wi);
            sample.bsdf_type = bsdf_enum_t::reflection | bsdf_enum_t::specluar;

            CHECK_DEBUG(sample.f.is_valid());
        }
        else
        {
            // specular refract/transmission

            normal_t normal(0, 0, 1); // use `z` as normal
            bool into = normal.dot(wo) > 0; // ray from outside going in?

            normal_t wo_normal = into ? normal : normal * -1;
            float_t eta = into ? eta_i_ / eta_t_ : eta_t_ / eta_i_;

            if (refract(wo, wo_normal, eta, &sample.wi))
            {
                sample.pdf = Pr_refract;
                sample.f = (transmittance_ * refract_percent) / abs_cos_theta(sample.wi);
                sample.bsdf_type = bsdf_enum_t::transmission | bsdf_enum_t::specluar;

                CHECK_DEBUG(sample.f.is_valid());
            }
            else
            {
                sample.f = color_t(); // total internal reflection
            }
        }

        return sample;
    }

    /*
    // smallpt version
    color_t sample_(vec3_t wo, float2_t random,
        vec3_t* out_wi, float_t* out_pdf_direction, bsdf_enum_t* out_bsdf_type) const override
    {
        normal_t normal(0, 0, 1);
        bool into = normal.dot(wo) > 0; // ray from outside going in?

        normal_t wo_normal = into ? normal : normal * -1;
        float_t eta = into ? etaI_ / etaT_ : etaT_ / etaI_;

        if (!refract(wo, wo_normal, eta, out_wi))
        {
            return color_t(); // total internal reflection
        }

        float_t cos_theta_a = wo.dot(wo_normal);
        float_t cos_theta_b = (*out_wi).dot(normal);
        float_t cos_theta_i = into ? cos_theta_a : cos_theta_b;

        float_t Re = fresnel_dielectric_schlick(cos_theta_a, etaI_, etaT_);
        float_t Tr = 1 - Re;

        if (random[0] < Re)
        {
            // Compute specular reflection for _FresnelSpecular_

            *out_wi = vec3_t(-wo.x, -wo.y, wo.z);
            *out_pdf_direction = Re; // Russian roulette???

            return (Re * R_) / abs_cos_theta(*out_wi);
        }
        else
        {
            // Compute specular transmission for _FresnelSpecular_

            *out_pdf_direction = Tr;
            return (T_ * Tr) / abs_cos_theta(*out_wi);
        }
    }
    */

private:
    // outside and inside ior of interface
    float_t eta_i_{}; // ior_i_
    float_t eta_t_{}; // ior_t_

    /*
       https://wiki.luxcorerender.org/LuxCoreRender_Materials_Glass
       https://mitsuba.readthedocs.io/en/latest/src/generated/plugins_bsdfs.html#smooth-dielectric-material-dielectric

       optional factor that can be used to modulate the specular reflection/transmission component. 
    */
    color_t reflectance_{};
    color_t transmittance_{};
};



/*
   physically based(energy conservation) Phong specular reflection model
   Lafortune and Willems, “Using the modified Phong reflectance model for physically based rendering”, Technical Report http://graphics.cs.kuleuven.be/publications/Phong/
*/
class phong_specular_reflection_t : public bsdf_t
{
public:
    phong_specular_reflection_t(const frame_t& shading_frame, /*color_t Kd,*/ color_t specular_reflectance, float_t exponent) :
        bsdf_t(shading_frame),
        specular_reflectance_{ specular_reflectance },
        exponent_{ exponent }
    {
    }

    bool is_delta() const override { return false; }

    color_t eval_(vec3_t wo, vec3_t wi) const override
    {
        // TODO: confirm
        if (!same_hemisphere(wo, wi))
            return color_t{};

        const vec3_t wr = reflect(wo, vec3_t(0, 0, 1));
        const float_t cos_alpha = dot(wr, wi);

        const color_t rho = specular_reflectance_ * (exponent_ + 2.f) * k_inv_2pi;
        return rho * std::pow(cos_alpha, exponent_);
    }

    float_t pdf_(vec3_t wo, vec3_t wi) const override
    {
        const vec3_t wr = reflect(wo, vec3_t(0, 0, 1));
        //const float_t cos_alpha = dot(wr, wi);

        return cosine_hemisphere_pdf_phong(wr, wi);
    }

    bsdf_sample_t sample_(vec3_t wo, float2_t random) const override
    {
        bsdf_sample_t sample;

        // TODO
        sample.wi = cosine_hemisphere_sample_phong(random);

        const vec3_t wr = reflect(wo, vec3_t(0, 0, 1));
        frame_t frame{ wr };
        sample.wi = frame.to_world(sample.wi);

        if (wo.z < 0)
            sample.wi.z *= -1;

        sample.f = eval_(wo, sample.wi);
        sample.pdf = pdf_(wo, sample.wi);
        sample.bsdf_type = bsdf_enum_t::reflection | bsdf_enum_t::glossy;
        
        return sample;
    }

private:
    // cosine lobe hemisphere sampling
    vec3_t cosine_hemisphere_sample_phong(vec2_t random) const
    {
        const float_t phi = 2.f * k_pi * random[0];
        const float_t cos_theta = std::pow(random[1], 1.f / (exponent_ + 1.f));
        const float_t sin_theta = std::sqrt(1.f - cos_theta * cos_theta);

        return vec3_t(
            std::cos(phi) * sin_theta,
            std::sin(phi) * sin_theta,
            cos_theta);
    }

    float_t cosine_hemisphere_pdf_phong(
        vec3_t aNormal, vec3_t aDirection) const
    {
        const float_t cosTheta = std::max(0.f, dot(aNormal, aDirection));
        return (exponent_ + 1.f) * std::pow(cosTheta, exponent_) * k_inv_2pi;
    }

private:
    color_t specular_reflectance_{};
    float_t exponent_{};
};

#pragma endregion

#pragma region texture

// TODO
// class texture_t

#pragma endregion

#pragma region material

class material_t
{
public:
    virtual ~material_t() = default;

    virtual bsdf_uptr_t scattering(const isect_t& isect) const = 0;
};

using material_sptr_t = std::shared_ptr<material_t>;
using material_list_t = std::vector<material_sptr_t>;

class matte_material_t : public material_t
{
public:
    matte_material_t(color_t diffuse_color) :
        diffuse_color_{ diffuse_color }
    {
    }

    bsdf_uptr_t scattering(const isect_t& isect) const override
    {
        return std::make_unique<lambertion_reflection_t>(frame_t(isect.normal), diffuse_color_);
    }

private:
    color_t diffuse_color_{}; // or named `Kd`, `C_diff`
};

class mirror_material_t : public material_t
{
public:
    mirror_material_t(color_t specular_color) :
        specular_color_{ specular_color }
    {
    }

    bsdf_uptr_t scattering(const isect_t& isect) const override
    {
        return std::make_unique<perfect_specular_reflection_t>(frame_t(isect.normal), specular_color_);
    }

private:
    color_t specular_color_{}; // or named `Ks`, `C_spec`
};

class glass_material_t : public material_t
{
public:
    /*
       reflection_color: specular reflection color
       transmission_color: specular transmission color
    */
    glass_material_t(
    float_t eta, color_t reflection_color = color_t{ 1, 1, 1 }, color_t transmission_color = color_t{ 1, 1, 1 }) :
        eta_{ eta },
        reflection_color_{ reflection_color },
        transmission_color_{ transmission_color }
    {
    }

    bsdf_uptr_t scattering(const isect_t& isect) const override
    {
        return std::make_unique<fresnel_specular_scattering_t>(frame_t(isect.normal), 1, eta_, reflection_color_, transmission_color_);
    }

private:
    float_t eta_{};
    color_t reflection_color_{}; // or named `Kr`
    color_t transmission_color_{}; // or named `Kt`
};

class plastic_material_t : public material_t
{
public:
    /*
       diffuse_color: diffuse reflection color
       color_t specular_color: specular reflection color
    */
    plastic_material_t(color_t diffuse_color, color_t specular_color, float_t shininess) :
        diffuse_color_{ diffuse_color },
        specular_color_{ specular_color },
        exponent_{ shininess } // TODO
    {
        //CHECK_DEBUG((Kd_ + Ks_).small_than({ 1, 1, 1 }));

        float_t diffuse = diffuse_color.luminance();
        float_t specular = specular_color.luminance();
        float_t luminance = diffuse + specular;
        
        diffuse_probility_ = diffuse / luminance;
        specular_probility_ = specular / luminance;
    }

    bsdf_uptr_t scattering(const isect_t& isect) const override
    {
        float_t random = rng_.uniform_float();
        if (random < specular_probility_)
        {
            return std::make_unique<phong_specular_reflection_t>(frame_t(isect.normal), specular_color_ / specular_probility_, exponent_);
        }
        else
        {
            return std::make_unique<lambertion_reflection_t>(frame_t(isect.normal), diffuse_color_ / diffuse_probility_);
        }
    }

private:
    color_t diffuse_color_{};
    color_t specular_color_{};
    float_t exponent_{};

    float_t diffuse_probility_{};
    float_t specular_probility_{};
    mutable rng_t rng_{};
};

// TODO: Normalizing Bling-Phong BRDF

#pragma endregion



#pragma region light

enum class light_flag_t
{
    delta_position = 1,
    delta_direction = 2,

    area = 4,
    infinite = 8
};
KY_ENUM_OPERATORS(light_flag_t)

inline bool is_delta_light(light_flag_t flags)
{
    return enum_have(flags, light_flag_t::delta_position) ||
           enum_have(flags, light_flag_t::delta_direction);
}


/*
// called `LightLeSample` in pbrt-v4, `PositionSample` in mitsuba3
// mainly used for light.sample_Le()
struct position_sample_t
{
    bool is_delta;
    point3_t position; // maybe zero
    normal_t normal; // maybe zero
    ray_t ray;
    float_t pdf_position = 0;
    float_t pdf_direction = 0;
    color_t Le; // for light

    position_sample_t() = default;
    position_sample_t(
    point3_t position, normal_t normal, const ray_t& ray,
    float_t pdf_position, float_t pdf_direction,
    color_t emission) :
        position{ position },
        normal{ normal },
        ray(ray),
        pdf_position(pdf_position),
        pdf_direction(pdf_direction),
        Le(emission)
    {
    }

    float_t abs_cos_theta(vec3_t w) const { return is_delta ? 1 : abs_dot(w, normal); }
};
*/

/*
   called `LightLiSample` in pbrt-v4, `DirectionSample` in mitsuba3
   mainly used for light.sample_Li()
*/
struct light_sample_t // : public position_sample_t
{
    point3_t position{};
    vec3_t wi{}; // isect -> light, alone ray's direction
    float_t pdf{}; // pdf of direction(convert from pdf of position)
    color_t Li{}; // for light

    light_sample_t() = default;
    light_sample_t(const isect_t& isect, vec3_t wi, float_t pdf, color_t Li) :
        position{ isect.position },
        wi{ wi },
        pdf{ pdf },
        Li{ Li }
    {
    }
};

class scene_t;

class light_t
{
public:
    virtual ~light_t() = default;
    light_t(/*light_flag_t flags,*/ point3_t world_position, int samples_num = 1):
        world_position_{ world_position },
        samples_num_{ samples_num }
    {
    }

public:
    // whether the light is delta distribution(point, directional), or not(area, environment)
    virtual bool is_delta() const = 0;
    // whether the light is finite extent (point, area) or not(directional, environment)
    virtual bool is_finite() const = 0;

    // only called from `scene_t`
    virtual void preprocess(const scene_t& scene);

    virtual color_t power() const = 0;

    // only work for environment light
    virtual color_t Le(const ray_t& r) const { return color_t{}; }

public:
    // Li means : camera <-wo- isect -wi-> light

    // sample_light/samle_direction
    virtual light_sample_t sample_Li(const isect_t& isect, float2_t random) const = 0;

    virtual float_t pdf_Li(const isect_t& isect, vec3_t world_wi) const = 0;

protected:
    point3_t world_position_;
    int samples_num_;
};

using light_sptr_t = std::shared_ptr<light_t>;
using light_list_t = std::vector<light_sptr_t>;

class point_light_t : public light_t
{
public:
    point_light_t(point3_t world_position, int samples_num, color_t intensity) :
        light_t(world_position, samples_num),
        intensity_{ intensity }
    {
    }

    bool is_delta() const override { return true; }
    bool is_finite() const override { return true; }

    color_t power() const override { return 4 * k_pi * intensity_; }

public:
    light_sample_t sample_Li(const isect_t& isect, float2_t random) const override
    {
        light_sample_t sample;
        sample.position = world_position_;
        sample.wi = normalize(world_position_ - isect.position);
        sample.pdf = 1.f;
        /*
          if a sphere hold the point light, then:
              $$ \Phi= A E = \frac{A I} {l^{2}} $$, where $$ A = 4 \pi r^{2} $$

          or from the definition:
              $$ \mathrm{d} \Phi= \mathrm{d} A E = \frac{\mathrm{d} A I} {l^{2}} $$

          finally we have:
             $$ E = \frac{I}{l^{2}}$$


          as for `Lo = f * Li * cos_theta / pdf`,

          for area light:
              Lo = f * Li * cos_theta / pdf = f * (Li * cos_theta * projected_solid_angle), see `shape::pdf_direction(...)`

          for point light:
              Lo = f * Li * cos_theta / pdf = f * (E * cos_theta / 1), it's no problem
        */
        sample.Li = intensity_ / distance_squared(world_position_, isect.position);

        return sample;
    }

    float_t pdf_Li(
        const isect_t& isect, vec3_t world_wi) const override
    {
        return 0;
    }

private:
    color_t intensity_{};
};

// use a disk to simulate
class direction_light_t : public light_t
{
public:
    direction_light_t(point3_t world_position, int samples_num, color_t irradiance, const vec3_t world_direction) :
        light_t(world_position, samples_num),
        irradiance_{ irradiance },
        world_direction_{ normalize(world_direction) },
        frame_{ world_direction }
    {
    }

public:
    bool is_delta() const override { return true; }
    bool is_finite() const override { return false; }

    void preprocess(const scene_t& scene) override;

    color_t power() const override
    {
        return power_;
    }

public:
    light_sample_t sample_Li(const isect_t& isect, float2_t random) const override
    {
        light_sample_t sample;
        sample.wi = -world_direction_;
        sample.position = isect.position + sample.wi * 2 * world_radius_;
        sample.pdf = 1;
        // both delta light, same as point_light_t::sample_Li(...)
        sample.Li = irradiance_;

        return sample;
    }

    float_t pdf_Li(
        const isect_t& isect, vec3_t world_wi) const override
    {
        return 0;
    }

private:
    color_t irradiance_{};

    point3_t world_center_{};
    float_t world_radius_{};
    float_t area_{};
    color_t power_{};

    unit_vec3_t world_direction_{};
    frame_t frame_{};
};

class area_light_t : public light_t
{
public:
    area_light_t(point3_t world_position, int samples_num, color_t radiance, const shape_t* shape):
        light_t(world_position, samples_num),
        radiance_{ radiance },
        shape_{ shape },
        power_{ radiance_ * shape->area() * k_pi }
    {
    }

public:
    bool is_delta() const override { return false; }
    bool is_finite() const override { return true; }

    color_t power() const override
    {
        return power_;
    }

public:
    /*
       isect
       ----
         ^    ^
          \   |
        wo \  | normal
            \ |
             \|
           -------
         light_isect
    */
    // TODO
    color_t Le(const isect_t& light_isect, vec3_t wo) const
    {
        return (dot(light_isect.normal, wo) > 0) ? radiance_ : color_t();
    }

public:
    // sample direction by sample potision on shape
    light_sample_t sample_Li(const isect_t& isect, float2_t random) const override
    {
        light_sample_t sample;
        isect_t light_isect = shape_->sample_direction(isect, random, &sample.pdf);
        sample.position = light_isect.position;

        if (sample.pdf == 0 || (light_isect.position - isect.position).magnitude_squared() == 0)
        {
            sample.Li = color_t{};
        }
        else
        {
            sample.wi = normalize(light_isect.position - isect.position);
            sample.Li = Le(light_isect, -sample.wi);
        }

        return sample;
    }

    float_t pdf_Li(
        const isect_t& isect, vec3_t world_wi) const override
    {
        return shape_->pdf_direction(isect, world_wi);
    }

private:
    color_t radiance_{};
    color_t power_{};
    const shape_t* shape_{};
};

// constant_environment_light_t
// use a sphere hold all the scene to simulate
class environment_light_t : public light_t
{
public:
    environment_light_t(point3_t world_position, int samples_num, color_t radiance) :
        light_t(world_position, samples_num),
        radiance_{ radiance }
    {
    }

public:
    bool is_delta() const override { return false; }
    bool is_finite() const override { return false; }

    void preprocess(const scene_t& scene) override;

    color_t power() const override
    {
        return power_;
    }

public:
    color_t Le(const ray_t& ray) const
    {
        return radiance_;
    }

public:
    light_sample_t sample_Li(const isect_t& isect, float2_t random) const override
    {
        light_sample_t sample;
        sample.wi = uniform_sphere_sample(random);
        sample.position = isect.position + sample.wi * 2 * world_radius_;

        float_t theta = spherical_theta(sample.wi);
        float_t sin_theta = std::sin(theta);
        sample.pdf = 1 / (2 * k_pi * k_pi * sin_theta);
        if (sin_theta == 0)
            sample.pdf = 0;

        sample.Li = radiance_;

        return sample;
    }

    float_t pdf_Li(
        const isect_t& isect, vec3_t world_wi) const override
    {
        float_t theta = spherical_theta(world_wi);
        float_t sin_theta = std::sin(theta);

        if (sin_theta == 0)
            return 0;

        return 1 / (2 * k_pi * k_pi * sin_theta);
    }

private:
    color_t radiance_{};

    point3_t world_center_{};
    float_t world_radius_{};
    float_t area_{};
    color_t power_{};
};

#pragma endregion



#pragma region surface(primitive)

struct surface_t
{
    const shape_t* shape{};
    const material_t* material{};
    const area_light_t* area_light{};

    bool intersect(const ray_t& ray, isect_t* isect) const
    {
        bool hit = shape->intersect(ray, isect);
        if (hit)
        {
            isect->surface_ = this;
            isect->bsdf_ = material->scattering(*isect);
            isect->emission_ = area_light ? area_light->Le(*isect, isect->wo) : color_t{};
        }

        return hit;
    }
};

using surface_list_t = std::vector<surface_t>;

#pragma endregion

#pragma region accelerator

enum class accel_enum_t
{
    trivial
    // bvh
};

// TODO
class accel_t
{
public:
    virtual ~accel_t() = default;
    accel_t(surface_list_t surface_list) :
        surface_list_{ surface_list }
    {
    }

private:
    surface_list_t surface_list_;
};

#pragma endregion

#pragma region scene

enum class cornell_box_enum_t
{
    // point_light_diffuse_ball,
    // area_light_specular_ball,

    none,

    light_area = 1,
    light_direction = 2,
    light_point = 4,
    light_environment = 8,

    large_mirror_sphere = 16,
    large_glass_sphere = 32,
    small_mirror_sphere = 64,
    small_glass_sphere = 128,

    glossy_floor = 256, // TODO

    both_small_spheres = small_mirror_sphere | small_glass_sphere,
    both_large_spheres = large_mirror_sphere | large_glass_sphere,

    default_scene = both_small_spheres | light_area,
};
KY_ENUM_OPERATORS(cornell_box_enum_t)

class scene_t : public nocopyable_t
{
public:
    scene_t() = default;
    scene_t(
    const_camera_sptr_t camera,
    shape_list_t shape_list, material_list_t material_list, light_list_t light_list, 
    surface_list_t surface_list, environment_light_t* env_light = nullptr) :
        camera_{ camera },
        shape_list_{ shape_list },
        material_list_{ material_list },
        light_list_{ light_list },
        environment_light_{ env_light },
        surface_list_{ surface_list },
        accel_{ surface_list_ }
    {
        for (light_sptr_t& light : light_list_)
        {
            light->preprocess(*this);
        }
        
        // TODO: environment light
    }

public:
    bool intersect(const ray_t& ray, isect_t* isect) const
    {
        bool is_hit = false;

        int surface_num = surface_list_.size();
        for (int i = 0; i < surface_num; ++i)
        {
            if (surface_list_[i].intersect(ray, isect))
                is_hit = true;
        }

        return is_hit;
    }


    bool occluded(
        point3_t position,
        normal_t normal,
        vec3_t direction,
        float_t distance) const
    {
        ray_t ray{ offset_ray_origin(position, normal, direction), direction, distance - 2e-3f };
        isect_t unused;
        return intersect(ray, &unused);
    }
    bool occluded(const isect_t& isect1, point3_t isect2) const
    {
        return occluded(isect1.position, isect1.normal, 
            normalize(isect2 - isect1.position), distance(isect1.position, isect2));
    }
    bool occluded(const isect_t& isect1, const isect_t& isect2) const
    {
        return occluded(isect1.position, isect1.normal,
            normalize(isect2.position - isect1.position), distance(isect1.position, isect2.position));
    }


    bounds3_t world_bound() const
    {
        bounds3_t bounds3;
        
        for (const surface_t& surface : surface_list_)
        {
            bounds3 = bounds3.join(surface.shape->world_bound());
        }

        return bounds3;
    }

public:
    const camera_t* get_camera() const { return camera_.get(); }

    int light_count() const { return light_list_.size(); }
    const light_list_t& light_list() const
    {
        return light_list_;
    }

    const environment_light_t* environment_light() const { return environment_light_; }
    color_t environment_lighting(ray_t ray) const
    {
        if (environment_light_ != nullptr)
            return environment_light_->Le(ray);

        return color_t{};
    }

public:
    static scene_t create_cornell_box_scene(cornell_box_enum_t scene_enum, point2_t film_resolution)
    {
        // https://github.com/SmallVCM/SmallVCM/blob/master/src/scene.hxx#L132

        /*
           world coord:

           z(0, 0, 1)
                |
                |
                |
                |
                |_ _ _ _ _ _ _ x(1, 0, 0)
               /
              /
             /
            /
            y(0, 1, 0)
        */

        const_camera_sptr_t camera = std::make_shared<camera_t>(
            point3_t{ -0.0439815f, 4.12529f,  0.222539f },
            vec3_t{ 0.00688625f, -0.998505f, -0.0542161f },
            vec3_t{ 3.73896e-4f, -0.0542148f, 0.998529f },
            80, film_resolution);

        using enum cornell_box_enum_t;

        if (enum_have(scene_enum, large_mirror_sphere) && enum_have(scene_enum, large_glass_sphere))
        {
            LOG_ERROR("cannot set both large balls\n");
        }

        material_sptr_t black = std::make_shared<matte_material_t>(color_t());
        material_sptr_t white = std::make_shared<matte_material_t>(color_t(.8, .8, .8));
        material_sptr_t red   = std::make_shared<matte_material_t>(color_t(0.803922f, 0.152941f, 0.152941f));
        material_sptr_t green = std::make_shared<matte_material_t>(color_t(0.156863f, 0.803922f, 0.172549f));
        material_sptr_t blue  = std::make_shared<matte_material_t>(color_t(0.156863f, 0.172549f, 0.803922f));

        material_sptr_t glossy = std::make_shared<plastic_material_t>(color_t(.1, .1, .1), color_t(.7, .7, .7), 90.);
        material_sptr_t mirror_mat = std::make_shared<mirror_material_t>(color_t(1, 1, 1));
        material_sptr_t glass_mat = std::make_shared<glass_material_t>(1.6);
        material_list_t material_list{ black, white, red, green, blue, glossy, mirror_mat, glass_mat };

        #pragma region shape

        /*
           cornell box
                                    z   /
              3-------2             |  /
             /|      /|             | /
            7-------6 |             |/
            | |     | |     - - - - o - - - x
            | 0-----|-1            /|
            |/      |/            / |
            4-------5            /  |
                                y   |
                              camera
        */
        vec3_t cb[8] = 
        {
            vec3_t(-1.27029f, -1.30455f, -1.28002f), // 0
            vec3_t( 1.28975f, -1.30455f, -1.28002f), // 1
            vec3_t( 1.28975f, -1.30455f,  1.28002f), // 2
            vec3_t(-1.27029f, -1.30455f,  1.28002f), // 3
            vec3_t(-1.27029f,  1.25549f, -1.28002f), // 4
            vec3_t( 1.28975f,  1.25549f, -1.28002f), // 5
            vec3_t( 1.28975f,  1.25549f,  1.28002f), // 6
            vec3_t(-1.27029f,  1.25549f,  1.28002f)  // 7
        };
        shape_sptr_t left   = std::make_shared<rectangle_t>(cb[3], cb[0], cb[4], cb[7]);
        shape_sptr_t right  = std::make_shared<rectangle_t>(cb[1], cb[2], cb[6], cb[5]);
        shape_sptr_t back   = std::make_shared<rectangle_t>(cb[0], cb[3], cb[2], cb[1]);
        shape_sptr_t bottom = std::make_shared<rectangle_t>(cb[0], cb[1], cb[5], cb[4]);
        shape_sptr_t top    = std::make_shared<rectangle_t>(cb[2], cb[3], cb[7], cb[6]);


        // large ball
        float_t large_radius = 0.8f;
        vec3_t large_center = (cb[0] + cb[4] + cb[5] + cb[1]) * (1.f / 4.f) + vec3_t(0, 0, large_radius);

        // small ball
        float_t small_radius = 0.5f;
        vec3_t left_wall_center  = (cb[0] + cb[4]) * (1.f / 2.f) + vec3_t(0, 0, small_radius);
        vec3_t right_wall_center = (cb[1] + cb[5]) * (1.f / 2.f) + vec3_t(0, 0, small_radius);

        float_t length_x = right_wall_center.x - left_wall_center.x;
        vec3_t left_center  = left_wall_center  + vec3_t(2.f * length_x / 7.f, 0, 0);
        vec3_t right_center = right_wall_center - vec3_t(2.f * length_x / 7.f, 0, 0);

        shape_sptr_t large_ball  = std::make_shared<sphere_t>(large_center, large_radius);
        shape_sptr_t left_ball   = std::make_shared<sphere_t>(left_center, small_radius);
        shape_sptr_t right_ball  = std::make_shared<sphere_t>(right_center, small_radius);


        // small light box at the ceiling
        vec3_t lb[8] = 
        {
            vec3_t(-0.25f, -0.25f, 1.26002f),
            vec3_t( 0.25f, -0.25f, 1.26002f),
            vec3_t( 0.25f, -0.25f, 1.28002f),
            vec3_t(-0.25f, -0.25f, 1.28002f),
            vec3_t(-0.25f,  0.25f, 1.26002f),
            vec3_t( 0.25f,  0.25f, 1.26002f),
            vec3_t( 0.25f,  0.25f, 1.28002f),
            vec3_t(-0.25f,  0.25f, 1.28002f)
        };
        shape_sptr_t left2   = std::make_shared<rectangle_t>(lb[3], lb[7], lb[4], lb[0]);
        shape_sptr_t right2  = std::make_shared<rectangle_t>(lb[1], lb[5], lb[6], lb[2]);
        shape_sptr_t front2  = std::make_shared<rectangle_t>(lb[4], lb[7], lb[6], lb[5]);
        shape_sptr_t back2   = std::make_shared<rectangle_t>(lb[0], lb[1], lb[2], lb[3]);
        shape_sptr_t bottom2 = std::make_shared<rectangle_t>(lb[0], lb[4], lb[5], lb[1]);

        shape_list_t shape_list
        { 
            left, right, back, bottom, top, 
            large_ball, left_ball, right_ball,
            left2, right2, front2, back2, bottom2, 
        };

        #pragma endregion


        #pragma region light

        light_list_t light_list{};
        if (enum_have(scene_enum, light_area))
        {
            light_list.push_back(
                std::make_shared<area_light_t>(point3_t(), 1, color_t(25, 25, 25), bottom2.get()));
        }

        if (enum_have(scene_enum, light_direction))
        {
            light_list.push_back(
                std::make_shared<direction_light_t>(point3_t(), 1, color_t(10, 4, 0), vec3_t(-1, -1.5, -1)));
        }

        if (enum_have(scene_enum, light_point))
        {
            float_t I = 70 * k_inv_4pi;
            light_list.push_back(
                std::make_shared<point_light_t>(point3_t(0.0, 0.5, 1.0), 1, color_t(I, I, I)));
        }

        environment_light_t* environment_light{};
        if (enum_have(scene_enum, light_environment))
        {
            color_t L = color_t(135. / 255, 206. / 255, 250. / 255);
            auto light = std::make_shared<environment_light_t>(point3_t(), 1, L);
            light_list.push_back(light);

            environment_light = light.get();
        }

        #pragma endregion


        #pragma region surface

        surface_list_t surface_list
        {
            {   left.get(),   green.get(), nullptr},
            {  right.get(),     red.get(), nullptr },
            {    top.get(),   white.get(), nullptr },
            { bottom.get(),  glossy.get(), nullptr },
            {   back.get(),    blue.get(), nullptr },
        };

        if (enum_have(scene_enum, large_mirror_sphere))
            surface_list.push_back({ large_ball.get(), mirror_mat.get(), nullptr });
        else if (enum_have(scene_enum, large_glass_sphere))
            surface_list.push_back({ large_ball.get(), glass_mat.get(), nullptr });

        if (enum_have(scene_enum, small_mirror_sphere))
            surface_list.push_back({ left_ball.get(), mirror_mat.get(), nullptr });
        if (enum_have(scene_enum, small_glass_sphere))
            surface_list.push_back({ right_ball.get(), glass_mat.get(), nullptr });

        if (enum_have(scene_enum, light_area))
        {
            surface_list.push_back({   left2.get(), white.get(), nullptr });
            surface_list.push_back({  right2.get(), white.get(), nullptr });
            surface_list.push_back({  front2.get(), white.get(), nullptr });
            surface_list.push_back({   back2.get(), white.get(), nullptr });
            surface_list.push_back({ bottom2.get(), black.get(), (area_light_t*)light_list[0].get() });
        }

        #pragma endregion


        return scene_t{ camera, shape_list, material_list, light_list, surface_list, environment_light };
    }

    static scene_t create_mis_scene(point2_t film_resolution)
    {
        // https://github.com/mitsuba-renderer/mitsuba-data/blob/master/docs/scenes/include/veach_mis.xml
        // TODO: specify film, sampler of this scene

        /*
           world coord:

           y(0, 1, 0)
                |
                |
                |
        cam->   |
                |_ _ _ _ _ _ _ z(0, 0, 1)
               /
              /
             /
            /
            x(1, 0, 0)
        */

        const_camera_sptr_t camera = std::make_unique<camera_t>(
            point3_t{ 0, 2, -15 },
            vec3_t{ 0, -4, 12.5 }, vec3_t{ 0, 1, 0 },
            50, film_resolution);

        material_sptr_t black = std::make_shared<matte_material_t>(color_t());
        material_sptr_t gray = std::make_shared<matte_material_t>(color_t(.4, .4, .4));
        material_sptr_t silver = std::make_shared<plastic_material_t>(color_t(0.07, 0.09, 0.13), color_t(1, 1, 1), 5000);
        material_list_t material_list{ black, gray, silver };

#pragma region shape

        shape_sptr_t bottom = std::make_shared<rectangle_t>(
            point3_t(-10, -4.14615, 10), point3_t(-10, -4.14615, -10), point3_t(10, -4.14615, -10), point3_t(10, -4.14615, 10), true);
        shape_sptr_t back = std::make_shared<rectangle_t>(
            point3_t(-10, -10, 2), point3_t(-10, 10, 2), point3_t(10, 10, 2), point3_t(10, -10, 2), true);

        shape_sptr_t plank0 = std::make_shared<rectangle_t>(
            point3_t(4, -2.70651, -0.25609), point3_t(4, -2.08375, 0.526323), point3_t(-4, -2.08375, 0.526323), point3_t(-4, -2.70651, -0.25609), true);
        shape_sptr_t plank1 = std::make_shared<rectangle_t>(
            point3_t(4, -3.28825, -1.36972), point3_t(4, -2.83856, -0.476536), point3_t(-4, -2.83856, -0.476536), point3_t(-4, -3.28825, -1.36972), true);
        shape_sptr_t plank2 = std::make_shared<rectangle_t>(
            point3_t(4, -3.73096, -2.70046), point3_t(4, -3.43378, -1.74564), point3_t(-4, -3.43378, -1.74564), point3_t(-4, -3.73096, -2.70046), true);
        shape_sptr_t plank3 = std::make_shared<rectangle_t>(
            point3_t(4, -3.99615, -4.0667), point3_t(4, -3.82069, -3.08221), point3_t(-4, -3.82069, -3.08221), point3_t(-4, -3.99615, -4.0667), true);

        shape_sptr_t ball0 = std::make_shared<sphere_t>(point3_t(10, 10, -4), 0.5);
        shape_sptr_t ball1 = std::make_shared<sphere_t>(point3_t(-3.75, 0, 0), 0.03333);
        shape_sptr_t ball2 = std::make_shared<sphere_t>(point3_t(-1.25, 0, 0), 0.1);
        shape_sptr_t ball3 = std::make_shared<sphere_t>(point3_t(1.25, 0, 0), 0.3);
        shape_sptr_t ball4 = std::make_shared<sphere_t>(point3_t(3.75, 0, 0), 0.9);

        shape_list_t shape_list
        {
            bottom, back,
            plank0, plank1, plank2, plank3,
            ball0, ball1, ball2, ball3, ball4
        };

#pragma endregion

        // light0 used as envirment light
        auto light0 = std::make_shared<area_light_t>(point3_t(), 1, color_t(800, 800, 800), ball0.get());
        auto light1 = std::make_shared<area_light_t>(point3_t(), 1, color_t(901.803, 901.803, 901.803), ball2.get());
        auto light2 = std::make_shared<area_light_t>(point3_t(), 1, color_t(100, 100, 100), ball1.get());
        auto light3 = std::make_shared<area_light_t>(point3_t(), 1, color_t(11.1111, 11.1111, 11.1111), ball3.get());
        auto light4 = std::make_shared<area_light_t>(point3_t(), 1, color_t(1.23457, 1.23457, 1.23457), ball4.get());

        light_list_t light_list
        {
           light0, light1, light2, light3, light4
        };

        /*
        float_t I = 70000 * k_inv_4pi;
        light_list.push_back(
            std::make_shared<point_light_t>(point3_t(10.0, 10, -4), 1, color_t(I, I, I)));
        */

        surface_list_t surface_list
        {
            { bottom.get(), gray.get(), nullptr},
            {   back.get(), gray.get(), nullptr},

            { plank0.get(), silver.get(), nullptr},
            { plank1.get(), silver.get(), nullptr},
            { plank2.get(), silver.get(), nullptr},
            { plank3.get(), silver.get(), nullptr},

            { ball0.get(),  black.get(), light0.get() },
            { ball1.get(),  black.get(), light1.get() },
            { ball2.get(),  black.get(), light2.get() },
            { ball3.get(),  black.get(), light3.get() },
            { ball4.get(),  black.get(), light4.get() },
        };

        // TODO
        return scene_t{ camera, shape_list, material_list, light_list, surface_list };
    }

private:
    const_camera_sptr_t camera_;

    shape_list_t shape_list_;
    material_list_t material_list_;

    light_list_t light_list_;
    environment_light_t* environment_light_;

    // TODO: std::vector<std::function<intersect(ray_t ray), result_t> surfaces_;
    surface_list_t surface_list_;
    accel_t accel_;
};



void light_t::preprocess(const scene_t& scene)
{
}

void direction_light_t::preprocess(const scene_t& scene)
{
    bounds3_t world_bound = scene.world_bound();

    world_bound.bounding_sphere(&world_center_, &world_radius_);

    area_ = k_pi * world_radius_ * world_radius_;
    power_ = irradiance_ * area_;
}

void environment_light_t::preprocess(const scene_t& scene)
{
    bounds3_t world_bound = scene.world_bound();

    world_bound.bounding_sphere(&world_center_, &world_radius_);

    // TODO
    area_ = k_pi * world_radius_ * world_radius_;
    power_ = radiance_ * area_;
}

#pragma endregion



#pragma region integrator

/* 
  Li = Lo = Le + ∫Li
          = Le + ∫(Le + ∫Li)
          = Le + ∫Le  + ∫∫Li
          = Le + ∫Le  + ∫∫(Le + ∫Li)
          = Le + ∫Le  + ∫∫Le  + ∫∫∫(Le + ∫Li)
          = Le + ∫Le  + ∫∫Le  + ∫∫∫Le  + ∫∫∫∫Le + ...
*/

enum class lighting_enum_t
{
    emit = 1, // Le = Le
    direct = 2, // Ld = ∫Le
    indirect = 4, // Li = ∫∫(Le + ∫Li)
    all_lighting = emit | direct | indirect,

    diffuse = 8,
    specular = 16,
    all_scattering = diffuse | specular,

    all = all_lighting | all_scattering
};
KY_ENUM_OPERATORS(lighting_enum_t)

// TODO: rename
// direct_lighting_sample
enum class direct_sample_enum_t
{
    idle,

    sample_single_light = 1,
    sample_all_light = 2,

    bsdf = 4, // direction
    light = 8, // position

    bsdf_mis = 16,
    light_mis = 32,
    both_mis = bsdf_mis | light_mis,

    default_stragtgy = sample_all_light | both_mis
};

enum class integrator_enum_t
{
    // debug
    position,
    normal,
    basecolor,

    // discrete
    delta_bsdf, // + area light
    delta_light, // + diffuse brdf
    direct_lighting_point, 

    // ray casting/direct lighting
    direct_lighting,

    // stochastic ray tracing(without RR???)
    stochastic_raytracing,

    // path tracing

    // Le + T(Le + T(Le + T(...)))
    simple_path_tracing_recursion,
    //simple_path_tracing_iterasion,

    // Le + T * Le + T(T * Le + T(...))
    path_tracing_recursion,
    path_tracing_recursion_defered,
    path_tracing_iteration,
    //path_tracing_split,
};


// TODO: remove
struct light_list_sample_t
{
    light_t* light{};
    float_t pdf_light{};
};


struct path_vertex_t
{
    scene_t* scene{};
    sampler_t* sampler{};
    ray_t ray;
    int depth;
    bool is_prev_specular;
    isect_t isect;
};

/*
  rendering scene by Rendering Equation(Li = Lo = Le + ∫Li)
  solving Rendering Equation(a integral equation) by numerical integration(Monte Carlo Integration)
*/
class integrator_t
{
public:
    ~integrator_t() = default;
    integrator_t()
    {
    }

public:
    // TODO: why can't const?
    void render(/*const*/ scene_t* scene, sampler_t* original_sampler, film_t* film)
    {
        auto camera = scene->get_camera();
        vec2_t resolution = film->get_resolution();
        int width = (int)resolution.x;
        int height = (int)resolution.y;

    #ifdef KY_RELEASE
        #pragma omp parallel for schedule(dynamic, 1) // OpenMP
    #endif // !KY_RELEASE
        for (int y = 0; y < height; y += 1)
        {
            auto sampler = original_sampler->clone(); // multi thread
            LOG("rendering... {} spp, {:.2f}%\r", sampler->ge_samples_per_pixel(), 100. * y / (height - 1));

            for (int x = 0; x < width; x += 1)
            {
                color_t L{};
                sampler->start_pixel();
                //film_->set_color(x, y, color_t(0, 0, 0));

                do
                {
                    auto camera_sample = sampler->get_camera_sample({ (float_t)x, (float_t)y });
                    ray_t ray = camera->generate_ray(camera_sample);

                    color_t dL = Li(ray, scene, sampler.get()) * (1. / sampler->ge_samples_per_pixel());
                    //LOG_DEBUG("dL: {}", dL.to_string());
                    CHECK_DEBUG(dL.is_valid(), "{}", dL.to_string());

                    L = L + dL;
                }
                while (sampler->next_sample());

                // TODO
                film->add_color(x, y, clamp01(L));
            }
        }
    }

    // TODO rename: render_phase()
    // ~~ATTENTION: debug_area() minus the horizontal and vertical coordinates of one pixel automatically~~
    void debug_area(/*const*/ scene_t* scene, sampler_t* original_sampler, film_t* film, point2_t begin, point2_t end)
    {
        //begin -= vec2_t{ 1, 1 };
        //end   -= vec2_t{ 1, 1 };

        // fill area with red color
        for (int y = begin.y - 1; y <= end.y; y += 1)
        {
            for (int x = begin.x - 1; x <= end.x; x += 1)
            {
                film->add_color(x, y, color_t{ 1.f });
            }
        }

        const camera_t* camera = scene->get_camera();
        sampler_t* sampler = original_sampler;

        for (int y = begin.y; y < end.y; y += 1)
        {
            LOG("debug... {} spp, {:.2f}%\r", sampler->ge_samples_per_pixel(), 100.f * (y - begin.y) / (end.y - begin.y - 1));

            for (int x = begin.x; x < end.x; x += 1)
            {
                film->clear_color(x, y);
                color_t L{};
                sampler->start_pixel();

                do
                {
                    auto camera_sample = sampler->get_camera_sample({ (float_t)x, (float_t)y });
                    ray_t ray = camera->generate_ray(camera_sample);

                    LOG_VAST("x: {}, y: {}\n", x, y);

                    color_t dL = Li(ray, scene, sampler) * (1. / sampler->ge_samples_per_pixel());
                    //LOG("dL:{}\n", dL.to_string());
                    L = L + dL;
                }
                while (sampler->next_sample());

                //LOG("L:{}\n", L.to_string());
                film->add_color(x, y, clamp01(L));
            }
        } 
    }

    void debug_area(scene_t* scene, sampler_t* original_sampler, film_t* film, point2_t begin, float_t width, float_t height)
    {
        debug_area(scene, original_sampler, film, begin, begin + vec2_t{width, height});
    }

    void debug_pixel(scene_t* scene, sampler_t* original_sampler, film_t* film, point2_t pixel_position)
    {
        debug_area(scene, original_sampler, film, pixel_position, pixel_position + vec2_t(1, 1));
    }

public:
    // estimate input radiance
    // TODO: virtual color_t Li(ray_t ray, const scene_t& scene, sampler_t& sampler) = 0;
    virtual color_t Li(ray_t ray, scene_t* scene, sampler_t* sampler) = 0;

protected:

#pragma region sampling_light

    light_list_sample_t pick_single_light(
        const isect_t& isect, scene_t* scene, sampler_t& sampler)
    {
        int light_count = int(scene->light_count());
        if (light_count == 0)
            return {};

        // TODO: power based, spatial based
        int light_index = std::min((int)(sampler.get_float() * light_count), light_count - 1);
        light_t* light = scene->light_list()[light_index].get();
        float_t pdf_light = float_t(1) / light_count;

        return { light, pdf_light };
    }

    static color_t sample_single_light(
        const isect_t& isect, scene_t* scene, sampler_t& sampler, bool skip_specular)
    {
        // Randomly choose a single light to sample
        int light_count = int(scene->light_count());
        if (light_count == 0)
            return color_t();

        // TODO: power based, spatial based
        int light_index = std::min((int)(sampler.get_float() * light_count), light_count - 1);
        float_t pdf_light = float_t(1) / light_count;

        light_t* light = scene->light_list()[light_index].get();
        point2_t uLight = sampler.get_float2();
        point2_t uScattering = sampler.get_float2();

        // default skip perfectly specular BSDF due to its delta distribution
        return estimate_direct_lighting_both_mis(isect, *light, uLight, uScattering,
            scene, sampler, skip_specular) / pdf_light; // for all light
    }

    static color_t sample_all_light(
        const isect_t& isect, scene_t* scene, sampler_t& sampler, bool skip_specular, direct_sample_enum_t sample_enum)
    {
        color_t Ld;

        std::function<decltype(estimate_direct_lighting_both_mis)> estimate_direct_lighting;
        switch (sample_enum)
        {
        case direct_sample_enum_t::idle:
            estimate_direct_lighting = estimate_direct_lighting_idle;
            break;
        case direct_sample_enum_t::bsdf:
            estimate_direct_lighting = estimate_direct_lighting_by_direction;
            break;
        case direct_sample_enum_t::light:
            estimate_direct_lighting = estimate_direct_lighting_by_position;
            break;
        case direct_sample_enum_t::bsdf_mis:
            estimate_direct_lighting = estimate_direct_lighting_by_direction_mis;
            break;
        case direct_sample_enum_t::light_mis:
            estimate_direct_lighting = estimate_direct_lighting_by_position_mis;
            break;
        case direct_sample_enum_t::both_mis:
            estimate_direct_lighting = estimate_direct_lighting_both_mis;
            break;
        default:
            break;
        }

        for (const light_sptr_t& light : scene->light_list())
        {
            Ld += estimate_direct_lighting(
                isect, *light, sampler.get_float2(), sampler.get_float2(),
                scene, sampler, skip_specular);
        }

        return Ld;
    }

#pragma endregion

protected:

#pragma region estimate_direct_lighting

    static color_t estimate_direct_lighting_idle(
        const isect_t& isect, const light_t& light,
        float2_t random_light, float2_t random_bsdf,
        scene_t* scene, sampler_t& sampler, bool skip_specular)
    {
        return {};
    }

    // estimate single light source's direct contirbution by sampling bsdf's direction
    static color_t estimate_direct_lighting_by_direction(
        const isect_t& isect, const light_t& light,
        float2_t random_light, float2_t random_bsdf,
        scene_t* scene, sampler_t& sampler, bool skip_specular)
    {
        if (light.is_delta())
            return {};

        if (skip_specular && isect.bsdf()->is_delta())
            return {};

        bsdf_sample_t bs = isect.bsdf()->sample(isect.wo, sampler.get_float2());
        if (bs.f.is_black() || bs.pdf == 0)
            return {};

        ray_t ray = isect.spawn_ray(bs.wi);
        isect_t light_isect;
        bool is_hit_light = scene->intersect(ray, &light_isect);

        color_t Li{};
        if (is_hit_light)
        {
            if (light_isect.surface()->area_light == &light)
                Li = light_isect.Le();
        }
        else
        {
            // only work for environment light
            Li = light.Le(ray);
        }

        if (Li.is_black())
            return {};

        float_t cos_theta = abs_dot(bs.wi, isect.normal);
        color_t Ld = bs.f * Li * cos_theta / bs.pdf;

        //LOG_DEBUG("{}, {}\n", bs.wi.to_string(), isect.normal.to_string());
        //LOG_DEBUG("{}, {}, {}, {}\n", bs.f.to_string(), Li.to_string(), cos_theta, bs.pdf);

        return Ld;
    }

    // estimate single light source's direct contirbution by sampling light's position
    static color_t estimate_direct_lighting_by_position(
        const isect_t& isect, const light_t& light,
        float2_t random_light, float2_t random_bsdf,
        scene_t* scene, sampler_t& sampler, bool skip_specular)
    {
        if (skip_specular && isect.bsdf()->is_delta())
            return {};

        light_sample_t ls = light.sample_Li(isect, random_light);
        if (ls.Li.is_black() || ls.pdf == 0)
            return {};

        // below commented code only used for reflection, now it's already handled by bsdf.eval(...)
        //if (dot(ls.wi, isect.normal) < 0)
        //    return {};

        if (scene->occluded(isect, ls.position))
            return {};

        color_t f = isect.bsdf()->eval(isect.wo, ls.wi);
        if (f.is_black())
            return {};

        float_t cos_theta = abs_dot(ls.wi, isect.normal);
        color_t Ld = f * ls.Li * cos_theta / ls.pdf;

        //LOG_DEBUG("{}, {}\n", ls.wi.to_string(), isect.normal.to_string());
        LOG_DEBUG("{}, {}, {}, {}, {}\n", Ld.to_string(), f.to_string(), ls.Li.to_string(), cos_theta, ls.pdf);

        return Ld;
    }

#pragma endregion

#pragma region estimate_direct_lighting_with_MIS

    static color_t estimate_direct_lighting_by_direction_mis(
        const isect_t& isect, const light_t& light,
        float2_t random_light, float2_t random_bsdf,
        scene_t* scene, sampler_t& sampler, bool skip_specular)
    {
        bool is_specular = isect.bsdf()->is_delta();
        if (skip_specular && is_specular)
            return {};

        if (light.is_delta())
            return {};

        // sample scattered direction for surface isect_t
        bsdf_sample_t bs = isect.bsdf()->sample(isect.wo, random_bsdf);
        bs.f *= abs_dot(bs.wi, isect.normal);
        if (bs.f.is_black() || bs.pdf <= 0)
            return {};

        // TODO: move to scene_t
        ray_t ray = isect.spawn_ray(bs.wi);
        isect_t light_isect;
        bool is_hit_light = scene->intersect(ray, &light_isect);

        color_t Li{};
        if (is_hit_light)
        {
            if (light_isect.surface()->area_light == &light)
                Li = light_isect.Le();
        }
        else
        {
            // only work for environment light
            Li = light.Le(ray);
        }

        if (Li.is_black())
            return {};

        color_t Ld{};
        if (is_specular)
            Ld = bs.f * Li / bs.pdf; // don't need MIS
        else
        {
            float_t light_pdf = light.pdf_Li(isect, bs.wi);
            if (light_pdf > 0) // visible
            {
                float_t weight = balance_heuristic(1, bs.pdf, 1, light_pdf);

                // sample BSDF with one-sample MIS
                Ld = (bs.f * Li * weight) / (0.5 * bs.pdf);
            }
        }

        return Ld;
    }

    static color_t estimate_direct_lighting_by_position_mis(
        const isect_t& isect, const light_t& light,
        float2_t random_light, float2_t random_bsdf,
        scene_t* scene, sampler_t& sampler, bool skip_specular)
    {
        if (skip_specular && isect.bsdf()->is_delta())
            return {};

        light_sample_t ls = light.sample_Li(isect, random_light);
        if (ls.Li.is_black() || ls.pdf <= 0)
            return {};

        if (scene->occluded(isect, ls.position))
            return {};

        color_t f = isect.bsdf()->eval(isect.wo, ls.wi) * abs_dot(ls.wi, isect.normal);
        if (f.is_black())
            return {};

        color_t Ld{};
        if (light.is_delta())
            Ld = f * ls.Li / ls.pdf; // don't need MIS
        else
        {
            float_t bsdf_pdf = isect.bsdf()->pdf(isect.wo, ls.wi);
            float_t weight = balance_heuristic(1, ls.pdf, 1, bsdf_pdf);

            // sample light source with one-sample MIS
            Ld = (f * ls.Li * weight) / (0.5 * ls.pdf);
        }

        return Ld;
    }

    static color_t estimate_direct_lighting_both_mis(
        const isect_t& isect, const light_t& light,
        float2_t random_light, float2_t random_bsdf,
        scene_t* scene, sampler_t& sampler, bool skip_specular)
    {
        color_t Lb = estimate_direct_lighting_by_direction_mis(isect, light, random_light, random_bsdf, scene, sampler, skip_specular);
        color_t Ll = estimate_direct_lighting_by_position_mis(isect, light, random_light, random_bsdf, scene, sampler, skip_specular);
        color_t Ld = 0.5 * Lb + 0.5 * Ll;

        //LOG_DEBUG("{}, {}, {}\n", Ld.to_string(), Lb.to_string(), Ll.to_string());

        return Ld;
    }

#pragma endregion
};


class debug_integrator_t : public integrator_t
{
private:
    integrator_enum_t integrator_enum_;

public:
    debug_integrator_t(integrator_enum_t integrator_enum_) :
        integrator_enum_{ integrator_enum_ }
    {
    }

    color_t Li(ray_t ray, scene_t* scene, sampler_t* sampler) override
    {
        isect_t isect;
        if (scene->intersect(ray, &isect))
        {
            switch (integrator_enum_)
            {
            case integrator_enum_t::position:
                return isect.position.normalize();
            case integrator_enum_t::normal:
                return isect.normal.normalize();
            case integrator_enum_t::basecolor:
                return isect.bsdf()->eval(isect.wo, isect.normal);
            }
        }

        return color_t();
    }
};

class direct_lighting_t : public integrator_t
{
private:
    direct_sample_enum_t direct_sample_enum_;

public:
    direct_lighting_t(direct_sample_enum_t direct_sample_enum) :
        direct_sample_enum_{ direct_sample_enum }
    {
    }

    color_t Li(ray_t ray, scene_t* scene, sampler_t* sampler) override
    {
        isect_t isect;
        bool hit = scene->intersect(ray, &isect);

        if (!hit)
            return scene->environment_lighting(ray);

        // emission lighting
        color_t Lo = isect.Le();

        // TODO
        if (!isect.bsdf()->is_delta())
        {
            // direct lighting
            Lo += sample_all_light(isect, scene, *sampler, true, direct_sample_enum_);
        }

        return Lo;
    }
};

/*
class stochastic_raytracing_t : public integrator_t
{
public:
    stochastic_raytracing_t(int max_path_depth) :
        max_path_depth_{ max_path_depth }
    {
    }

protected:
    int max_path_depth_;
};
*/


class path_integrator_t : public integrator_t
{
public:
    path_integrator_t(int max_path_depth, direct_sample_enum_t direct_sample_enum):
        max_path_depth_{ max_path_depth },
        direct_sample_enum_{ direct_sample_enum }
    {
    }

protected:
    int max_path_depth_;
    direct_sample_enum_t direct_sample_enum_;
};


/*
   simple path tracing implementation, only sample BRDF
   Li = Le + T*(Le + T*(le + ...))
*/
class simple_path_tracing_recursion_t : public path_integrator_t
{
public:
    using path_integrator_t::path_integrator_t;

    color_t Li(ray_t ray, scene_t* scene, sampler_t* sampler) override
    {
        return Li(ray, scene, sampler, 0);
    }

    color_t Li(ray_t ray, scene_t* scene, sampler_t* sampler, int depth)
    {
        isect_t isect;
        if (!scene->intersect(ray, &isect))
        {
            return scene->environment_lighting(ray);
        }

        if (depth >= max_path_depth_)
            return isect.Le();


        bsdf_sample_t bs = isect.bsdf()->sample(isect.wo, sampler->get_float2());

        if (bs.f.is_black() || bs.pdf == 0.f) // pdf == 0 => NaN
            return isect.Le();

        //russian roulette
        if (++depth > 3)
        {
            float_t bsdf_max_comp = bs.f.max_component_value();
            if (sampler->get_float() < bsdf_max_comp) // continue
                bs.f *= (1 / bsdf_max_comp);
            else
                return isect.Le();
        }

        /*
          color_t beta = f * abs_dot(wi, isect.normal) / pdf;
          return beta * Li(wi_ray, scene, sampler, depth));
        */
        ray_t wi(isect.position, bs.wi);
        color_t Ls = bs.f * Li(wi, scene, sampler, depth) * abs_dot(bs.wi, isect.normal) / bs.pdf;

        // TODO: CHECK_DEBUG
        return isect.Le() + Ls;
    }
};

#pragma region path_tracing_recursion_t

/*
  simple_path_tracing_recursion_t only sample BRDF, it can't sample point/directional light, 
  and too slow to sample small area light(only little samples can hit light)
  if we split ∫Li to ∫(Le + ∫Li), then direct sample light, can solve above problems

  a bit of pity, it makes things complicated:
      Li = Lo = Le + ∫Li
              = Le + ∫(Le + ∫Li)
              = Le + ∫Le  + ∫(∫Li)
              = Le + ∫Le  + ∫(∫(Le + ∫Li) )
              = Le + ∫Le  + ∫(∫Le  + ∫(∫Li) )
              = Le + ∫Le  + ∫(∫Le  + ∫(∫(Le + ∫Li) ))
              = Le + ∫Le  + ∫(∫Le  + ∫(∫Le  + ∫(∫Li) ))
              = Le + ∫Le  + ∫(∫Le  + ∫(∫Le  + ∫(∫Le + ...)))  <-- LOOK THIS

  further, handle specular BRDF make things more complex

  pseudo-code:
    def Li(ray, scene, depth):
        Lo = 0

        cast ray to scene, find isect

        if depth == 0
            Lo += emission_lighting(ray, scene, depth, isect)
        
        if isect && depth < max_depth
            if isect.bsdf().not_delta()
                Lo += direct_lighting(ray, scene, sampler, isect)
            else
                compute specular vertex's reflect/refract direction
                wi_ray = ray(isect.position, new direction)
                cast wi_ray to scene, find next_isect
                Lo += emission_lighting(ray, scene, depth, next_isect)

            Lo += indirect_lighting(ray, scene, sampler, isect, depth)

        return Lo
    end

    def indirect_lighting(ray, scene, sampler, isect, depth)
        compute current vertex's reflect/refract direction
        wi_ray = ray(isect.position, new direction)
        ...
        Lo += Li(ray, scene, depth)
    end
  end

  `path_tracing_recursion_t` are implemention of above

  since there have duplicate code and compute, in actual code implementatioin, 
  only compute specular vertex's reflect/refract direction once, 
  defer it's direct lighting to the next recursion

  `path_tracing_recursion_defered_t` are implemention of above,
  `path_tracing_iteration_t` are iteration version of above
*/

/*
  recursion style path tracing
  Li = Le + T*Le + T*(T*Le + T*(T*Le + ...))
*/

class path_tracing_recursion_t : public path_integrator_t
{
public:
    path_tracing_recursion_t(int max_path_depth, direct_sample_enum_t direct_sample_enum) :
        path_integrator_t(max_path_depth, direct_sample_enum)
    {
    }

    color_t Li(ray_t ray, scene_t* scene, sampler_t* sampler) override
    {
        return Li(scene, sampler, ray, 0 /*, lighting_enum_*/);
    }

private:
    // cast ray to scene, find the nearest isect, return the indident radiance alone ray's direciotn from isect
    // this function will be called recursion from `indirect_lighting(...)`
    color_t Li(scene_t* scene, sampler_t* sampler, ray_t ray, int depth)
    {
        color_t Lo;

        isect_t isect;
        bool hit = scene->intersect(ray, &isect);

        if (depth == 0)
        {
            Lo += emission_lighting(scene, sampler, ray, isect, hit);
        }

        if (hit && depth < max_path_depth_)
        {
            if (!isect.bsdf()->is_delta())
            {
                Lo += direct_lighting(scene, sampler, isect);
            }
            else // specular vertex' direct lighting
            { 
                bsdf_sample_t bs = isect.bsdf()->sample(isect.wo, sampler->get_float2());

                ray_t wi_ray{ isect.position, bs.wi };
                isect_t next_isect;
                bool next_hit = scene->intersect(wi_ray, &next_isect);

                // cos_theta is offset, bs.pdf=1 below, so actual:
                // Lo += isect.bsdf().f(wo, reflect(wo)) * emission_lighting(...);
                Lo += bs.f * emission_lighting(scene, sampler, wi_ray, next_isect, next_hit) * abs_dot(bs.wi, isect.normal) / bs.pdf;
            }

            Lo += indirect_lighting(scene, sampler, isect, depth);
        }

        return Lo;
    }

    color_t emission_lighting(scene_t* scene, sampler_t* sampler, ray_t ray, const isect_t& isect, bool hit)
    {
        color_t Le;

        if (hit)
        {
            Le = isect.Le();
        }
        else
        {
            Le = scene->environment_lighting(ray);
        }

        return Le;
    }

    color_t direct_lighting(scene_t* scene, sampler_t* sampler, const isect_t& isect)
    {
        color_t Ld = sample_all_light(isect, scene, *sampler, true, direct_sample_enum_);

        return Ld;
    }

    color_t indirect_lighting(scene_t* scene, sampler_t* sampler, const isect_t& isect, int depth)
    {
        bsdf_sample_t bs = isect.bsdf()->sample(isect.wo, sampler->get_float2());

        if (bs.f.is_black() || bs.pdf == 0.f)
            return color_t{};

        //russian roulette
        if (++depth > 3)
        {
            float_t bsdf_max_comp = bs.f.max_component_value();

            if (sampler->get_float() < bsdf_max_comp) // continue
                bs.f *= 1 / bsdf_max_comp;
            else
                return color_t{};
        }

        ray_t wi_ray{ offset_ray_origin(isect.position, isect.normal, bs.wi), bs.wi};
        return bs.f * Li(scene, sampler, wi_ray, depth) * abs_dot(bs.wi, isect.normal) / bs.pdf;
    }
};



/*
  based on path_tracing_recursion_t, defer specular vertex's direct lighting to the next recursion
*/
class path_tracing_recursion_defered_t : public path_integrator_t
{
private:
    lighting_enum_t lighting_enum_;

public:
    path_tracing_recursion_defered_t(int max_path_depth, direct_sample_enum_t direct_sample_enum, lighting_enum_t lighting_enum) :
        path_integrator_t(max_path_depth, direct_sample_enum),
        lighting_enum_{ lighting_enum }
    {
    }

    color_t Li(ray_t ray, scene_t* scene, sampler_t* sampler) override
    {
        return Li(scene, sampler, ray, 0, false /*, lighting_enum_*/);
    }

private:
    /*
      prev   n   next
      ----   ^   ----
        \    |    ^
         \   | θ /
       wo \  |  / wi is unknown, sampling from bsdf
           \ | /
            v|/
          -------
           isect

      is_prev_specular: whether previous vertex is specular
    */
    color_t Li(scene_t* scene, sampler_t* sampler, ray_t ray, int depth, bool is_prev_specular)
    {
        color_t Lo;

        isect_t isect;
        bool hit = scene->intersect(ray, &isect);

        // current isect's _Le_ is already integral by previous vertex's direct lighting
        // only accumulate _Le_ for first vertex, or previous vertex is specular(it skips integral, restore below)
        if (depth == 0 || is_prev_specular)
        {
            Lo += emission_lighting(scene, sampler, ray, isect, hit);
        }

        if (hit && depth < max_path_depth_)
        {
            if (!isect.bsdf()->is_delta())
            {
                Lo += direct_lighting(scene, sampler, isect);
            }
            // else, defer specular vertex's direct lighting to the next recursion

            Lo += indirect_lighting(scene, sampler, isect, depth);
        }

        return Lo;
    }
 
    color_t emission_lighting(scene_t* scene, sampler_t* sampler, ray_t ray, const isect_t& isect, bool hit)
    {
        color_t Le;

        if (hit)
        {
            Le = isect.Le();
        }
        else
        {
            Le = scene->environment_lighting(ray);
        }

        return Le;
    }
 
    color_t direct_lighting(scene_t* scene, sampler_t* sampler, const isect_t& isect)
    {
        color_t Ld = sample_all_light(isect, scene, *sampler, true, direct_sample_enum_);

        return Ld;
    }

    // for diffuse/glossy BSDF, compute indirect lighting, so skip _Le_ on Li()
    // for specular BSDF, compute reflect/refract direciotn's lighting, so skip _Ld_ on direct_lighting()
    color_t indirect_lighting(scene_t* scene, sampler_t* sampler, const isect_t& isect, int depth)
    {
        bsdf_sample_t bs = isect.bsdf()->sample(isect.wo, sampler->get_float2());

        if (bs.f.is_black() || bs.pdf == 0.f)
            return color_t{};

        //russian roulette
        if (++depth > 3)
        {
            float_t bsdf_max_comp = bs.f.max_component_value();

            if (sampler->get_float() < bsdf_max_comp) // continue
                bs.f *= 1 / bsdf_max_comp;
            else
                return color_t{};
        }

        ray_t wi_ray{ isect.position, bs.wi };
        return bs.f * Li(scene, sampler, wi_ray, depth, isect.bsdf()->is_delta()) * abs_dot(bs.wi, isect.normal) / bs.pdf;
    }
};

#pragma endregion

/*
  iteration style path tracing
  Li = Le + T*Le + T*(T*Le + T*(T*Le + ...))
     = Le + T*Le + T^2*Le  + ...
*/
class path_tracing_iteration_t : public path_integrator_t
{
public:
    using path_integrator_t::path_integrator_t;

    // sample bsdf/direction on front vertexs, and sample light/position on final vertex
    color_t Li(ray_t ray, scene_t* scene, sampler_t* sampler) override
    {
        // Lo(out), Le(emit), Ld(direct), Li(indirect)
        color_t Lo{};

        color_t beta{ 1, 1, 1 }; // beta holds path throughput weight
        bool is_prev_specular = false; // whether pervious vertex's material has perfect specular property

        for (int bounces = 0; ; ++bounces)
        {
            // find next path vertex and accumulate contribution
            // cast _ray_ to scene and store intersection in _isect_
            isect_t isect;
            bool hit = scene->intersect(ray, &isect);


            // Le: emission

            // possibly add emitted light at intersection
            if (bounces == 0 || is_prev_specular)
            {
                // add emitted light at path vertex or from the environment
                if (hit)
                {
                    Lo += beta * isect.Le();
                }
                else
                {
                    Lo += beta * scene->environment_lighting(ray);
                }
            }


            // terminate path if ray escaped or _maxDepth_ was reached
            if (!hit || bounces >= max_path_depth_)
                break;


            // Ld: direct lighting / NEE(Next Event Estimation)

            // sample illumination from lights to find path contribution.
            // (but skip this for perfectly specular BSDFs.)
            if (!isect.bsdf()->is_delta())
                //&& bounces > 0 && bounces < 2) // for debug
                //&& bounces == 1) // for debug
            {
                color_t Ld = beta * sample_all_light(isect, scene, *sampler, true, direct_sample_enum_);
                Lo += Ld;

                LOG_VAST("isect.position: {}, .normal: {}, .wo: {} -> Ld: {}\n",
                    isect.position.to_string(), isect.normal.to_string(), isect.wo.to_string(), Ld.to_string());
            }


            // Li: indirect lighting (compute by next iteration)

            // sample BSDF to get new path direction
            bsdf_sample_t bs = isect.bsdf()->sample(isect.wo, sampler->get_float2());

            if (bs.f.is_black() || bs.pdf == 0.f)
                break;

            // update path throughout
            beta *= bs.f * abs_dot(bs.wi, isect.normal) / bs.pdf;
            CHECK_DEBUG(beta.luminance() > 0.f, "{}", beta.to_string());
            CHECK_DEBUG(!std::isinf(beta.luminance()));

            // TODO
            is_prev_specular = is_delta_bsdf(bs.bsdf_type);
            ray = isect.spawn_ray(bs.wi); 


            // possibly terminate the path with Russian roulette.
            if (bounces > 3)
            {
                float_t beta_max_comp = beta.max_component_value();
                float_t q = std::max((float_t).05, 1 - beta_max_comp);

                if (sampler->get_float() < q)
                    break;
                else
                {
                    beta *= 1 / (1 - q);
                    CHECK_DEBUG(!std::isinf(beta.luminance()));
                }
            }
        }

        return Lo;
    }
};


std::unique_ptr<integrator_t> create_integrator(integrator_enum_t integrator_enum,
    int depth, direct_sample_enum_t direct_sample_enum)
{
    switch (integrator_enum)
    {
    case integrator_enum_t::direct_lighting:
        return std::make_unique<direct_lighting_t>(direct_sample_enum);
    case integrator_enum_t::simple_path_tracing_recursion:
        return std::make_unique<simple_path_tracing_recursion_t>(depth, direct_sample_enum);
    case integrator_enum_t::path_tracing_recursion:
        return std::make_unique<path_tracing_recursion_t>(depth, direct_sample_enum);
    case integrator_enum_t::path_tracing_recursion_defered:
        return std::make_unique<path_tracing_recursion_defered_t>(depth, direct_sample_enum, lighting_enum_t::all);
    case integrator_enum_t::path_tracing_iteration:
        return std::make_unique<path_tracing_iteration_t>(depth, direct_sample_enum);
    }

    return nullptr;
}

#pragma endregion



#pragma region window/input

#pragma endregion

#pragma region interactive/debug

// single ray debug

#pragma endregion



#pragma region main

class profiler_t
{

};

class option_t
{

};

class build_t_
{

};

// TODO: remove params
void render_single_scene(int argc, char* argv[])
{
#define KY_BOX_SCENE
#ifdef KY_BOX_SCENE
    int width = 1024, height = 1024;
    film_t film(width, height); //film.clear(color_t(1., 0., 0.));
    scene_t scene = scene_t::create_cornell_box_scene(
        cornell_box_enum_t::both_small_spheres | cornell_box_enum_t::light_environment, film.get_resolution());
#else
    int width = 1024, height = 618;
    film_t film(width, height); //film.clear(color_t(1., 0., 0.));
    scene_t scene = scene_t::create_mis_scene(film.get_resolution());
#endif // !KY_MIS_SCENE

#ifdef KY_RELEASE
    int samples_per_pixel = argc == 2 ? atoi(argv[1]) / 4 : 16; // # samples per pixel
    std::unique_ptr<sampler_t> sampler =
        std::make_unique<random_sampler_t>(samples_per_pixel);

    auto integrator = create_integrator(integrator_enum_t::path_tracing_iteration, 5, direct_sample_enum_t::both_mis);
    float seconds = timing_seconds([&]()
    { 
        integrator->render(&scene, sampler.get(), &film);
    });
    LOG("\n{} seconds\n", seconds);
#else
    int samples_per_pixel = 1;
    std::unique_ptr<sampler_t> sampler =
        std::make_unique<debug_sampler_t>(samples_per_pixel);

    auto integrator = create_integrator(integrator_enum_t::path_tracing_iteration, 5, direct_sample_enum_t::both_mis);

    //integrator->render(&scene, sampler.get(), &film);
    integrator->debug_area(&scene, sampler.get(), &film, { 265, 239 }, 20, 1);
    //integrator->debug_pixel(&scene, sampler.get(), &film, { 73, 239 });
#endif // KY_RELEASE

    film.store_image("single");
}

void render_debug(int argc, char* argv[])
{
    film_grid_t film(1, 3, 512, 308);
    std::unique_ptr<sampler_t> sampler = std::make_unique<random_sampler_t>(10);
    scene_t scene = scene_t::create_mis_scene(film.get_resolution());

    auto debug_integrator_enums = std::vector<integrator_enum_t>
    {
        integrator_enum_t::position,
        integrator_enum_t::normal,
        integrator_enum_t::basecolor
    };

    for (auto debug_integrator_enum : debug_integrator_enums)
    {
        std::unique_ptr<integrator_t> integrator =
            std::make_unique<debug_integrator_t>(debug_integrator_enum);
        integrator->render(&scene, sampler.get(), &film);

        film.next_subfilm();
    }

    film.store_image("render_debug");
}

void render_multiple_integrator()
{
    auto scene_params = std::vector<std::pair<cornell_box_enum_t, int>>
    {
        { cornell_box_enum_t::light_point, 1 },
        { cornell_box_enum_t::light_direction, 10 },
        { cornell_box_enum_t::light_area, 1 },
        { cornell_box_enum_t::light_environment, 10 },
    };

    auto integrator_enums = std::vector<integrator_enum_t>
    {
        integrator_enum_t::direct_lighting,
        integrator_enum_t::simple_path_tracing_recursion,
        integrator_enum_t::path_tracing_recursion,
        integrator_enum_t::path_tracing_recursion_defered,
        integrator_enum_t::path_tracing_iteration
    };

    film_grid_t film(4, 5, 256, 256); //film.clear(color_t(1., 0., 0.));
    for (auto [scene_enum, spp] : scene_params)
    {
        scene_t scene = scene_t::create_cornell_box_scene(
            cornell_box_enum_t::both_small_spheres | scene_enum, film.get_resolution());
        std::unique_ptr<sampler_t> sampler =
            std::make_unique<random_sampler_t>(spp);

        for (auto integrator_enum : integrator_enums)
        {
            auto integrator = create_integrator(integrator_enum, 5, direct_sample_enum_t::both_mis);
            integrator->render(&scene, sampler.get(), &film);

            film.next_subfilm();
        }
    }

    film.store_image("direct_sample");
}

void render_direct_sample_enum(int argc, char* argv[])
{
    auto scene_params = std::vector<std::pair<cornell_box_enum_t, int>>
    {
        //{ cornell_box_enum_t::light_point, 1 },
        { cornell_box_enum_t::light_direction, 10 },
        //{ cornell_box_enum_t::light_area, 1 },
        //{ cornell_box_enum_t::light_environment, 10 },
    };

    auto sample_enums = std::vector<direct_sample_enum_t>
    {
        //direct_sample_enum_t::bsdf,
        //direct_sample_enum_t::light,
        //direct_sample_enum_t::bsdf_mis,
        //direct_sample_enum_t::light_mis,
        direct_sample_enum_t::both_mis,
    };

    film_grid_t film(3, 2, 256, 256); //film.clear(color_t(1., 0., 0.));
    for (auto [scene_enum, spp] : scene_params)
    {
        std::unique_ptr<sampler_t> sampler =
            std::make_unique<random_sampler_t>(spp);
        scene_t scene = scene_t::create_cornell_box_scene(
            cornell_box_enum_t::both_small_spheres | scene_enum, film.get_resolution());

        for (auto sample_enum : sample_enums)
        {
            std::unique_ptr<integrator_t> integrator =
                std::make_unique<path_tracing_iteration_t>(5, sample_enum);
            integrator->render(&scene, sampler.get(), &film);

            film.next_subfilm();
        }
    }

    film.store_image("direct_sample");
}

void render_multiple_scene(int argc, char* argv[])
{
    auto scene_params = std::vector<std::pair<cornell_box_enum_t, int>>
    {
        { cornell_box_enum_t::light_point, 10 },
        { cornell_box_enum_t::light_direction, 40 },
        { cornell_box_enum_t::light_area, 40 },
        { cornell_box_enum_t::light_environment, 10 },
    };

    auto sample_enums = std::vector<direct_sample_enum_t>
    {
        direct_sample_enum_t::bsdf,
        direct_sample_enum_t::light,
        //direct_sample_enum_t::bsdf_mis,
        //direct_sample_enum_t::light_mis,
        direct_sample_enum_t::both_mis,
    };

    film_grid_t film(3, 4, 256, 256); //film.clear(color_t(1., 0., 0.));
    for (auto sample_enum : sample_enums)
    {
        std::unique_ptr<integrator_t> integrator =
            std::make_unique<path_tracing_iteration_t>(5, sample_enum);

        for (auto [scene_enum, spp] : scene_params)
        {
            std::unique_ptr<sampler_t> sampler =
                std::make_unique<random_sampler_t>(spp);
            scene_t scene = scene_t::create_cornell_box_scene(
                cornell_box_enum_t::both_small_spheres | scene_enum, film.get_resolution());

            integrator->render(&scene, sampler.get(), &film);
            film.next_subfilm();
        }
    }

    /*
    for (auto [scene_enum, spp] : scene_params)
    {
        std::unique_ptr<sampler_t> sampler =
            std::make_unique<random_sampler_t>(spp);
        scene_t scene = scene_t::create_cornell_box_scene(
            cornell_box_enum_t::both_small_spheres | scene_enum, film.get_resolution());

        for (auto sample_enum : sample_enums)
        {
            std::unique_ptr<integrator_t> integrator =
                std::make_unique<path_tracing_iteration_t>(5, sample_enum);
            integrator->render(&scene, sampler.get(), &film);
            
            film.next_cell();
        }
    }
    */

    film.store_image("light_mis");
}

void render_mis_scene(int argc, char* argv[])
{
    film_grid_t film(2, 3, 512, 308); //film.clear(color_t(1., 0., 0.));
    std::unique_ptr<sampler_t> sampler =
        std::make_unique<random_sampler_t>(10);
    scene_t scene = scene_t::create_mis_scene(film.get_resolution());

    auto sample_enums = std::vector<direct_sample_enum_t>
    {
        direct_sample_enum_t::bsdf,
        direct_sample_enum_t::light,
        direct_sample_enum_t::idle,
        direct_sample_enum_t::bsdf_mis,
        direct_sample_enum_t::light_mis,    
        direct_sample_enum_t::both_mis,
    };

    for (auto sample_enum : sample_enums)
    {
        std::unique_ptr<integrator_t> integrator =
            std::make_unique<path_tracing_iteration_t>(5, sample_enum);
        integrator->render(&scene, sampler.get(), &film);

        film.next_subfilm();
    }

    film.store_image("veach_mis");
}

/*
void render_lighting_enum()
{
    film_grid_t film(1, 4, 256, 256); //film.clear(color_t(1., 0., 0.));
    std::unique_ptr<sampler_t> sampler =
        std::make_unique<random_sampler_t>(10);
    scene_t scene = scene_t::create_cornell_box_scene(
        cornell_box_enum_t::both_small_spheres | cornell_box_enum_t::light_area, film.get_resolution());

    auto lighting_enums = std::vector<lighting_enum_t>
    {
        lighting_enum_t::emit,
        lighting_enum_t::direct,
        lighting_enum_t::indirect,
        lighting_enum_t::all,
    };

    for (auto lighing_enum : lighting_enums)
    {
        std::unique_ptr<integrator_t> integrator =
            std::make_unique<path_tracing_recursion_defered_t>(10, direct_sample_enum_t::both_mis, lighing_enum);
        integrator->render(&scene, sampler.get(), &film);

        film.next_cell();
    }

    film.store_image("lighting");
}
*/

int main(int argc, char* argv[])
{
    // TODO: parsing params: ky -h

    render_single_scene(argc, argv);
    //render_debug(argc, argv);
    //render_multiple_integrator();
    //render_direct_sample_enum(argc, argv);
    //render_multiple_scene(argc, argv);
    //render_mis_scene(argc, argv);

    return 0;
}

#pragma endregion
