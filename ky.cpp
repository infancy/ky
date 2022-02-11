#include <cmath>   // smallpt, a Path Tracer by Kevin Beason, 2008
#include <cstdlib> // Make : g++ -O3 -fopenmp smallpt.cpp -o smallpt
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
#include <string>
#include <string_view>
#include <vector>
using namespace std::literals::string_literals;


#pragma region macro

#if defined(_DEBUG)
    #define KY_DEBUG
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

#pragma endregion

// namespace ky

#pragma region math

using uint = uint32_t;
using Float = float; // TODO: undef float_t; using float_t = float;

using radian_t = Float;
using degree_t = Float;

constexpr Float k_epsilon = std::numeric_limits<Float>::epsilon();
constexpr Float k_infinity = std::numeric_limits<Float>::infinity();
constexpr Float k_pi = std::numbers::pi;
constexpr Float k_2pi = 2 * k_pi;
constexpr Float k_pi_over2 = k_pi / 2;
constexpr Float k_pi_over4 = k_pi / 4;
constexpr Float k_inv_pi = std::numbers::inv_pi;
constexpr Float k_inv_2pi = k_inv_pi / 2;
constexpr Float k_inv_4pi = k_inv_pi / 4;

constexpr radian_t radians(degree_t deg) { return (k_pi / 180) * deg; }
constexpr degree_t degrees(radian_t rad) { return (180 / k_pi) * rad; }

constexpr Float lerp(Float a, Float b, Float t) { return a + t * (b - a); }

inline bool is_infinity(const std::floating_point auto x) { return std::isinf(x); }
inline bool is_nan(const std::floating_point auto x) { return std::isnan(x); }

inline bool is_invalid(const std::floating_point auto x) { return is_infinity(x) || is_nan(x); }
inline bool is_valid(const std::floating_point auto x) { return !is_invalid(x); }

#pragma endregion

#pragma region utility

class reference_type_t
{
protected:
    constexpr reference_type_t() = default;

    reference_type_t(const reference_type_t&) = delete;
    reference_type_t& operator=(const reference_type_t&) = delete;
};



template <typename... Ts>
inline void LOG(const std::string_view fmt, const Ts&... args)
{
    auto msg = std::vformat(fmt, std::make_format_args(args...));
    std::printf("%s", msg.c_str());
}

// TODO: file, line
template <typename... Ts>
inline void LOG_ERROR(const std::string_view fmt, const Ts&... args)
{
    auto msg = std::vformat(fmt, std::make_format_args(args...));
    std::printf("%s", msg.c_str());
    throw std::exception(msg.c_str());
}


#define EXPAND( x ) x
#define CHECK1(condition)      if(!(condition)) LOG_ERROR("{}", #condition)
#define CHECK2(condition, msg) if(!(condition)) LOG_ERROR("{}", msg)
#define CHECK3(condition, ...) EXPAND( if(!(condition)) LOG_ERROR(__VA_ARGS__) )
#define GET_MACRO_(_1, _2, _3, _4, _5, NAME, ...) NAME

#define CHECK(...) EXPAND( GET_MACRO_(__VA_ARGS__, CHECK3, CHECK3, CHECK3, CHECK2, CHECK1, UNUSED)(__VA_ARGS__) )

#ifdef KY_DEBUG
    #ifndef LOG_DEBUG
        #define LOG_DEBUG(...) LOG(__VA_ARGS__)
    #endif

    #ifndef DCHECK
        #define DCHECK(...) EXPAND( CHECK(__VA_ARGS__) )
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



struct vec2_t
{
    union
    {
        struct { Float x, y; };
    };

    vec2_t(Float x = 0, Float y = 0) { this->x = x; this->y = y; }

    Float operator[](int index) const
    {
        DCHECK((index >= 0) && (index <= 1));

        if (index == 0) return x;
        else return y;
    }

    vec2_t operator+(const vec2_t& vec2) const { return vec2_t(x + vec2.x, y + vec2.y); }
    vec2_t operator-(const vec2_t& vec2) const { return vec2_t(x - vec2.x, y - vec2.y); }

    friend vec2_t operator*(Float scalar, vec2_t v) { return vec2_t(v.x * scalar, v.y * scalar); }
};

using float2_t = vec2_t;
using point2_t = vec2_t;



struct vec3_t
{        // Usage: time ./smallpt 5000 && xv image.ppm
    union
    {
        struct { Float x, y, z; };
        struct { Float r, g, b; };
        //std::array<Float, 3> a_{};
    };

    vec3_t() : x{ 0 }, y{ 0 }, z{ 0 } {}
    vec3_t(Float x, Float y, Float z) { this->x = x; this->y = y; this->z = z; }

    Float operator[](int i) const { DCHECK(i >= 0 && i < 3); return (&x)[i]; }
    vec3_t operator-() const { return vec3_t(-x, -y, -z); }

    vec3_t& operator+=(const vec3_t& v) { x += v.x; y += v.y; z += v.z; return *this; }
    vec3_t& operator-=(const vec3_t& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    vec3_t& operator*=(Float s) { x *= s; y *= s; z *= s; return *this; }
    vec3_t& operator/=(Float s) { x /= s; y /= s; z /= s; return *this; }

    vec3_t operator+(const vec3_t& v) const { return vec3_t(x + v.x, y + v.y, z + v.z); }
    vec3_t operator-(const vec3_t& v) const { return vec3_t(x - v.x, y - v.y, z - v.z); }
    vec3_t operator*(Float scalar)    const { return vec3_t(x * scalar, y * scalar, z * scalar); }
    vec3_t operator/(Float scalar)    const { return vec3_t(x / scalar, y / scalar, z / scalar); }

    friend vec3_t operator*(Float scalar, vec3_t v) { return vec3_t(v.x * scalar, v.y * scalar, v.z * scalar); }

    friend Float     dot(const vec3_t& u, const vec3_t& v) { return u.dot(v); }
    friend Float abs_dot(const vec3_t& u, const vec3_t& v) { return std::abs(u.dot(v)); }
    friend vec3_t  cross(const vec3_t& u, const vec3_t& v) { return u.cross(v); }
    friend vec3_t normalize(const vec3_t& v) { return v.normalize(); }

    friend Float     cos(const vec3_t& u, const vec3_t& v) { return u.dot(v); }
    friend Float abs_cos(const vec3_t& u, const vec3_t& v) { return std::abs(u.dot(v)); }
    friend vec3_t   lerp(const vec3_t& u, const vec3_t& v, Float t) { return u + t * (v - u); }

    Float dot(const vec3_t& v) const { return x * v.x + y * v.y + z * v.z; }
    vec3_t cross(const vec3_t& v) const
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

    // or length(), abs(), absolute_value()
    Float magnitude()    const { return sqrt(magnitude_squared()); }
    Float magnitude_squared() const { return x * x + y * y + z * z; }

    // unit_vec3_t& normlize() const { return *this * (1 / sqrt(x * x + y * y + z * z)); }
    vec3_t normalize() const { return *this * (1 / sqrt(x * x + y * y + z * z)); }
    bool is_unit() const { return is_equal(magnitude(), (Float)1); }

public:
    // per-component
    friend vec3_t min(const vec3_t& a, const vec3_t& b)
    {
        return vec3_t(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
    }
    // per-component
    friend vec3_t max(const vec3_t& a, const vec3_t& b)
    {
        return vec3_t(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
    }


public:
    // TODO
    // only for color_t
    vec3_t& operator*=(const vec3_t& v) { r *= v.r; g *= v.g; b *= v.b; return *this; }
    vec3_t operator*(const vec3_t& v) const { return vec3_t(r * v.r, g * v.g, b * v.b); }

    Float max_component_value() const
    {
        return std::max({ r, g, b });
    }

    Float luminance()
    {
        return
            0.212671f * r +
            0.715160f * g +
            0.072169f * b;
    }

    bool is_black() const { return (r <= 0) && (g <= 0) && (b <= 0); }

public:
    // deubg
    bool is_valid() const { return ::is_valid(x) && ::is_valid(y) && ::is_valid(z); }
    bool has_negative() const { return (x < 0) || (y < 0) || (z < 0); }
    bool small_than(vec3_t vec3) const { return (x < vec3.x) || (y < vec3.y) || (z < vec3.z); }

    std::string to_string() const { return std::format("[{}, {}, {}]", x, y, z);  }
    friend std::ostream& operator<<(std::ostream& console, const vec3_t& vec3)
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

using float3_t = vec3_t;
using point3_t = vec3_t; // object_point, world_point... we need a frame
using normal_t = vec3_t;
using unit_vec3_t = vec3_t;

// TODO
using color_t  = vec3_t;


inline Float distance(const point3_t& p1, const point3_t& p2)
{
    return (p1 - p2).magnitude();
}
inline Float distance_squared(const point3_t& p1, const point3_t& p2)
{
    return (p1 - p2).magnitude_squared();
}


// unit direction vector -> spherical coordinate
inline radian_t spherical_theta(const unit_vec3_t& v)
{
    return std::acos(std::clamp(v.z, (Float)-1, (Float)1));
}
inline radian_t spherical_phi(const unit_vec3_t& v)
{
    Float phi = std::atan2(v.y, v.x);
    return (phi < 0) ? (phi + 2 * k_pi) : phi;
}

// convert spherical coordinate (θ theta, φ phi) into direction vector (x, y, z)
inline vec3_t spherical_to_direction(Float sin_theta, Float cos_theta, Float phi)
{
    return vec3_t(
        sin_theta * std::cos(phi),
        sin_theta * std::sin(phi),
        cos_theta);
}
// takes three basis vectors representing the x, y, and z axes and
// returns the appropriate direction vector with respect to the coordinate frame defined by them
inline vec3_t spherical_to_direction(
    Float sin_theta, Float cos_theta, Float phi,
    const vec3_t& x, const vec3_t& y, const vec3_t& z)
{
    return
        sin_theta * std::cos(phi) * x + 
        sin_theta * std::sin(phi) * y +
        cos_theta * z;
}



/*
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
        Float min = std::numeric_limits<Float>::lowest();
        Float max = std::numeric_limits<Float>::max();

        min_ = point3_t(max, max, max);
        max_ = point3_t(min, min, min);
    }

    explicit bounds3_t(const point3_t& p) : min_(p), max_(p)
    {
    }

    bounds3_t(const point3_t& p1, const point3_t& p2): 
        min_(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z)),
        max_(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z))
    {
    }

public:
    // union, merge
    bounds3_t join(const point3_t& p) const
    {
        return bounds3_t(min(min_, p), max(max_, p));
    }
    bounds3_t join(const bounds3_t& b) const
    {
        return bounds3_t(min(min_, b.min_), max(max_, b.max_));
    }

    friend bounds3_t join(const bounds3_t& b, const point3_t& p) { return b.join(p); }
    friend bounds3_t join(const bounds3_t& b1, const bounds3_t& b2) { return b1.join(b2); }

public:
    bool contain(const point3_t& p) const
    {
        return
            p.x >= min_.x && p.x <= max_.x &&
            p.y >= min_.y && p.y <= max_.y &&
            p.z >= min_.z && p.z <= max_.z;
    }

public:
    // return a sphere that hold this bounding box
    void bounding_sphere(point3_t* center, Float* radius_) const
    {
        *center = lerp(min_, max_, (Float)0.5);
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
    frame_t(const vec3_t& s, const vec3_t& t, const normal_t& n) :
        s_{ s.normalize() },
        t_{ t.normalize() },
        n_{ n.normalize() }
    {
    }

    frame_t(const normal_t& n) :
        n_{ n.normalize()}
    {
        set_from_z();
    }

public:
    // think if {s, t, n} is (1, 0, 0), (0, 1, 0), (0, 0, 1)
    vec3_t to_local(const vec3_t& world_vec3) const
    {
        return vec3_t(
            dot(s_, world_vec3),
            dot(t_, world_vec3),
            dot(n_, world_vec3));
    }

    vec3_t to_world(const vec3_t& local_vec3) const
    {
        return 
            s_ * local_vec3.x + 
            t_ * local_vec3.y + 
            n_ * local_vec3.z;
    }

    const vec3_t& binormal() const { return s_; }
    const vec3_t& tangent() const { return t_; }
    const vec3_t& normal() const { return n_; }

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
    ray_t(const point3_t& origin, const unit_vec3_t& direction, const Float distance = k_infinity) :
        origin_{ origin },
        direction_{ direction },
        distance_{ distance }
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

    point3_t origin() const { return origin_; }
    unit_vec3_t direction() const { return direction_; }
    Float distance() const { return distance_; }

    // TODO
    void set_distance(Float distance) const { distance_ = distance; }

    point3_t operator()(Float t) const
    {
        DCHECK(t >= 0);
        return origin_ + t * direction_;
    }

private:
    point3_t origin_;
    unit_vec3_t direction_; // confirm it is a unit vector
    mutable Float distance_; // distance from ray to intersection
};



// TODO
class bsdf_t;
using bsdf_uptr_t = std::unique_ptr<bsdf_t>;
class material_t;
class area_light_t;
class surface_t;

/*
  prev   n   light
  ----   ^   -----
    ^    |    ^
     \   | θ /
   wo \  |  / wi is unknown, sampling for bsdf or light
       \ | / 
        \|/
      -------
       isect
*/

// surface intersection
class isect_t
{
public:
    isect_t() = default;
    isect_t(const point3_t& position, const normal_t& normal, unit_vec3_t wo) :
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
    ray_t spawn_ray(const unit_vec3_t& direction) const
    {
        // TODO: offset
        return ray_t(position + direction * k_epsilon, direction);
    }

    ray_t spawn_ray_to(const point3_t& target) const
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
    surface_t* surface_{};
    bsdf_uptr_t bsdf_{};
    color_t emission_{};

    friend surface_t;
};

#pragma endregion



#pragma region sampling

inline point2_t sample_disk_uniform(const point2_t& random)
{
    Float r = std::sqrt(random[0]);
    Float theta = 2 * k_pi * random[1];
    return point2_t(r * std::cos(theta), r * std::sin(theta));
}

inline point2_t sample_disk_concentric(const point2_t& random)
{
    // Map uniform random numbers to $[-1,1]^2$
    point2_t uOffset = 2.f * random - vec2_t(1, 1);

    // Handle degeneracy at the origin
    if (uOffset.x == 0 && uOffset.y == 0)
        return point2_t(0, 0);

    // Apply concentric mapping to point
    Float theta, r;
    if (std::abs(uOffset.x) > std::abs(uOffset.y))
    {
        r = uOffset.x;
        theta = k_pi_over4 * (uOffset.y / uOffset.x);
    }
    else
    {
        r = uOffset.y;
        theta = k_pi_over2 - k_pi_over4 * (uOffset.x / uOffset.y);
    }

    return r * point2_t(std::cos(theta), std::sin(theta));
}


// cosine-weighted sampling
inline vec3_t sample_hemisphere_cosine(const point2_t& random)
{
    point2_t d = sample_disk_concentric(random);
    Float z = std::sqrt(std::max((Float)0, 1 - d.x * d.x - d.y * d.y));
    return vec3_t(d.x, d.y, z);
}

inline Float pdf_hemisphere_cosine(Float cos_theta) { return cos_theta * k_inv_pi; }


inline vec3_t sample_sphere_uniform(const point2_t& random)
{
    Float z = 1 - 2 * random[0]; // (-1, 1)

    Float r = std::sqrt(std::max((Float)0, (Float)1 - z * z));
    Float phi = 2 * k_pi * random[1];

    return vec3_t(r * std::cos(phi), r * std::sin(phi), z);
}

// TODO: pdf_uniform_shphere
inline Float pdf_sphere_uniform() { return k_inv_4pi; }


/*

        /         _
       /        / O \
      /         O O O (a sphere)
     /       .  \ O /
    /    .
   / .     theta
  . _ _ _ _ _ _ _ _

*/
vec3_t sample_cone_uniform(const point2_t& random, Float cos_theta_max)
{
    Float cos_theta = ((Float)1 - random[0]) + random[0] * cos_theta_max;
    Float sin_theta = std::sqrt((Float)1 - cos_theta * cos_theta);
    Float phi = random[1] * 2 * k_pi;

    return vec3_t(
        std::cos(phi) * sin_theta,
        std::sin(phi) * sin_theta,
        cos_theta);
}

Float pdf_cone_uniform(Float cos_theta_max)
{
    return 1 / (2 * k_pi * (1 - cos_theta_max));
}


point2_t sample_triangle_uniform(const point2_t& random)
{
    Float su0 = std::sqrt(random[0]);
    return point2_t(1 - su0, random[1] * su0);
}


inline Float power_heuristic(int nf, Float fPdf, int ng, Float gPdf)
{
    Float f = nf * fPdf, g = ng * gPdf;
    return (f * f) / (f * f + g * g);
}

#pragma endregion

#pragma region sampler

// TODO: Pseudo or Quasi
// random number generator
// https://github.com/SmallVCM/SmallVCM/blob/master/src/rng.hxx
class rng_t
{
public:
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
    Float uniform_float01()
    {
        return float_dist_(rng_engine_);
    }

    // [0, 1) * [0, 1)
    vec2_t uniform_vec2()
    {
        return vec2_t(uniform_float01(), uniform_float01());
    }

private:
    std::mt19937_64 rng_engine_;

    std::uniform_int_distribution<int> int_dist_;
    std::uniform_int_distribution<uint32_t> uint_dist_;
    std::uniform_real_distribution<Float> float_dist_{ (Float)0, (Float)1 };
};


struct camera_sample_t
{
    point2_t p_film{}; // film_sample_point
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

    virtual int ge_samples_per_pixel()
    {
        return samples_per_pixel_;
    }
    virtual void set_samples_per_pixel(int samples_per_pixel)
    {
        samples_per_pixel_ = samples_per_pixel;
    }

    virtual std::unique_ptr<sampler_t> clone() = 0;

public:
    virtual void start_sample()
    {
        current_sample_index_ = 0;
    }

    virtual bool next_sample()
    {
        current_sample_index_ += 1;
        return current_sample_index_ < samples_per_pixel_;
    }

public:
    virtual Float get_float() = 0;
    virtual vec2_t get_vec2() = 0;
    virtual camera_sample_t get_camera_sample(point2_t p_film) = 0;

protected:
    rng_t rng_{};

    int samples_per_pixel_{};
    int current_sample_index_{};
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
    Float get_float() override
    {
        return rng_.uniform_float01();
    }

    vec2_t get_vec2() override
    {
        return rng_.uniform_vec2();
    }

    // TODO: coroutine
    camera_sample_t get_camera_sample(point2_t p_film) override
    {
        return { p_film + rng_.uniform_vec2() };
    }
};

class stratified_sampler_t : public sampler_t
{
public:
    camera_sample_t get_camera_sample(point2_t p_film) override
    {
        return { p_film + rng_.uniform_vec2() };
    }
};

#pragma endregion



#pragma region shape

class shape_t
{
public:
    virtual bool intersect(const ray_t& ray, isect_t* out_isect) const = 0;

    virtual bounds3_t world_bound() const = 0;
    virtual Float area() const = 0;

public:
    // sample direction on front vertexs, and sample position/point/area on last vertex

    // TODO: return position_sample_t
    virtual isect_t sample_position(const point2_t& random, Float* out_pdf_position) const = 0;
    virtual Float pdf_position(const isect_t& isect) const { return 1 / area(); }

    // TODO: return direction_sample_t
    // default compute `*_direction` by `*_position` 
    virtual isect_t sample_direction(const isect_t& isect, const point2_t& random, Float* out_pdf_direction) const
    {
        isect_t light_isect = sample_position(random, out_pdf_direction);
        vec3_t wi = light_isect.position - isect.position;

        if (wi.magnitude_squared() == 0)
            *out_pdf_direction = 0;
        else
        {
            wi = normalize(wi);
            // 1 / (projected area / distance squared)
            *out_pdf_direction *= distance_squared(light_isect.position, isect.position) / abs_dot(isect.normal, -wi);
            if (std::isinf(*out_pdf_direction))
                *out_pdf_direction = 0.f;
        }

        return std::move(light_isect);
    }
    virtual Float pdf_direction(const isect_t& isect, const unit_vec3_t& world_wi) const
    {
        ray_t ray = isect.spawn_ray(world_wi);
        isect_t light_isect;

        if (!intersect(ray, &light_isect))
            return 0;

        // convert light sample weight to solid angle measure
        // 1 / (projected area / distance squared)
        Float pdf = distance_squared(isect.position, light_isect.position) / (abs_dot(light_isect.normal, -world_wi) * area());
        if (std::isinf(pdf))
            pdf = 0.f;

        return pdf;
    }

protected:
    static constexpr Float epsilon = 1e-3;// TODO
};

using shape_sp = std::shared_ptr<shape_t>; 
using shape_list_t = std::vector<shape_sp>;



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

// TODO: disk
class plane_t : public shape_t
{
public:
    plane_t(const point3_t& p, const normal_t& normal)
    {
        p_ = p;
        normal_ = normal;
    }

    bool intersect(const ray_t& ray, isect_t* out_isect) const override
    {
        if (is_equal(dot(ray.direction(), normal_), (Float)0))
            return false;
 
        const vec3_t op = p_ - ray.origin();
        const Float distance = dot(normal_, op) / dot(normal_, ray.direction());

        if ((distance > epsilon) && (distance < ray.distance()))
        {
            ray.set_distance(distance);
            point3_t hit_point = ray(distance);
            *out_isect = isect_t(hit_point, normal_, -ray.direction());

            return true;
        }
    }

    bounds3_t world_bound() const override
    {
        return bounds3_t(); // TODO
    }

    Float area() const override { return k_infinity; } // TODO

public:
    point3_t p_;
    normal_t normal_;
};

class triangle_t : public shape_t
{
public:
    triangle_t(const point3_t& p0, const point3_t& p1, const point3_t& p2)
    {
        p0_ = p0;
        p1_ = p1;
        p2_ = p2;

        normal_ = normalize(cross(p1_ - p0_, p2_ - p0_));
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

        const Float v0d = dot(v0, ray.direction());
        const Float v1d = dot(v1, ray.direction());
        const Float v2d = dot(v2, ray.direction());

        if (((v0d <  0.f) && (v1d <  0.f) && (v2d <  0.f)) ||
            ((v0d >= 0.f) && (v1d >= 0.f) && (v2d >= 0.f)))
        {
            // 1. first calculate the vertical distance from ray.origin to the plane,
            //    by `dot(normal, op)` (or `bo`, `co`)
            // 2. then calculate the distance from ray.origin to the plane alone ray.direction, 
            //    by `distance * dot(normal, ray.direction()) = vertical_distance`
            const Float distance = dot(normal_, oa) / dot(normal_, ray.direction());

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

    Float area() const override { return 0.5 * cross(p1_ - p0_, p2_ - p0_).magnitude(); }

public:
    isect_t sample_position(const point2_t& random, Float* pdf) const override
    {
        point2_t b = sample_triangle_uniform(random);

        isect_t isect;
        isect.position = b.x * p0_ + b.y * p1_ + (1 - b.x - b.y) * p2_;
        isect.normal = normal_;

        *pdf = 1 / area();
        return std::move(isect);
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
    rectangle_t(const point3_t& p0, const point3_t& p1, const point3_t& p2, const point3_t& p3)
    {
        p0_ = p0;
        p1_ = p1;
        p2_ = p2;
        p3_ = p3;
        // TODO: DCHECK

        normal_ = normalize(cross(p1_ - p0_, p2_ - p0_));
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

        const Float v0d = dot(v0, ray.direction());
        const Float v1d = dot(v1, ray.direction());
        const Float v2d = dot(v2, ray.direction());
        const Float v3d = dot(v3, ray.direction());

        if (((v0d <  0.f) && (v1d <  0.f) && (v2d <  0.f) && (v3d <  0.f)) ||
            ((v0d >= 0.f) && (v1d >= 0.f) && (v2d >= 0.f) && (v3d >= 0.f)))
        {
            const Float distance = dot(normal_, oa) / dot(normal_, ray.direction());

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
        return bounds3_t(p0_, p1_).join(p2_).join(p3_);
    }

    Float area() const override { return cross(p1_ - p0_, p2_ - p0_).magnitude(); }

public:
    isect_t sample_position(const point2_t& random, Float* pdf) const override
    {
        isect_t isect;
        isect.position = p1_ + (p1_ - p0_) * random[0] + (p1_ - p2_) * random[1];
        isect.normal = normalize(normal_);

        *pdf = 1 / area();
        return std::move(isect);
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
    sphere_t(vec3_t center, Float radius) :
        center_(center),
        radius_(radius),
        radius_sq_(radius * radius)
    {
    }

    bool intersect(const ray_t& ray, isect_t* out_isect) const override
    { 
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
            = -b' +/- sqrt(discr)
        */

        vec3_t co = center_ - ray.origin();
        Float neg_b = dot(co, ray.direction());
        Float discr = neg_b * neg_b - dot(co, co) + radius_sq_;

        Float distance = 0;
        bool hit = false;
        if (discr >= 0)
         {
            Float sqrt_discr = sqrt(discr);

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

    Float area() const override { return 4 * k_pi * radius_sq_; }

public:
    isect_t sample_position(const point2_t& random, Float* pdf) const override
    {
        unit_vec3_t direction = sample_sphere_uniform(random);
        point3_t position = center_ + radius_ * direction;

        isect_t isect;
        isect.position = position;
        isect.normal = normalize(direction);

        *pdf = 1 / area();

        return std::move(isect);
    }

    // TODO
    isect_t sample_direction(const isect_t& isect, const point2_t& random, Float* pdf) const override
    {
        if (distance_squared(isect.position, center_) <= radius_ * radius_)
        {
            isect_t light_isect = sample_position(random, pdf);
            vec3_t wi = light_isect.position - isect.position;

            if (wi.magnitude_squared() == 0)
                *pdf = 0;
            else
            {
                // Convert from area measure returned by Sample() call above to solid angle measure.
                wi = normalize(wi);
                *pdf *= distance_squared(light_isect.position, isect.position) / abs_dot(isect.normal, -wi);
            }

            if (std::isinf(*pdf))
                *pdf = 0.f;

            return std::move(light_isect);
        }

        // Sample sphere uniformly inside subtended cone

        /*
                /         _
               /        / O \
              /         O O O (a sphere)
             /       .  \ O /
            /    .
           / .     theta
          . _ _ _ _ _ _ _ _

        */

        Float dist = distance(isect.position, center_);
        Float inv_dist = 1 / dist;

        // Compute $\theta$ and $\phi$ values for sample in cone
        Float sin_theta_max = radius_ * inv_dist;
        Float sin_theta_max_sq = sin_theta_max * sin_theta_max;
        Float inv_sin_theta_max = 1 / sin_theta_max;
        Float cos_theta_max = std::sqrt(std::max((Float)0.f, 1 - sin_theta_max_sq));

        Float cos_theta = (cos_theta_max - 1) * random[0] + 1;
        Float sin_theta_sq = 1 - cos_theta * cos_theta;

        if (sin_theta_max_sq < 0.00068523f /* sin^2(1.5 deg) */)
        {
            /* Fall back to a Taylor series expansion for small angles, where
               the standard approach suffers from severe cancellation errors */
            sin_theta_sq = sin_theta_max_sq * random[0];
            cos_theta = std::sqrt(1 - sin_theta_sq);
        }

        // Compute angle $\alpha$ from center of sphere to sampled point on surface
        Float cos_alpha = sin_theta_sq * inv_sin_theta_max +
            cos_theta * std::sqrt(std::max((Float)0.f, 1.f - sin_theta_sq * inv_sin_theta_max * inv_sin_theta_max));
        Float sin_alpha = std::sqrt(std::max((Float)0.f, 1.f - cos_alpha * cos_alpha));
        Float phi = random[1] * 2 * k_pi;

        // Compute coordinate system for sphere sampling
        vec3_t normal = (center_ - isect.position) * inv_dist;
        frame_t frame{ normal };

        // Compute surface normal and sampled point on sphere
        vec3_t world_normal =
            spherical_to_direction(sin_alpha, cos_alpha, phi, -frame.binormal(), -frame.tangent(), -frame.normal());
        point3_t world_position = center_ + radius_ * point3_t(world_normal.x, world_normal.y, world_normal.z);

        isect_t light_isect;
        light_isect.position = world_position;
        light_isect.normal = world_normal;

        // Uniform cone PDF.
        *pdf = 1 / (2 * k_pi * (1 - cos_theta_max));

        return std::move(light_isect);
    }

    Float pdf_direction(const isect_t& isect, const vec3_t& world_wi) const override
    {
        // Return uniform PDF if point is inside sphere
        if (distance_squared(isect.position, center_) <= radius_ * radius_)
            return shape_t::pdf_direction(isect, world_wi);

        // Compute general sphere PDF
        Float sin_theta_max_sq = radius_ * radius_ / distance_squared(isect.position, center_);
        Float cos_theta_max = std::sqrt(std::max((Float)0, 1 - sin_theta_max_sq));
        return pdf_cone_uniform(cos_theta_max);
    }

private:
    vec3_t center_;
    Float radius_;
    Float radius_sq_;
};

#pragma endregion



#pragma region filter

#pragma endregion

#pragma region film

// film_option_t
struct film_desc_t
{
    int width;
    int height;
};

constexpr Float clamp01(Float x) { return std::clamp(x, (Float)0, (Float)1); }
inline vec3_t clamp01(vec3_t vec3) { return vec3_t(clamp01(vec3.x), clamp01(vec3.y), clamp01(vec3.z)); }

inline std::byte gamma_encoding(Float x) { return std::byte(pow(clamp01(x), 1 / 2.2) * 255 + .5); }

/*
  warpper of `color_t pixels[]`
  * get/set color
  * save image
*/
class film_t : public reference_type_t
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
    
    virtual vec2_t get_resolution() const { return { (Float)width_, (Float)height_ }; }
    virtual color_t& operator()(int x, int y)
    {
        DCHECK(x >= 0 && x < width_ && y >= 0 && y < height_, 
            "out of bound: {}, {}", x, y);
        return *(pixels_.get() + get_width() * y + x);
    }

    void set_color(int x, int y, const color_t& color)
    {
        operator()(x, y) = color;
    }

    void add_color(int x, int y, const color_t& delta)
    {
        auto& color_ = operator()(x, y);
        color_ = color_ + delta;
    }

    void clear(color_t color)
    {
        for (int i = 0; i < get_pixel_num(); ++i)
        {
            pixels_[i] = color;
        }
    }

public:
    //virtual bool store_image(bool with_alpha);
    virtual bool store_image(std::string filename, bool with_alpha = false) const
    {
        return store_bmp_impl(filename, get_width(), get_height(), get_channels(), (Float*)pixels_.get());
    }
   
    // https://github.com/SmallVCM/SmallVCM/blob/master/src/framebuffer.hxx#L149-L215
    // https://github.com/skywind3000/RenderHelp/blob/master/RenderHelp.h#L937-L1018
    static bool store_bmp_impl(const std::string& filename, int width, int height, int channel, const Float* floats)
    {
        std::fstream bmp_file(filename, std::ios::binary | std::ios::out);


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
            uint32_t databody_offset{};

            // info header
            uint32_t	info_header_size{ INFO_HEADER_SIZE };

            int32_t     width{};
            int32_t		height{};
            uint16_t	planes{ 1 };
            uint16_t	per_pixel_bits{};
            uint32_t	compression{ 0 };
            uint32_t	image_bytes{};

            uint32_t	x_pixels_per_meter{ 0xb12 };
            uint32_t	y_pixels_per_meter{ 0xb12 };
            uint32_t	color_used{ 0 };
            uint32_t	color_important{ 0 };
        }
        bmp_header
        {
            .file_size{ FILE_HEADER_SIZE + INFO_HEADER_SIZE + padding_image_bytes },
            .width{ width },
            .height{ height },
            .per_pixel_bits{ (uint16_t)(channel * 8) },
            .image_bytes{ uint32_t(padding_image_bytes) }
        };

        bmp_file
            .write("BM", 2)
            .write((char*)&bmp_header, FILE_HEADER_SIZE + INFO_HEADER_SIZE);
        

        // without color table


        // gamma encoding
        int byte_num = width * height * channel;
        auto bytes = std::make_unique<std::byte[]>(byte_num);
        for (int i = 0; i < byte_num; i += 3)
        {
            // BGR
            bytes[i]     = gamma_encoding(floats[i + 2]);
            bytes[i + 1] = gamma_encoding(floats[i + 1]);
            bytes[i + 2] = gamma_encoding(floats[i]);
        }

        // write data body 
        int line_num = width * channel;
        // bmp is stored from bottom to up
        for (int y = height - 1; y >= 0; --y)
            bmp_file.write((char*)(bytes.get() + y * line_num), line_num);


        return true;
    }

private:
    int32_t width_;
    int32_t height_;

    std::unique_ptr<color_t[]> pixels_;
};

// multi film
class film_grid_t : public film_t
{
public:
    film_grid_t(int row, int column, int width, int height) :
        film_t(width * column, height * row),
        row_{ row },
        column_{ column },
        sub_width_{ width },
        sub_height_{ height }
    {
    }

public:
    vec2_t get_resolution() const override { return { (Float)sub_width_, (Float)sub_height_ }; }

    color_t& operator()(int x, int y) override
    {
        int col_index = currrent_index_ % column_;
        int row_index = currrent_index_ / column_;
        return film_t::operator()(x + col_index * sub_width_ , y + row_index * sub_height_);
    }

    void next_cell()
    {
        ++currrent_index_;
    }

private:
    int row_;
    int column_;
    int currrent_index_{};

    int sub_width_;
    int sub_height_;
};

#pragma endregion

#pragma region camera

/*
 * OpenGL-style
 * generate ray
 * 
 *
 */

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
*/

class camera_t
{
public:
    virtual ~camera_t() {}

    camera_t(
        const vec3_t& position, const vec3_t& front, const vec3_t& up,
        degree_t fov, vec2_t resolution):
        position_{ position },
        front_{ front.normalize() },
        up_{ up },
        resolution_{ resolution }
    {
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/transform.cpp#L394-L397
 
        Float tan_fov = std::tan(radians(fov) / 2);

        // left hand
        right_ = up_.cross(front_).normalize() * tan_fov * get_aspect();
        up_ = front_.cross(right_).normalize() * tan_fov;
    }

public:
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
    Float get_aspect() { return resolution_.x / resolution_.y; }

private:
    vec3_t position_;
    unit_vec3_t front_;
    unit_vec3_t right_;
    unit_vec3_t up_;

    vec2_t resolution_;
};

using const_camera_sptr_t = std::shared_ptr<const camera_t>;

// TODO: smallpt_camera

#pragma endregion



#pragma region bsdf utility

inline Float cos_theta(const vec3_t& w) { return w.z; }
inline Float abs_cos_theta(const vec3_t& w) { return std::abs(w.z); }

inline bool same_hemi_sphere(const vec3_t& w, const vec3_t& wp) { return w.z * wp.z > 0; }

inline vec3_t reflect(const vec3_t& wo, const normal_t& normal)
{
    // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#SpecularReflection

    return -wo + 2 * dot(wo, normal) * normal;
}

// eta = eta_i/eta_t
inline bool refract(const vec3_t& wo, const normal_t& normal, Float eta, vec3_t* out_wt)
{
    // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#SpecularTransmission

    // Compute $\cos \theta_\roman{t}$ using Snell's law
    Float cos_theta_i = dot(normal, wo);
    Float sin_theta_i_sq = std::max(Float(0), Float(1 - cos_theta_i * cos_theta_i));
    Float sin_theta_t_sq = eta * eta * sin_theta_i_sq;

    // Handle total internal reflection for transmission
    if (sin_theta_t_sq >= 1)
        return false;

    Float cos_theta_t = std::sqrt(1 - sin_theta_t_sq);
    *out_wt = eta * -wo + (eta * cos_theta_i - cos_theta_t) * vec3_t(normal);

    return true;
}

#pragma endregion

#pragma region fresnel

Float fresnel_dielectric(
    Float cos_theta_i, 
    Float eta_i, Float eta_t)
{
    // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.cpp#L66-L90

    cos_theta_i = std::clamp(cos_theta_i, (Float)-1, (Float)1);

    bool entering = cos_theta_i > 0.f;
    if (!entering)
    {
        std::swap(eta_i, eta_t);
        cos_theta_i = std::abs(cos_theta_i);
    }


    // Compute _cos_thetaT_ using Snell's law
    Float sin_theta_i = std::sqrt(std::max((Float)0, 1 - cos_theta_i * cos_theta_i));
    Float sin_theta_t = eta_i / eta_t * sin_theta_i;

    // Handle total internal reflection
    if (sin_theta_t >= 1)
        return 1;

    Float cos_theta_t = std::sqrt(std::max((Float)0, 1 - sin_theta_t * sin_theta_t));


    Float r_para = ((eta_t * cos_theta_i) - (eta_i * cos_theta_t)) /
                   ((eta_t * cos_theta_i) + (eta_i * cos_theta_t));
    Float r_perp = ((eta_i * cos_theta_i) - (eta_t * cos_theta_t)) /
                   ((eta_i * cos_theta_i) + (eta_t * cos_theta_t));
    return (r_para * r_para + r_perp * r_perp) / 2;
}


Float fresnel_dielectric_schlick(
    Float cos_theta_i, 
    Float eta_i, Float eta_t)
{
    Float R0 = (eta_t - eta_i) / (eta_t + eta_i);
    R0 *= R0;

    return lerp(R0, 1.0f, std::pow(1 - cos_theta_i, 5.0f));
}

Float fresnel_dielectric_schlick(
    Float cos_theta_i, Float cos_theta_t, 
    Float eta_i, Float eta_t)
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

    Float R0 = (eta_t - eta_i) / (eta_t + eta_i);
    R0 *= R0;

    //Float cos_i = eta_i < eta_t ? cos_theta_i : cos_theta_t;
    Float cos_i = cos_theta_i < 0 ? -cos_theta_i : cos_theta_t;

    return lerp(R0, 1.0f, std::pow(1 - cos_i, 5.0f) );
}


/*
  given
    the reflectance at normal incidence `R0` and
    the cosine of the angle of incidence `cos_theta_i`,
  compute reflectance
*/
Float fresnel_dielectric_schlick(Float R0, Float cos_theta_i)
{
    return lerp(R0, 1.0f, std::pow((1.0f - cos_theta_i), 5.0f));
}

/*
vec3_t fresnel_dielectric_schlick(vec3_t R0, Float cos_theta_i)
{
    return lerp(R0, vec3_t(1.0f), std::pow((1.0f - cos_theta_i), 5.0f));
}
*/



class fresnel_t
{
public:
    virtual ~fresnel_t() = default;

    virtual Float Evaluate(Float cosI) const = 0;
};

class fresnel_dielectric_t : public fresnel_t
{
public:
    fresnel_dielectric_t()
    {
    }

    Float Evaluate(Float cosI) const override
    {
        return 0;
    }
};

// class fresnel_dummy_t : public fresnel_t

#pragma endregion

#pragma region bsdf

enum class bsdf_enum_t
{
    reflection = 1,
    transmission = 2,
    scattering = reflection | transmission,

    diffuse = 4,
    glossy = 8,
    specluar = 16,
};

// TODO: KY_ENUM_OPERATORS(bsdf_enum_t)
bsdf_enum_t operator|(bsdf_enum_t a, bsdf_enum_t b)
{
    return bsdf_enum_t((int)a | (int)b);
}

inline bool is_delta_bsdf(bsdf_enum_t bsdf_type)
{
    return ((int)bsdf_type & (int)bsdf_enum_t::specluar) > 0;
}

/* 
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

  prev   n   light
  ----   ^   -----
    ^    |    ^
     \   | θ /
   wo \  |  / wi is unknown, sampling for bsdf or light
       \ | /
        \|/
      -------
       isect

   https://www.pbr-book.org/3ed-2018/Reflection_Models#x0-GeometricSetting
*/

struct bsdf_sample_t
{
    color_t f; // scattering rate 
    vec3_t wi; // world wi
    Float pdf;
    bsdf_enum_t bsdf_type; // flags
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

    // or called `eval`, `evaluate`
    color_t f(const vec3_t& world_wo, const vec3_t& world_wi) const
    {
        return f_(to_local(world_wo), to_local(world_wi));
    }
    Float pdf(const vec3_t& world_wo, const vec3_t& world_wi) const
    {
        return pdf_(to_local(world_wo), to_local(world_wi));
    }


    color_t f(const vec3_t& world_wo, const vec3_t& world_wi, Float* out_pdf) const
    {
        vec3_t wo = to_local(world_wo), wi = to_local(world_wi);
        *out_pdf = pdf_(wo, wi);
        return f_(wo, wi);
    }

    // or called `sample`, `sample_direction`, `sample_solid_angle`
    bsdf_sample_t sample_f(const vec3_t& world_wo, const point2_t& random) const
    {
        auto sample = sample_f_(to_local(world_wo), random);
        sample.wi = to_world(sample.wi);

        return sample;
    }

protected: 
    virtual color_t f_(const vec3_t& wo, const vec3_t& wi) const = 0;
    virtual Float pdf_(const vec3_t& wo, const vec3_t& wi) const = 0;

    virtual bsdf_sample_t sample_f_(const vec3_t& wo, const point2_t& random) const = 0;

private:
    vec3_t to_local(const vec3_t& world_vec3) const
    {
        return shading_frame_.to_local(world_vec3);
    }

    vec3_t to_world(const vec3_t& local_vec3) const
    {
        return shading_frame_.to_world(local_vec3);
    }

private:
    frame_t shading_frame_;
};



class lambertion_reflection_t : public bsdf_t
{
public:
    lambertion_reflection_t(const frame_t& shading_frame, const color_t& R) :
        bsdf_t(shading_frame), R_{ R }
    {
    }

    bool is_delta() const override { return false; }

    color_t f_(const vec3_t& wo, const vec3_t& wi) const override { return R_ * k_inv_pi; }
    Float pdf_(const vec3_t& wo, const vec3_t& wi) const override
    {
        return same_hemi_sphere(wo, wi) ? abs_cos_theta(wi) * k_inv_pi : 0;
    }

    bsdf_sample_t sample_f_(const vec3_t& wo, const point2_t& random) const override
    {
        bsdf_sample_t sample;
        // Cosine-sample the hemisphere, flipping the direction if necessary
        sample.wi = sample_hemisphere_cosine(random);

        if (wo.z < 0) 
            sample.wi.z *= -1;

        sample.pdf = pdf_(wo, sample.wi);
        sample.f = f_(wo, sample.wi);
        sample.bsdf_type = bsdf_enum_t::reflection | bsdf_enum_t::diffuse;

        return sample;
    }

private:
    color_t R_; // surface reflectance
};


class specular_reflection_t : public bsdf_t
{
public:
    specular_reflection_t(const frame_t& shading_frame, const color_t& R) :
        bsdf_t(shading_frame), R_{ R }
    {
    }

    bool is_delta() const override { return true; }

    color_t f_(const vec3_t& wo, const vec3_t& wi) const override { return color_t(); }
    Float pdf_(const vec3_t& wo, const vec3_t& wi) const override { return 0; }

    bsdf_sample_t sample_f_(const vec3_t& wo, const point2_t& random) const override
    {
        // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#SpecularReflection
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.h#L387-L408
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.cpp#L181-L191
        
        bsdf_sample_t sample;
        //sample.wi = reflect(wo, vec3_t(0, 0, 1));
        sample.wi = vec3_t(-wo.x, -wo.y, wo.z);
        sample.pdf = 1;
        sample.f = R_ / abs_cos_theta(sample.wi);
        sample.bsdf_type = bsdf_enum_t::reflection | bsdf_enum_t::specluar;

        return sample;
    }

private:
    color_t R_;
};

// TODO
class specular_transmission_t : public bsdf_t
{
public:
    specular_transmission_t(const frame_t& shading_frame, const color_t& T) :
        bsdf_t(shading_frame), T_{ T }
    {
    }

    bool is_delta() const override { return true; }

    color_t f_(const vec3_t& wo, const vec3_t& wi) const override { return color_t(); }
    Float pdf_(const vec3_t& wo, const vec3_t& wi) const override { return 0; }

    bsdf_sample_t sample_f_(const vec3_t& wo, const point2_t& random) const override
    {
        // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#SpecularTransmission
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.h#L410-L436
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.cpp#L198-L218

        bsdf_sample_t sample;
        sample.wi = vec3_t(-wo.x, -wo.y, wo.z); // TODO ERROR
        sample.pdf = 1;
        sample.f = T_ / abs_cos_theta(sample.wi);
        sample.bsdf_type = bsdf_enum_t::transmission | bsdf_enum_t::specluar;

        return sample;
    }

private:
    color_t T_;
};


class fresnel_specular_t : public bsdf_t
{
public:
    fresnel_specular_t(const frame_t& shading_frame, const color_t& R, const color_t& T, Float etaA, Float etaB) :
        bsdf_t(shading_frame), R_{ R }, T_{ T }, etaA_{ etaA }, etaB_{ etaB }
    {
    }

    bool is_delta() const override { return true; }

    color_t f_(const vec3_t& wo, const vec3_t& wi) const override { return color_t(); }
    Float pdf_(const vec3_t& wo, const vec3_t& wi) const override { return 0; }

    bsdf_sample_t sample_f_(const vec3_t& wo, const point2_t& random) const override
    {
        // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#Fresnel-ModulatedSpecularReflectionandTransmission
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.h#L440-L463
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.cpp#L627-L667

        bsdf_sample_t sample;

        Float Re = fresnel_dielectric_schlick(abs_cos_theta(wo), etaA_, etaB_);
        // or Float Re = fresnel_dielectric(cos_theta(wo), etaA_, etaB_);
        Float Tr = 1 - Re;

        if (random[0] < Re)
        {
            // Compute specular reflection for _FresnelSpecular_

            sample.wi = vec3_t(-wo.x, -wo.y, wo.z);
            sample.pdf = Re; // Russian roulette???

            sample.f = (R_ * Re) / abs_cos_theta(sample.wi);
            sample.bsdf_type = bsdf_enum_t::reflection | bsdf_enum_t::specluar;
        }
        else
        {
            // Compute specular transmission for _FresnelSpecular_

            normal_t normal(0, 0, 1);
            bool into = normal.dot(wo) > 0; // ray from outside going in?

            normal_t wo_normal = into ? normal : normal * -1;
            Float eta = into ? etaA_ / etaB_ : etaB_ / etaA_;

            if (!refract(wo, wo_normal, eta, &sample.wi))
            {
                sample.f = color_t(); // total internal reflection
            }
            
            sample.pdf = Tr;
            sample.f = (T_ * Tr) / abs_cos_theta(sample.wi);
            sample.bsdf_type = bsdf_enum_t::transmission | bsdf_enum_t::specluar;
        }

        return sample;
    }

    /*
    // smallpt version
    color_t sample_f_(const vec3_t& wo, const point2_t& random,
        vec3_t* out_wi, Float* out_pdf_direction, bsdf_enum_t* out_bsdf_type) const override
    {
        normal_t normal(0, 0, 1);
        bool into = normal.dot(wo) > 0; // ray from outside going in?

        normal_t wo_normal = into ? normal : normal * -1;
        Float eta = into ? etaA_ / etaB_ : etaB_ / etaA_;

        if (!refract(wo, wo_normal, eta, out_wi))
        {
            return color_t(); // total internal reflection
        }

        Float cos_theta_a = wo.dot(wo_normal);
        Float cos_theta_b = (*out_wi).dot(normal);
        Float cos_theta_i = into ? cos_theta_a : cos_theta_b;

        Float Re = fresnel_dielectric_schlick(cos_theta_a, etaA_, etaB_);
        Float Tr = 1 - Re;

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
    color_t R_;
    color_t T_;
    Float etaA_;
    Float etaB_;
};


class phong_reflection_t : public bsdf_t
{
public:
    phong_reflection_t(const frame_t& shading_frame, const color_t& R) :
        bsdf_t(shading_frame), R_{ R }
    {
    }

    bool is_delta() const override { return false; }

    color_t f_(const vec3_t& wo, const vec3_t& wi) const override
    {
    }

    Float pdf_(const vec3_t& wo, const vec3_t& wi) const override { return 0; }

    bsdf_sample_t sample_f_(const vec3_t& wo, const point2_t& random) const override
    {
        bsdf_sample_t sample;


        return sample;
    }

private:
    color_t R_;
};

#pragma endregion

// TODO
// class texture_t

#pragma region material

class material_t
{
public:
    virtual ~material_t() = default;

    virtual bsdf_uptr_t scattering(const isect_t& isect) const = 0;
};

using material_sp = std::shared_ptr<material_t>;
using material_list_t = std::vector<material_sp>;

class matte_material_t : public material_t
{
public:
    matte_material_t(const color_t Kd) :
        Kd_{ Kd }
    {
    }

    bsdf_uptr_t scattering(const isect_t& isect) const override
    {
        return std::make_unique<lambertion_reflection_t>(frame_t(isect.normal), Kd_);
    }

private:
    color_t Kd_;
};

class mirror_material_t : public material_t
{
public:
    mirror_material_t(const color_t Kr) :
        Kr_{ Kr }
    {
    }

    bsdf_uptr_t scattering(const isect_t& isect) const override
    {
        return std::make_unique<specular_reflection_t>(frame_t(isect.normal), Kr_);
    }

private:
    color_t Kr_;
};

class glass_material_t : public material_t
{
public:
    glass_material_t(const color_t& Kr, const color_t& Kt, Float eta) :
        Kr_{ Kr }, Kt_{ Kt }, eta_{ eta }
    {
    }

    bsdf_uptr_t scattering(const isect_t& isect) const override
    {
        return std::make_unique<fresnel_specular_t>(frame_t(isect.normal), Kr_, Kt_, 1, eta_);
    }

private:
    color_t Kr_;
    color_t Kt_;
    Float eta_;
};

// TODO: Normalizing Bling-Phong BRDF

#pragma endregion



#pragma region light

enum class light_flag : int
{
    delta_position = 1,
    delta_direction = 2,

    area = 4,
    infinite = 8
};

inline bool is_delta_light(int flags)
{
    return flags & (int)light_flag::delta_position ||
           flags & (int)light_flag::delta_direction;
}


/*
  called `LightLeSample` in pbrt - v4, `PositionSample` in mitsuba2
  mainly for light.sample_Le()
*/
struct position_sample_t
{
    bool is_delta;
    point3_t position; // maybe zero
    normal_t normal; // maybe zero
    ray_t ray;
    Float pdf_position = 0;
    Float pdf_direction = 0;
    color_t Le; // for light

    position_sample_t() = default;
    position_sample_t(
        const point3_t& position, const normal_t& normal, const ray_t& ray,
        Float pdf_position, Float pdf_direction,
        const color_t& emission) :
        position{ position },
        normal{ normal },
        ray(ray),
        pdf_position(pdf_position),
        pdf_direction(pdf_direction),
        Le(emission)
    {
    }

    Float abs_cos_theta(vec3_t w) const { return is_delta ? 1 : abs_dot(w, normal); }
};

/*
  called `LightLiSample` in pbrt-v4, `DirectionSample` in mitsuba2
  mainly for light.sample_Li()
*/
struct direction_sample_t // : public position_sample_t
{
    point3_t position;
    vec3_t wi; // isect -> light, alone ray's direction
    Float pdf; // pdf of direction
    color_t Li; // for light

    direction_sample_t() = default;
    direction_sample_t(const isect_t& isect, const vec3_t& wi, Float pdf, const color_t& Li) :
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
    light_t(/*int flags,*/ const point3_t& world_position, int samples_num = 1):
        world_position_{ world_position },
        samples_num_{ samples_num }
    {
    }

public:
    // whether the light is delta distribution(point, directional), or not(area, environment)
    virtual bool is_delta() const = 0;
    // whether the light is finite extent (point, area) or not(directional, environment)
    virtual bool is_finite() const = 0;

    virtual void preprocess(const scene_t& scene);
    virtual color_t power() const = 0;

public:
    // Le: camera <- light

    // only for environment light
    virtual color_t Le(const ray_t& r) const { return color_t(); }

    // TODO: remove
    // for bidirectional method
    // sample_position, sample_ray
    virtual color_t sample_Le(
        const point2_t& random1, const point2_t& random2,
        ray_t* ray, normal_t* light_normal, 
        Float* pdf_position, Float* pdf_direction) const = 0;

    virtual void pdf_Le(
        const ray_t& ray, const normal_t& light_normal, 
        Float* pdf_position, Float* pdf_direction) const = 0;

public:
    // Li: camera <-wo isect wi-> light

    // sample_light/samle_direction
    virtual direction_sample_t sample_Li(const isect_t& isect, const point2_t& random) const = 0;

    virtual Float pdf_Li(const isect_t& isect, const vec3_t& world_wi) const = 0;

protected:
    const point3_t& world_position_;
    int samples_num_;
};

using light_sp = std::shared_ptr<light_t>;
using light_list_t = std::vector<light_sp>;

class point_light_t : public light_t
{
public:
    point_light_t(const point3_t& world_position, int samples_num, color_t intensity) :
        light_t(world_position, samples_num),
        intensity_{ intensity }
    {
    }

    bool is_delta() const override { return true; }
    bool is_finite() const override { return true; }

    color_t power() const override { return 4 * k_pi * intensity_; }

public:
    color_t sample_Le(
        const point2_t& random1, const point2_t& random2,
        ray_t* ray, normal_t* light_normal,
        Float* pdf_position, Float* pdf_direction) const override
    {
        *ray = ray_t(world_position_, sample_sphere_uniform(random1));
        *light_normal = ray->direction();

        *pdf_position = 1;
        *pdf_direction = pdf_sphere_uniform();

        return intensity_;
    }

    void pdf_Le(
        const ray_t& ray, const normal_t& light_normal,
        Float* pdf_position, Float* pdf_direction) const override
    {
        *pdf_position = 0;
        *pdf_direction = pdf_sphere_uniform();
    }

public:
    direction_sample_t sample_Li(const isect_t& isect, const point2_t& random) const override
    {
        direction_sample_t sample;
        sample.position = world_position_;
        sample.wi = normalize(world_position_ - isect.position);
        sample.pdf = 1.f;
        sample.Li = intensity_ / distance_squared(world_position_, isect.position); // TODO

        return sample;
    }

    Float pdf_Li(
        const isect_t& isect, const vec3_t& world_wi) const override
    {
        return 0;
    }

private:
    color_t intensity_;
};

// a disk
class direction_light_t : public light_t
{
public:
    direction_light_t(const point3_t& world_position, int samples_num, color_t radiance, const vec3_t world_direction) :
        light_t(world_position, samples_num),
        radiance_{ radiance },
        world_direction_{ world_direction },
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
    color_t sample_Le(
        const point2_t& random1, const point2_t& random2,
        ray_t* ray, normal_t* light_normal,
        Float* pdf_position, Float* pdf_direction) const override
    {
        point2_t sample_point = sample_disk_concentric(random1);
        point3_t disk_position = world_center_ + world_radius_ * (frame_.binormal() * sample_point.x + frame_.tangent() * sample_point.y);

        *ray = ray_t(disk_position + world_radius_ * world_direction_, -world_direction_);
        *light_normal = ray->direction();

        *pdf_position = 1 / area_;
        *pdf_direction = 1;

        return radiance_;
    }

    void pdf_Le(
        const ray_t& ray, const normal_t& light_normal,
        Float* pdf_position, Float* pdf_direction) const override
    {
        *pdf_position = 1 / area_;
        *pdf_direction = 0;
    }

public:
    direction_sample_t sample_Li(const isect_t& isect, const point2_t& random) const override
    {
        direction_sample_t sample;
        sample.position = world_position_;
        sample.wi = -world_direction_;
        sample.pdf = 1;
        sample.Li = radiance_;

        return sample;
    }

    Float pdf_Li(
        const isect_t& isect, const vec3_t& world_wi) const override
    {
        return 0;
    }

private:
    color_t radiance_;

    point3_t world_center_;
    Float world_radius_;
    Float area_;
    color_t power_;

    vec3_t world_direction_;
    frame_t frame_;
};

class area_light_t : public light_t
{
public:
    area_light_t(const point3_t& world_position, int samples_num, color_t radiance, const shape_t* shape):
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
       prev
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
    color_t Le(const isect_t& isect, const vec3_t& wo) const
    {
        return (dot(isect.normal, wo) > 0) ? radiance_ : color_t();
    }

    color_t sample_Le(
        const point2_t& random1, const point2_t& random2,
        ray_t* ray, normal_t* light_normal,
        Float* pdf_position, Float* pdf_direction) const override
    {
        // sample_position a point on the area light's _Shape_, _pShape_
        isect_t light_isect = shape_->sample_position(random1, pdf_position);
        *light_normal = light_isect.normal;

        // sample_position a cosine-weighted outgoing direction _w_ for area light
        vec3_t w;
        w = sample_hemisphere_cosine(random2);
        *pdf_direction = pdf_hemisphere_cosine(w.z);

        // TODO
        frame_t frame{ light_isect.normal };
        vec3_t w2 = w.x * frame.binormal() + w.y * frame.tangent() + w.z * frame.normal();
        *ray = light_isect.spawn_ray(w2);

        return Le(light_isect, w2);
    }

    void pdf_Le(
        const ray_t& ray, const normal_t& light_normal,
        Float* pdf_position, Float* pdf_direction) const override
    {
        isect_t isect(ray.origin(), light_normal, vec3_t());
        *pdf_position = shape_->pdf_position(isect);

        *pdf_direction = pdf_hemisphere_cosine(dot(light_normal, ray.direction()));
    }

public:
    // sample direction by sample potision
    direction_sample_t sample_Li(const isect_t& isect, const point2_t& random) const override
    {
        direction_sample_t sample;
        isect_t light_isect = shape_->sample_direction(isect, random, &sample.pdf);
        sample.position = light_isect.position;

        if (sample.pdf == 0 || (light_isect.position - isect.position).magnitude_squared() == 0)
        {
            sample.Li = color_t();
        }
        else
        {
            sample.wi = normalize(light_isect.position - isect.position);
            sample.Li = Le(light_isect, -sample.wi);
        }

        return sample;
    }

    Float pdf_Li(
        const isect_t& isect, const vec3_t& world_wi) const override
    {
        return shape_->pdf_direction(isect, world_wi);
    }

private:
    color_t radiance_;
    color_t power_;
    const shape_t* shape_;
};

// constant_environment_light_t
// a sphere hold all the scene
class environment_light_t : public light_t
{
public:
    environment_light_t(const point3_t& world_position, int samples_num, color_t radiance) :
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

    color_t sample_Le(
        const point2_t& random1, const point2_t& random2,
        ray_t* ray, normal_t* light_normal,
        Float* pdf_position, Float* pdf_direction) const override
    {
        *light_normal = sample_sphere_uniform(random1);

        frame_t frame{ *light_normal };
        point2_t sample_point = sample_disk_concentric(random2);
        point3_t disk_position = world_center_ + 
            world_radius_ * (frame.binormal() * sample_point.x + frame.tangent() * sample_point.y);
        *ray = ray_t(disk_position + world_radius_ * -*light_normal, *light_normal);

        Float theta = spherical_theta(*light_normal);
        Float sin_theta = std::sin(theta);
        *pdf_direction = sin_theta == 0 ? 0 : 1 / (2 * k_pi * k_pi * sin_theta);
        *pdf_position = 1 / (k_pi * world_radius_ * world_radius_);

        return radiance_;
    }

    void pdf_Le(
        const ray_t& ray, const normal_t& light_normal,
        Float* pdf_position, Float* pdf_direction) const override
    {
        Float theta = spherical_theta(ray.direction());
        *pdf_direction = 1 / (2 * k_pi * k_pi * std::sin(theta));
        *pdf_position = 1 / area_;
    }

public:
    direction_sample_t sample_Li(const isect_t& isect, const point2_t& random) const override
    {
        direction_sample_t sample;
        sample.wi = sample_sphere_uniform(random);
        sample.position = isect.position + sample.wi * 2 * world_radius_;

        Float theta = spherical_theta(sample.wi);
        Float sin_theta = std::sin(theta);
        sample.pdf = 1 / (2 * k_pi * k_pi * sin_theta);
        if (sin_theta == 0)
            sample.pdf = 0;

        sample.Li = radiance_;

        return sample;
    }

    Float pdf_Li(
        const isect_t& isect, const vec3_t& world_wi) const override
    {
        Float theta = spherical_theta(world_wi);
        Float sin_theta = std::sin(theta);

        if (sin_theta == 0)
            return 0;

        return 1 / (2 * k_pi * k_pi * sin_theta);
    }

private:
    color_t radiance_;

    point3_t world_center_;
    Float world_radius_;
    Float area_;
    color_t power_;
};

#pragma endregion



#pragma region surface(primitive)

struct surface_t
{
    const shape_t* shape;
    const material_t* material;
    const area_light_t* area_light;

    bool intersect(const ray_t& ray, isect_t* isect)
    {
        bool hit = shape->intersect(ray, isect);
        if (hit)
        {
            isect->surface_ = this;
            isect->bsdf_ = material->scattering(*isect);
            isect->emission_ = area_light ? area_light->Le(*isect, isect->wo) : color_t();
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
    light_directional = 2,
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

constexpr cornell_box_enum_t operator&(cornell_box_enum_t a, cornell_box_enum_t b) { return (cornell_box_enum_t)((int)a & (int)b); }
constexpr cornell_box_enum_t operator|(cornell_box_enum_t a, cornell_box_enum_t b) { return (cornell_box_enum_t)((int)a | (int)b); }

class scene_t : public reference_type_t
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
		// TODO
        for (auto& light : light_list_)
        {
            light->preprocess(*this);
        }
		
		// TODO: environment light
    }

public:
    // TODO
    bool intersect(const ray_t& ray, isect_t* isect)
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

    // TODO
    bool occluded(
        const vec3_t& point,
        const vec3_t& dir,
        float distance) //const
    {
        ray_t ray(point + dir * k_epsilon, dir, distance - 2 * k_epsilon);
        isect_t unused;
        return intersect(ray, &unused);
    }
    bool occluded(const isect_t& isect1, const point3_t& isect2) //const
    {
        auto ray = isect1.spawn_ray_to(isect2);
        ray.set_distance(
            distance(isect1.position, isect2) - 2 * k_epsilon);
        isect_t unused;
        return intersect(ray, &unused);
    }
    bool occluded(const isect_t& isect1, const isect_t& isect2) //const
    {
        auto ray = isect1.spawn_ray_to(isect2);
        ray.set_distance(
            distance(isect1.position, isect2.position) - 2 * k_epsilon);
        isect_t unused;
        return intersect(ray, &unused);
    }


    bounds3_t world_bound() const
    {
        bounds3_t bounds3;
        
        for (const auto& surface : surface_list_)
        {
            bounds3 = bounds3.join(surface.shape->world_bound());
        }

        return bounds3;
    }

public:
    const camera_t* camera() const { return camera_.get(); }

    int light_count() const { return light_list_.size(); }
    const light_list_t& light_list() const
    {
        return light_list_;
    }

    const environment_light_t* environment_light() const { return environment_light_; }

public:
    /*
    static scene_t create_smallpt_scene()
    {
        bool is_double = std::is_same_v<Float, double>;
        KY_CHECK(is_double);

        shape_sp left   = std::make_shared<sphere_t>(1e5, vec3_t(1e5 + 1,   40.8,        81.6));
        shape_sp right  = std::make_shared<sphere_t>(1e5, vec3_t(-1e5 + 99, 40.8,        81.6));
        shape_sp back   = std::make_shared<sphere_t>(1e5, vec3_t(50,        40.8,        1e5));
        shape_sp front  = std::make_shared<sphere_t>(1e5, vec3_t(50,        40.8,        -1e5 + 170));
        shape_sp bottom = std::make_shared<sphere_t>(1e5, vec3_t(50,        1e5,         81.6));
        shape_sp top    = std::make_shared<sphere_t>(1e5, vec3_t(50,        -1e5 + 81.6, 81.6));

        shape_sp mirror = std::make_shared<sphere_t>(16.5, vec3_t(27, 16.5,        47));
        shape_sp glass  = std::make_shared<sphere_t>(16.5, vec3_t(73, 16.5,        78));
        shape_sp light  = std::make_shared<sphere_t>(600,  vec3_t(50, 681.6 - .27, 81.6));
        shape_list_t shape_list{ left, right, back, front, bottom, top, mirror, glass, light };


        material_sp red   = std::make_shared<matte_material_t>(color_t(.75, .25, .25));
        material_sp blue  = std::make_shared<matte_material_t>(color_t(.25, .25, .75));
        material_sp gray  = std::make_shared<matte_material_t>(color_t(.75, .75, .75));
        material_sp black = std::make_shared<matte_material_t>(color_t());

        material_sp mirror_mat = std::make_shared<mirror_material_t>(color_t(1, 1, 1) * 0.999);
        material_sp glass_mat  = std::make_shared<glass_material_t>(color_t(1, 1, 1) * 0.999, color_t(1, 1, 1) * 0.999, 1.5);
        material_list_t material_list{ red, blue, gray, black, mirror_mat, glass_mat };

        // TODO
        std::shared_ptr<area_light_t> area_light = std::make_shared<area_light_t>(vec3_t(), 1, color_t(12, 12, 12), light.get());
        light_list_t light_list{ area_light };


        surface_list_t surface_list
        {
            {   left.get(),   red.get(), nullptr},
            {  right.get(),  blue.get(), nullptr },
            {   back.get(),  gray.get(), nullptr },
            {  front.get(), black.get(), nullptr },
            { bottom.get(),  gray.get(), nullptr },
            {    top.get(),  gray.get(), nullptr },

            { mirror.get(), mirror_mat.get(), nullptr },
            {  glass.get(),  glass_mat.get(), nullptr },

            {  light.get(), black.get(), area_light.get()},
        };

        return scene_t{ shape_list, material_list, light_list, surface_list};
    }
    */

    static scene_t create_cornell_box_scene(cornell_box_enum_t scene_enum, point2_t film_resolution)
    {
        if ((scene_enum & cornell_box_enum_t::both_large_spheres) == cornell_box_enum_t::both_large_spheres)
        {
            LOG_ERROR("Cannot have both large balls, using mirror\n\n");
        }

        const_camera_sptr_t camera = std::make_unique<camera_t>(
                vec3_t{ -0.0439815f, -4.12529f,  0.222539f },
                vec3_t{ 0.00688625f, 0.998505f, -0.0542161f },
                vec3_t{ 3.73896e-4f, 0.0542148f, 0.998529f },
                80, film_resolution);

        material_sp black = std::make_shared<matte_material_t>(color_t());
        material_sp white = std::make_shared<matte_material_t>(color_t(.8, .8, .8));
        material_sp red   = std::make_shared<matte_material_t>(color_t(0.803922f, 0.152941f, 0.152941f));
        material_sp green = std::make_shared<matte_material_t>(color_t(0.156863f, 0.803922f, 0.172549f));
        material_sp blue  = std::make_shared<matte_material_t>(color_t(0.156863f, 0.172549f, 0.803922f));

        material_sp mirror_mat = std::make_shared<mirror_material_t>(color_t(1, 1, 1));
        material_sp glass_mat = std::make_shared<glass_material_t>(color_t(1, 1, 1), color_t(1, 1, 1), 1.6);
        material_list_t material_list{ black, white, red, green, blue, mirror_mat, glass_mat };

        #pragma region shape

        // cornell box
        vec3_t cb[8] = 
        {
            vec3_t(-1.27029f,  1.30455f, -1.28002f),
            vec3_t( 1.28975f,  1.30455f, -1.28002f),
            vec3_t( 1.28975f,  1.30455f,  1.28002f),
            vec3_t(-1.27029f,  1.30455f,  1.28002f),
            vec3_t(-1.27029f, -1.25549f, -1.28002f),
            vec3_t( 1.28975f, -1.25549f, -1.28002f),
            vec3_t( 1.28975f, -1.25549f,  1.28002f),
            vec3_t(-1.27029f, -1.25549f,  1.28002f)
        };
        shape_sp left   = std::make_shared<rectangle_t>(cb[3], cb[0], cb[4], cb[7]);
        shape_sp right  = std::make_shared<rectangle_t>(cb[1], cb[2], cb[6], cb[5]);
        shape_sp back   = std::make_shared<rectangle_t>(cb[0], cb[3], cb[2], cb[1]);
        shape_sp bottom = std::make_shared<rectangle_t>(cb[0], cb[1], cb[5], cb[4]);
        shape_sp top    = std::make_shared<rectangle_t>(cb[2], cb[3], cb[7], cb[6]);


        // large ball
        Float large_radius = 0.8f;
        vec3_t large_center = (cb[0] + cb[4] + cb[5] + cb[1]) * (1.f / 4.f) + vec3_t(0, 0, large_radius);

        // small ball
        Float small_radius = 0.5f;
        vec3_t left_wall_center  = (cb[0] + cb[4]) * (1.f / 2.f) + vec3_t(0, 0, small_radius);
        vec3_t right_wall_center = (cb[1] + cb[5]) * (1.f / 2.f) + vec3_t(0, 0, small_radius);

        Float length_x = right_wall_center.x - left_wall_center.x;
        vec3_t left_center  = left_wall_center  + vec3_t(2.f * length_x / 7.f, 0, 0);
        vec3_t right_center = right_wall_center - vec3_t(2.f * length_x / 7.f, 0, 0);

        shape_sp large_ball  = std::make_shared<sphere_t>(large_center, large_radius);
        shape_sp left_ball   = std::make_shared<sphere_t>(left_center, small_radius);
        shape_sp right_ball  = std::make_shared<sphere_t>(right_center, small_radius);


        // small light box at the ceiling
        vec3_t lb[8] = 
        {
            vec3_t(-0.25f,  0.25f, 1.26002f),
            vec3_t( 0.25f,  0.25f, 1.26002f),
            vec3_t( 0.25f,  0.25f, 1.28002f),
            vec3_t(-0.25f,  0.25f, 1.28002f),
            vec3_t(-0.25f, -0.25f, 1.26002f),
            vec3_t( 0.25f, -0.25f, 1.26002f),
            vec3_t( 0.25f, -0.25f, 1.28002f),
            vec3_t(-0.25f, -0.25f, 1.28002f)
        };
        shape_sp left2   = std::make_shared<rectangle_t>(lb[3], lb[0], lb[4], lb[7]);
        shape_sp right2  = std::make_shared<rectangle_t>(lb[1], lb[2], lb[6], lb[5]);
        shape_sp front2  = std::make_shared<rectangle_t>(lb[4], lb[5], lb[6], lb[7]);
        shape_sp back2   = std::make_shared<rectangle_t>(lb[0], lb[3], lb[2], lb[1]);
        shape_sp bottom2 = std::make_shared<rectangle_t>(lb[0], lb[1], lb[5], lb[4]);

        shape_list_t shape_list
        { 
            left, right, back, bottom, top, 
            large_ball, left_ball, right_ball,
            left2, right2, front2, back2, bottom2, 
        };

        #pragma endregion


        #pragma region light

        bool light_area = (scene_enum & cornell_box_enum_t::light_area) != cornell_box_enum_t::none;
        bool light_directional = (scene_enum & cornell_box_enum_t::light_directional) != cornell_box_enum_t::none;
        bool light_point = (scene_enum & cornell_box_enum_t::light_point) != cornell_box_enum_t::none;
        bool light_environment = (scene_enum & cornell_box_enum_t::light_environment) != cornell_box_enum_t::none;

        light_list_t light_list{};
        if (light_area)
        {
            light_list.push_back(
                std::make_shared<area_light_t>(point3_t(), 1, color_t(25, 25, 25), bottom2.get()));
        }

        if (light_directional)
        {
            light_list.push_back(
                std::make_shared<direction_light_t>(point3_t(-5., 5., -5.), 1, color_t(1, 1, 1), vec3_t(1, -1.5, 1)));
        }

        if (light_point)
        {
            Float I = 70 * k_inv_4pi;
            light_list.push_back(
                std::make_shared<point_light_t>(point3_t(0.0, -0.5, 1.0), 1, color_t(I, I, I)));
        }

        environment_light_t* environment_light{};
        if (light_environment)
        {
            color_t L = vec3_t(135. / 255, 206. / 255, 250. / 255);
            auto light = std::make_shared<environment_light_t>(point3_t(), 1, L);
            light_list.push_back(light);

            environment_light = light.get();
        }

        #pragma endregion


        #pragma region surface

        surface_list_t surface_list
        {
            {   left.get(),    red.get(), nullptr},
            {  right.get(),   blue.get(), nullptr },
            {    top.get(),  white.get(), nullptr },
            { bottom.get(),  white.get(), nullptr },
            {   back.get(),  green.get(), nullptr },
        };

        if ((scene_enum & cornell_box_enum_t::glossy_floor) != cornell_box_enum_t::none)
        {
            surface_list[3].material = blue.get();
            surface_list[4].material = white.get(); // TODO
        }

        if ((scene_enum & cornell_box_enum_t::large_mirror_sphere) != cornell_box_enum_t::none)
            surface_list.push_back({ large_ball.get(), mirror_mat.get(), nullptr });
        else if ((scene_enum & cornell_box_enum_t::large_glass_sphere) != cornell_box_enum_t::none)
            surface_list.push_back({ large_ball.get(), glass_mat.get(), nullptr });

        if ((scene_enum & cornell_box_enum_t::small_mirror_sphere) != cornell_box_enum_t::none)
            surface_list.push_back({ left_ball.get(), mirror_mat.get(), nullptr });
        if ((scene_enum & cornell_box_enum_t::small_glass_sphere) != cornell_box_enum_t::none)
            surface_list.push_back({ right_ball.get(), glass_mat.get(), nullptr });

        if (light_area)
        {
            surface_list.push_back({   left2.get(),   red.get(), nullptr });
            surface_list.push_back({  right2.get(),  blue.get(), nullptr });
            surface_list.push_back({  front2.get(), white.get(), nullptr });
            surface_list.push_back({   back2.get(), white.get(), nullptr });
            surface_list.push_back({ bottom2.get(), white.get(), (area_light_t*)light_list[0].get() });
        }

        #pragma endregion


        return scene_t{ camera, shape_list, material_list, light_list, surface_list, environment_light };
    }

    static scene_t create_mis_scene(point2_t film_resolution)
    {
        const_camera_sptr_t camera = std::make_unique<camera_t>(
            vec3_t{ 0, 5, -5 },
            vec3_t{ 0, 0, 1 },
            vec3_t{ 0, 1, 0 },
            105, film_resolution);

        material_sp black = std::make_shared<matte_material_t>(color_t());
        material_sp gray = std::make_shared<matte_material_t>(color_t(.2, .2, .2));
        material_sp white = std::make_shared<matte_material_t>(color_t(.9, .9, .9));

        material_sp mirror_mat = std::make_shared<mirror_material_t>(color_t(1, 1, 1));
        material_sp glass_mat = std::make_shared<glass_material_t>(color_t(1, 1, 1), color_t(1, 1, 1), 1.6);
        material_list_t material_list{ black, gray, white, mirror_mat, glass_mat };

#pragma region shape

        shape_sp bottom = std::make_shared<rectangle_t>(
            point3_t(-50, 0, -50), point3_t(-50, 0, 50), point3_t(50, 0, 50), point3_t(50, 0, -50));
        shape_sp back = std::make_shared<rectangle_t>(
            point3_t(-50, -50, 20), point3_t(-50, 50, 20), point3_t(50, 50, 20), point3_t(50, -50, 20));

        shape_sp plank0 = std::make_shared<rectangle_t>(
            point3_t(-5, 0.1, 5), point3_t(-5, 0.3, 6), point3_t(5, 0.3, 6), point3_t(5, 0.1, 5));
        shape_sp plank1 = std::make_shared<rectangle_t>(
            point3_t(-5, 0.4, 7), point3_t(-5, 0.8, 8), point3_t(5, 0.8, 8), point3_t(5, 0.4, 7));
        shape_sp plank2 = std::make_shared<rectangle_t>(
            point3_t(-5, 1.1, 9), point3_t(-5, 1.9, 10), point3_t(5, 1.9, 10), point3_t(5, 1.1, 9));
        shape_sp plank3 = std::make_shared<rectangle_t>(
            point3_t(-5, 2.2, 11), point3_t(-5, 4, 12), point3_t(5, 4, 12), point3_t(5, 2.2, 11));

        shape_sp ball0 = std::make_shared<sphere_t>(point3_t(-4, 6, 8), 0.1);
        shape_sp ball1 = std::make_shared<sphere_t>(point3_t(-2, 6, 8), 0.4);
        shape_sp ball2 = std::make_shared<sphere_t>(point3_t(0.2, 6, 8), 0.9);
        shape_sp ball3 = std::make_shared<sphere_t>(point3_t( 4, 6, 8), 1.6);

        shape_list_t shape_list
        {
            bottom, back,
            plank0, plank1, plank2, plank3,
            ball0, ball1, ball2, ball3
        };

#pragma endregion

        auto light0 = std::make_shared<area_light_t>(point3_t(), 1, color_t(25, 25, 25), ball0.get());
        auto light1 = std::make_shared<area_light_t>(point3_t(), 1, color_t(25, 25, 25), ball1.get());
        auto light2 = std::make_shared<area_light_t>(point3_t(), 1, color_t(25, 25, 25), ball2.get());
        auto light3 = std::make_shared<area_light_t>(point3_t(), 1, color_t(25, 25, 25), ball3.get());

        light_list_t light_list
        {
            light0, light1, light2, light3
        };

        surface_list_t surface_list
        {
            { bottom.get(), gray.get(), nullptr},
            {   back.get(), gray.get(), nullptr},

            { plank0.get(), white.get(), nullptr},
            { plank1.get(), white.get(), nullptr},
            { plank2.get(), white.get(), nullptr},
            { plank3.get(), white.get(), nullptr},

            { ball0.get(),  black.get(), light0.get() },
            { ball1.get(),  black.get(), light1.get() },
            { ball2.get(),  black.get(), light2.get() },
            { ball3.get(),  black.get(), light3.get() },
        };

        return scene_t{ camera, shape_list, material_list, light_list, surface_list, nullptr };
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
    //TODO
    /*
    auto world_bound = scene.world_bound();

    world_bound.bounding_sphere(&world_center_, &world_radius_);

    area_ = k_pi * world_radius_ * world_radius_;
    power_ = radiance_ * area_;
    */
}

void environment_light_t::preprocess(const scene_t& scene)
{
    // TODO
    world_radius_ = 1000;
    /*
    auto world_bound = scene.world_bound();

    world_bound.bounding_sphere(&world_center_, &world_radius_);

    area_ = k_pi * world_radius_ * world_radius_;
    power_ = radiance_ * area_;
    */
}

#pragma endregion



#pragma region integrater

/* 
  Li = Lo = Le + ∫Li
          = Le + ∫(Le + ∫Li)
          = Le + ∫Le + ∫∫(Le + ∫Li)) = ...
*/
enum class lighting_enum_t
{
    emit, // Le = Le
    direct, // Ld = ∫Le
    indirect, // Li = ∫∫(Le + ∫Li)
};

// direct_lighting_sample
enum class direct_sample_enum_t
{
    sample_single_light = 1,
    sample_all_light = 2,

    bsdf = 4, // direction
    light = 8, // position

    bsdf_mis = 16, // for debug
    light_mis = 32, // for debug
    both_mis = bsdf_mis | light_mis,

    default_stragtgy = sample_all_light | both_mis
};

enum class integrater_enum_t
{
    // debug
    depth,
    normal,

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
    simple_path_tracing_iterasion,

    // Le + T * Le + T(T * Le + T(...))
    path_tracing_recursion,
    path_tracing_iteration,
};

// TODO: remove
struct light_sample_t
{
    light_t* light{};
    Float pdf_light{};
};

struct scene_sample_t
{
    scene_t* scene{};
    sampler_t* sampler{};
    ray_t ray;
    int depth;
    bool is_last_specular;
    isect_t isect;
};

/*
  rendering scene by Rendering Equation(Li = Lo = Le + ∫Li)
  solving Rendering Equation(a integral equation) by numerical integration(monte carlo integration)
*/
class integrater_t
{
public:
    ~integrater_t() = default;
    integrater_t()
    {
    }

public:
    void render(/*const*/ scene_t* scene, sampler_t* original_sampler, film_t* film)
    {
        auto resolution = film->get_resolution();
        int width = (int)resolution.x;
        int height = (int)resolution.y;

    #ifndef KY_DEBUG
        #pragma omp parallel for schedule(dynamic, 1) // OpenMP
    #endif // !KY_DEBUG
        for (int y = 0; y < height; y += 1)
        {
            auto sampler = original_sampler->clone(); // multi thread
            auto camera = scene->camera();
            LOG("\rrendering... ({} spp) {:.2f}%", sampler->ge_samples_per_pixel(), 100. * y / (height - 1));

            for (int x = 0; x < width; x += 1)
            {
                color_t L{};
                sampler->start_sample();
                //film_->set_color(x, y, color_t(0, 0, 0));

                do
                {
                    auto sample = sampler->get_camera_sample({ (Float)x, (Float)y });
                    auto ray = camera->generate_ray(sample);

                    L = L + Li(ray, scene, sampler.get()) * (1. / sampler->ge_samples_per_pixel());
                }
                while (sampler->next_sample());

                // TODO
                film->add_color(x, y, clamp01(L));
            }
        }
    }


    // render_phase    
    void debug()
    {

    }

    void debug(scene_t* scene, sampler_t* original_sampler, film_t* film, const point2_t& film_position)
    {
        color_t L{};
        auto camera = scene->camera();
        auto sampler = original_sampler->clone(); // multi thread
        sampler->start_sample();
        //film_->set_color(x, y, color_t(0, 0, 0));

        do
        {
            auto sample = sampler->get_camera_sample(film_position);
            auto ray = camera->generate_ray(sample);

            auto dL = Li(ray, scene, sampler.get()) * (1. / sampler->ge_samples_per_pixel());
            LOG("dL:{}\n", dL.to_string());
            L = L + dL;
        }
        while (sampler->next_sample());
    }

    // estimate input radiance
    // TODO: virtual color_t Li(ray_t ray, const scene_t& scene, sampler_t& sampler) = 0;
    virtual color_t Li(ray_t ray, scene_t* scene, sampler_t* sampler) = 0;

protected:
    light_sample_t pick_single_light(
        const isect_t& isect, scene_t* scene, sampler_t& sampler)
    {
        int light_count = int(scene->light_count());
        if (light_count == 0)
            return {};

        // TODO: power based, spatial based
        int light_index = std::min((int)(sampler.get_float() * light_count), light_count - 1);
        auto light = scene->light_list()[light_index].get();
        Float pdf_light = Float(1) / light_count;

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
        Float pdf_light = Float(1) / light_count;

        auto light = scene->light_list()[light_index].get();
        point2_t uLight = sampler.get_vec2();
        point2_t uScattering = sampler.get_vec2();

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
        case direct_sample_enum_t::bsdf:
            estimate_direct_lighting = estimate_direct_lighting_direction;
            break;
        case direct_sample_enum_t::light:
            estimate_direct_lighting = estimate_direct_lighting_position;
            break;
        case direct_sample_enum_t::bsdf_mis:
            estimate_direct_lighting = estimate_direct_lighting_direction_mis;
            break;
        case direct_sample_enum_t::light_mis:
            estimate_direct_lighting = estimate_direct_lighting_direction_mis;
            break;
        case direct_sample_enum_t::both_mis:
            estimate_direct_lighting = estimate_direct_lighting_both_mis;
            break;
        default:
            break;
        }

        for (const auto& light : scene->light_list())
        {
            Ld += estimate_direct_lighting(
                isect, *light, sampler.get_vec2(), sampler.get_vec2(),
                scene, sampler, skip_specular);
        }

        return Ld;
    }

protected:
    // sample direct lighting

    // sample from bsdf/direction
    static color_t estimate_direct_lighting_direction(
        const isect_t& isect, const light_t& light,
        const point2_t& random_light, const point2_t& random_bsdf,
        scene_t* scene, sampler_t& sampler, bool skip_specular)
    {
        color_t Ld{};

        if (light.is_delta())
            return Ld;
        if (skip_specular && isect.bsdf()->is_delta())
            return Ld;

        auto bs = isect.bsdf()->sample_f(isect.wo, sampler.get_vec2());
        if (bs.f.is_black() || bs.pdf == 0)
            return Ld;

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
            // for environment light
            Li = light.Le(ray);
        }

        if (!Li.is_black())
            Ld = bs.f * Li * abs_dot(bs.wi, isect.normal) / bs.pdf;

        return Ld;
    }

    // sample from light/position
    static color_t estimate_direct_lighting_position(
        const isect_t& isect, const light_t& light,
        const point2_t& random_light, const point2_t& random_bsdf,
        scene_t* scene, sampler_t& sampler, bool skip_specular)
    {
        color_t Ld{};

        if (skip_specular && isect.bsdf()->is_delta())
            return Ld;

        auto ls = light.sample_Li(isect, random_light);
        if (ls.Li.is_black() || ls.pdf == 0)
            return Ld;

        if (scene->occluded(isect, ls.position))
            return Ld;

        color_t f = isect.bsdf()->f(isect.wo, ls.wi);
        if (!f.is_black())
        {
            Ld += f * ls.Li * abs_dot(ls.wi, isect.normal) / ls.pdf;
        }

        return Ld;
    }


    // sample direct lighting with MIS

    static color_t estimate_direct_lighting_direction_mis(
        const isect_t& isect, const light_t& light,
        const point2_t& random_light, const point2_t& random_bsdf,
        scene_t* scene, sampler_t& sampler, bool skip_specular)
    {
        color_t Ld{};

        bool sample_specular = isect.bsdf()->is_delta();
        if (skip_specular && sample_specular)
            return Ld;

        // Sample BSDF with multiple importance sampling
        if (!light.is_delta())
        {
            // Sample scattered direction for surface isect_ts
            auto bs = isect.bsdf()->sample_f(isect.wo, random_bsdf);
            bs.f *= abs_dot(bs.wi, isect.normal);

            if (!bs.f.is_black() && bs.pdf > 0)
            {
                Float weight = 1;
                if (!sample_specular)
                {
                    Float pdf_light = light.pdf_Li(isect, bs.wi);
                    if (pdf_light == 0)
                        return Ld;

                    weight = power_heuristic(1, bs.pdf, 1, pdf_light);
                }
                // else 
                //    weight = 1;

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
                    // for environment light
                    Li = light.Le(ray);
                }

                if (!Li.is_black())
                    Ld += bs.f * Li * weight / bs.pdf;
            }
        }

        return Ld;
    }

    static color_t estimate_direct_lighting_position_mis(
        const isect_t& isect, const light_t& light,
        const point2_t& random_light, const point2_t& random_bsdf,
        scene_t* scene, sampler_t& sampler, bool skip_specular)
    {
        color_t Ld{};

        if (skip_specular && isect.bsdf()->is_delta())
            return Ld;

        // Sample light source with multiple importance sampling
        auto ls = light.sample_Li(isect, random_light);
        if (!ls.Li.is_black() && ls.pdf > 0)
        {
            // Compute BSDF or phase function's value for light sample
            color_t f = isect.bsdf()->f(isect.wo, ls.wi) * abs_dot(ls.wi, isect.normal);
            Float pdf_bsdf = isect.bsdf()->pdf(isect.wo, ls.wi);

            if (!f.is_black())
            {
                if (scene->occluded(isect, ls.position))
                {
                    ls.Li = color_t();
                }

                // Add light's contribution to reflected radiance
                if (!ls.Li.is_black())
                {
                    if (light.is_delta())
                        Ld += f * ls.Li / ls.pdf;	// return f * Li / pdf_light;
                    else
                    {
                        Float weight = power_heuristic(1, ls.pdf, 1, pdf_bsdf);
                        Ld += f * ls.Li * weight / ls.pdf;
                    }
                }
            }
        }

        return Ld;
    }

    static color_t estimate_direct_lighting_both_mis(
        const isect_t& isect, const light_t& light,
        const point2_t& random_light, const point2_t& random_bsdf,
        scene_t* scene, sampler_t& sampler, bool skip_specular)
    {
        return
            estimate_direct_lighting_direction_mis(isect, light, random_light, random_bsdf, scene, sampler, skip_specular) +
            estimate_direct_lighting_position_mis(isect, light, random_light, random_bsdf, scene, sampler, skip_specular);
    }
};


/*
class direct_lighting_t : public integrater_t
{
public:
    direct_lighting_t()
    {
    }
};


class stochastic_raytracing_t : public integrater_t
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


class path_integrater_t : public integrater_t
{
public:
    path_integrater_t(int max_path_depth):
        max_path_depth_{ max_path_depth }
    {
    }

protected:
    int max_path_depth_;
};

class simple_path_tracing_recursion_t : public path_integrater_t
{
public:
    using path_integrater_t::path_integrater_t;

    color_t Li(ray_t ray, scene_t* scene, sampler_t* sampler) override
    {
        return Li(ray, scene, sampler, 0);
    }

    // Le + T(Le + T(Le + T(...)))
    color_t Li(ray_t ray, scene_t* scene, sampler_t* sampler, int depth)
    {
        isect_t isect;
        if (!scene->intersect(ray, &isect))
        {
            if (auto env_light = scene->environment_light(); env_light != nullptr)
                return env_light->Le(ray);
            else
                return color_t();
        }

        if (depth >= max_path_depth_)
            return isect.Le();


        auto bs = isect.bsdf()->sample_f(isect.wo, sampler->get_vec2());

        if (bs.f.is_black() || bs.pdf == 0.f) // pdf == 0 => NaN
            return isect.Le();

        //russian roulette
        if (++depth > 3)
        {
            Float bsdf_max_comp = bs.f.max_component_value();
            if (sampler->get_float() < bsdf_max_comp) // continue
                bs.f *= (1 / bsdf_max_comp);
            else
                return isect.Le();
        }

        /*
          auto beta = f * abs_dot(wi, isect.normal) / pdf;
          return beta * Li(wi_ray, scene, sampler, depth));
        */
        ray_t wi_ray(isect.position, bs.wi);
        color_t Ls = bs.f * Li(wi_ray, scene, sampler, depth) * abs_dot(bs.wi, isect.normal) / bs.pdf;

        // TODO: DCHECK
        return isect.Le() + Ls;
    }
};

/*
  Le + T*(Le + T*(le + ...))
  class path_tracing_recursion

  Le + T*Le + T^2*Le + ...
  class path_tracing_iteration
*/

class path_integrater_sample_t : public path_integrater_t
{
public:
    path_integrater_sample_t(int max_path_depth, direct_sample_enum_t sample_enum) :
        path_integrater_t(max_path_depth),
        sample_enum_{ sample_enum }
    {
    }

protected:
    /*
    color_t Ld_(ray_t ray, scene_t* scene, sampler_t* sampler, int depth, bool is_last_specular,
        const isect_t& isect)
    {
        color_t L;
        void* sample_light_func;
        void* estimate_func;

        switch (sample_enum_)
        {
        case direct_sample_enum_t::sample_single_light:
            sample_light_func = (void*)sample_single_light;
            break;
        case direct_sample_enum_t::sample_all_light:
            sample_light_func = (void*)sample_all_light;
            break;
        default:
            LOG_ERROR(" unspecified sample light method");
            break;
        }

        switch (sample_enum_)
        {
        case direct_sample_enum_t::bsdf:
            estimate_func = (void*)estimate_direct_lighting_direction;
            break;
        case direct_sample_enum_t::light:
            estimate_func = (void*)estimate_direct_lighting_position;
            break;
        case direct_sample_enum_t::both:
            estimate_func = (void*)estimate_direct_lighting_both;
            break;
        case direct_sample_enum_t::bsdf_mis:
            break;
        case direct_sample_enum_t::light_mis:
            break;
        case direct_sample_enum_t::both_mis:
            break;
        default:
            LOG_ERROR(" unspecified direct lighting method");
            break;
        }

        return L;
    }
    */

protected:
    direct_sample_enum_t sample_enum_;
};


class path_tracing_recursion_t : public path_integrater_sample_t
{
public:
    using path_integrater_sample_t::path_integrater_sample_t;

    color_t Li(ray_t ray, scene_t* scene, sampler_t* sampler) override
    {
        return Li(ray, scene, sampler, 0, false);
    }

private:
    /// <param name="is_last_specular">last vertex is specular</param>
    color_t Li(ray_t ray, scene_t* scene, sampler_t* sampler, int depth, bool is_last_specular)
    {
        isect_t isect;
        bool hit = scene->intersect(ray, &isect);

        color_t Le = emission_lighting(ray, scene, sampler, depth, is_last_specular, isect, hit);

        if (!hit || (depth >= max_path_depth_))
            return Le;
        else
        {
            color_t Ls = direct_lighting(ray, scene, sampler, depth, is_last_specular, isect)
                       + indirect_lighting(ray, scene, sampler, depth, is_last_specular, isect);

            return Le + Ls;
        }
    }
 
    color_t emission_lighting(ray_t ray, scene_t* scene, sampler_t* sampler, int depth, bool is_last_specular,
        const isect_t& isect, bool hit)
    {
        color_t Le;
        if (depth == 0 || is_last_specular)
        {
            if (hit)
            {
                Le = isect.Le();
            }
            else
            {
                if (auto env_light = scene->environment_light(); env_light != nullptr)
                    Le = env_light->Le(ray);
            }
        }

        return Le;
    }
 
    color_t direct_lighting(ray_t ray, scene_t* scene, sampler_t* sampler, int depth, bool is_last_specular,
        const isect_t& isect)
    {
        color_t Ld{};
        // skip specular bsdf
        if (!isect.bsdf()->is_delta())
        {
            Ld = sample_all_light(isect, scene, *sampler, true, sample_enum_);
        }
        return Ld;
    }

    color_t indirect_lighting(ray_t ray, scene_t* scene, sampler_t* sampler, int depth, bool is_last_specular,
        const isect_t& isect)
    {
        auto bs = isect.bsdf()->sample_f(isect.wo, sampler->get_vec2());

        if (bs.f.is_black() || bs.pdf == 0.f)
            return color_t();

        //russian roulette
        if (++depth > 3)
        {
            Float bsdf_max_comp = bs.f.max_component_value();
            if (sampler->get_float() < bsdf_max_comp) // continue
                bs.f *= (1 / bsdf_max_comp);
            else
                return isect.Le();
        }

        ray_t wi_ray(isect.position, bs.wi);
        return bs.f * Li(wi_ray, scene, sampler, depth, isect.bsdf()->is_delta()) * abs_dot(bs.wi, isect.normal) / bs.pdf;
    }
};

class path_tracing_iteration_t : public path_integrater_sample_t
{
public:
    using path_integrater_sample_t::path_integrater_sample_t;

    // Le + T * Le + T(T * Le + T(T * Le + T(...)))
    // Le(emit), Ld(direct), Li(indirect)
    color_t Li(ray_t ray, scene_t* scene, sampler_t* sampler) override
    {
        color_t L{};
        color_t beta(1., 1., 1.); // beta holds path throughput weight
        bool is_last_specular = false; // if last vertex's material has perfect specular property

        for (int bounces = 0; ; ++bounces)
        {
            // Find next path vertex and accumulate contribution
            // Intersect _ray_ with scene and store intersection in _isect_
            isect_t isect;
            bool hit = scene->intersect(ray, &isect);


            // Le: Li

            // Possibly add emitted light at intersection
            if (bounces == 0 || is_last_specular)
            {
                // Add emitted light at path vertex or from the environment
                if (hit)
                {
                    L += beta * isect.Le();
                }
                else
                {
                    if (auto env_light = scene->environment_light(); env_light != nullptr)
                        L += beta * env_light->Le(ray);
                }
            }


            // Terminate path if ray escaped or _maxDepth_ was reached
            if (!hit || bounces >= max_path_depth_)
                break;


            // Ld: direct lighting

            // Sample illumination from lights to find path contribution.
            // (But skip this for perfectly specular BSDFs.)
            if (!isect.bsdf()->is_delta())
            {
                color_t Ld = beta * sample_all_light(isect, scene, *sampler, true, sample_enum_);
                L += Ld;
            }


            // Li: indirect lighting (compute by next iteration)

            // Sample BSDF to get new path direction
            auto bs = isect.bsdf()->sample_f(isect.wo, sampler->get_vec2());

            if (bs.f.is_black() || bs.pdf == 0.f)
                break;

            // update path throughout
            beta *= bs.f * abs_dot(bs.wi, isect.normal) / bs.pdf;
            DCHECK(beta.luminance() > 0.f);
            DCHECK(!std::isinf(beta.luminance()));

            // TODO
            is_last_specular = is_delta_bsdf(bs.bsdf_type);
            ray = isect.spawn_ray(bs.wi); 

            // Possibly terminate the path with Russian roulette.
            // Factor out radiance scaling due to refraction in rrBeta.
            if (bounces > 3)
            {
                Float beta_max_comp = beta.max_component_value();
                Float q = std::max((Float).05, 1 - beta_max_comp);

                if (sampler->get_float() < q)
                    break;
                else
                {
                    beta *= 1 / (1 - q);
                    DCHECK(!std::isinf(beta.luminance()));
                }
            }
        }

        return L;
    }
};

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


void render_single_scene(int argc, char* argv[])
{
    int width = 256, height = 256;
    int samples_per_pixel = argc == 2 ? atoi(argv[1]) / 4 : 1; // # samples per pixel

    film_t film(width, height); //film.clear(color_t(1., 0., 0.));
    std::unique_ptr<sampler_t> sampler =
        std::make_unique<random_sampler_t>(samples_per_pixel);
    std::unique_ptr<integrater_t> integrater =
        std::make_unique<path_tracing_iteration_t>(10, direct_sample_enum_t::bsdf);

    //scene_t scene = scene_t::create_mis_scene(film.get_resolution());
    //scene_t scene = scene_t::create_cornell_box_scene(cornell_box_enum_t::default_scene);
    scene_t scene = scene_t::create_cornell_box_scene(
        cornell_box_enum_t::both_small_spheres | cornell_box_enum_t::light_point, film.get_resolution());

    integrater->render(&scene, sampler.get(), &film);
    //integrater->debug(&scene, { 160, 150 });

    film.store_image("single.bmp"s);
#ifdef KY_WINDOWS
    system("mspaint single.bmp");
#endif
}

void render_multiple_scene(int argc, char* argv[])
{
    auto scene_params = std::vector<std::pair<cornell_box_enum_t, int>>
    {
        { cornell_box_enum_t::light_point, 10 },
        { cornell_box_enum_t::light_directional, 1 },
        { cornell_box_enum_t::light_area, 1 },
        { cornell_box_enum_t::light_environment, 1 },
    };

    auto sample_enums = std::vector<direct_sample_enum_t>
    {
        direct_sample_enum_t::bsdf,
        direct_sample_enum_t::light,
        direct_sample_enum_t::both_mis,
    };

    film_grid_t film(3, 4, 256, 256); //film.clear(color_t(1., 0., 0.));
    for (auto sample_enum : sample_enums)
    {
        std::unique_ptr<integrater_t> integrater =
            std::make_unique<path_tracing_iteration_t>(5, sample_enum);

        for (auto [scene_enum, spp] : scene_params)
        {
            std::unique_ptr<sampler_t> sampler =
                std::make_unique<random_sampler_t>(spp);
            scene_t scene = scene_t::create_cornell_box_scene(
                cornell_box_enum_t::both_small_spheres | scene_enum, film.get_resolution());

            integrater->render(&scene, sampler.get(), &film);
            film.next_cell();
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
            std::unique_ptr<integrater_t> integrater =
                std::make_unique<path_tracing_iteration_t>(5, sample_enum);
            integrater->render(&scene, sampler.get(), &film);
            
            film.next_cell();
        }
    }
    */

    film.store_image("multi.bmp"s);
#ifdef KY_WINDOWS
    system("mspaint multi.bmp");
#endif
}

int main(int argc, char* argv[])
{
    clock_t start = clock(); // MILO

    //render_single_scene(argc, argv);
    render_multiple_scene(argc, argv);

    LOG("\n{} sec\n", (Float)(clock() - start) / CLOCKS_PER_SEC); // MILO
    return 0;
}

#pragma endregion



#pragma region window/input

#pragma endregion

