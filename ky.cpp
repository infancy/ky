#include <algorithm>
#include <array>
#include <cmath>   // smallpt, a Path Tracer by Kevin Beason, 2008
#include <cstdlib> // Make : g++ -O3 -fopenmp smallpt.cpp -o smallpt
#include <cstdio>
#include <ctime>
#include <exception>
#include <format>
#include <memory>
#include <numbers>
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

#pragma region using/cosntant/math

using uint = uint32_t;
using Float = double;

using radian_t = Float;
using degree_t = Float;

constexpr Float k_pi = std::numbers::pi;
constexpr Float k_inv_pi = std::numbers::inv_pi;

constexpr radian_t radians(degree_t deg) { return (k_pi / 180) * deg; }
constexpr degree_t degrees(radian_t rad) { return (180 / k_pi) * rad; }

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

template <typename... Ts>
inline void LOG_ERROR(const std::string_view fmt, const Ts&... args)
{
    auto msg = std::vformat(fmt, std::make_format_args(args...));
    std::printf("%s", msg.c_str());
    throw std::exception(msg.c_str());
}

#ifdef KY_DEBUG
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



struct vec2_t
{
    union
    {
        struct { Float x, y; };
    };

    vec2_t(Float x = 0, Float y = 0) { this->x = x; this->y = y; }

    vec2_t operator+(const vec2_t& vec2) const { return vec2_t(x + vec2.x, y + vec2.y); }
};

using point2_t = vec2_t;



struct vec3_t
{        // Usage: time ./smallpt 5000 && xv image.ppm
    union
    {
        struct { Float x, y, z; };
        struct { Float r, g, b; };
        //std::array<Float, 3> array_{};
    };

    vec3_t(Float x = 0, Float y = 0, Float z = 0) { this->x = x; this->y = y; this->z = z; }

    Float operator[](int i)              const { DCHECK(i >= 0 && i < 3); return (&x)[i]; }
    vec3_t operator-() const { return vec3_t(-x, -y, -z); }
    vec3_t operator+(const vec3_t& v) const { return vec3_t(x + v.x, y + v.y, z + v.z); }
    vec3_t operator-(const vec3_t& v) const { return vec3_t(x - v.x, y - v.y, z - v.z); }
    vec3_t operator*(Float scalar)       const { return vec3_t(x * scalar, y * scalar, z * scalar); }
    vec3_t operator/(Float scalar)       const { return vec3_t(x / scalar, y / scalar, z / scalar); }

    friend vec3_t operator*(Float scalar, vec3_t v) { return vec3_t(v.x * scalar, v.y * scalar, v.z * scalar); }

    // ???
    vec3_t multiply(const vec3_t& v) const { return vec3_t(x * v.x, y * v.y, z * v.z); }

    friend Float     dot(const vec3_t& u, const vec3_t& v) { return u.dot(v); }
    friend Float abs_dot(const vec3_t& u, const vec3_t& v) { return std::abs(u.dot(v)); }
    friend vec3_t  cross(const vec3_t& u, const vec3_t& v) { return u.cross(v); }
    friend vec3_t normalize(const vec3_t& v) { return v.normalize(); }

    friend Float     cos(const vec3_t& u, const vec3_t& v) { return u.dot(v); }
    friend Float abs_cos(const vec3_t& u, const vec3_t& v) { return std::abs(u.dot(v)); }

    Float dot(const vec3_t& v) const { return x * v.x + y * v.y + z * v.z; }
    vec3_t cross(const vec3_t& v) const
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
    Float magnitude()    const { return sqrt(magnitude_sq()); }
    Float magnitude_sq() const { return x * x + y * y + z * z; }

    // unit_vec3_t& normlize() const { return *this * (1 / sqrt(x * x + y * y + z * z)); }
    vec3_t normalize() const { return *this * (1 / sqrt(x * x + y * y + z * z)); }
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
    // think if stn is (1, 0, 0), (0, 1, 0), (0, 0, 1)
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

private:
    point3_t origin_;
    unit_vec3_t direction_; // confirm it is Unit Vector
};



enum class surface_scattering_e { diffuse, specular, refractive };  // material types, used in radiance()

// TODO
class bsdf_t;
using bsdf_uptr_t = std::unique_ptr<bsdf_t>;
class material_t;
class area_light_t;
class primitive_t;

// prev   n   light
// ----   ^   -----
//   ^    |    ^
//    \   |   /
//     \  |  /
//   wo \ | / wi.direction is unknown, sampling for bsdf or light?
//       \|/
//     -------
//      isect

// surface intersection
class isect_t
{
public:
    isect_t() = default;
    isect_t(Float distance, point3_t position, unit_vec3_t wo, normal_t normal) :
        distance{ distance },
        position{ position },
        wo{ wo },
        normal{ normal }
    {
    }
    isect_t& operator=(isect_t&& isect) = default;

    surface_scattering_e surface_scattering_type() const;
    //void scattering();

    const bsdf_t* get_bsdf() const { return bsdf_.get(); }
    color_t emission() const { return emission_; }

public:
    Float distance{ 1e20 }; // distance from ray to intersection
    point3_t position{}; // world position of intersection
    unit_vec3_t wo{};
    normal_t normal{};

private:
    friend primitive_t;

    const material_t* material_{}; // TODO: remove
    bsdf_uptr_t bsdf_{};
    color_t emission_{};
};

#pragma endregion

#pragma region shape

class shape_t
{
public:
    virtual bool intersect(const ray_t& ray, isect_t& out_isect) const = 0;

protected:
    static constexpr Float eps = 1e-4;
};

using shape_sp = std::shared_ptr<shape_t>; 
using shape_list_t = std::vector<shape_sp>;



/*
       z(0, 0, 1)
       |
       |
       |
       |
       |_ _ _ _ _ _ x(1, 0, 0)
      /
     /
    /
   y(0, 1, 0)

   https://www.pbr-book.org/3ed-2018/Shapes/Spheres
*/

class sphere_t : public shape_t
{
public:
    sphere_t(Float radius, vec3_t center) :
        center_(center),
        radius_(radius),
        radius_sq_(radius * radius)
    {
    }

    bool intersect(const ray_t& ray, isect_t& isect) const override
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

        Float t = 0;
        bool hit = false;
        if (discr >= 0)
         {
            Float sqrt_discr = sqrt(discr);

            if (t = neg_b - sqrt_discr; t > eps && t < isect.distance)
            {
                hit = true;
            }
            else if (t = neg_b + sqrt_discr; t > eps && t < isect.distance)
            {
                hit = true;
            }
        }

        if (hit)
        {
            point3_t hit_point = ray(t);
            isect = isect_t(t, hit_point, -ray.direction(), (hit_point - center_).normalize());
        }

        return hit;
    }

private:
    vec3_t center_;
    Float radius_;
    Float radius_sq_;
};

#pragma endregion

#pragma region accelerator

enum class accel_e
{
    trivial
    // bvh
};

class accel_t
{

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

constexpr Float clamp01(Float x) { return std::clamp(x, 0., 1.); }
std::byte gamma_encoding(Float x) { return std::byte(pow(clamp01(x), 1 / 2.2) * 255 + .5); }

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
        pixels_{ std::make_unique<color_t[]>(width_ * height_) }
    {
    }

public:
    int get_width() const { return width_; }
    int get_height() const { return height_; }
    int get_channels() const { return 3; }
    vec2_t get_resolution() const { return { (Float)width_, (Float)height_ }; }

    color_t& operator()(int x, int y)
    {
        DCHECK(x >= 0 && x < width_&& y >= 0 && y < height_);
        return *(pixels_.get() + get_width() * y + x);
    }

    void add_color(int x, int y, const color_t& delta)
    {
        auto& color_ = operator()(x, y);
        color_ = color_ + delta;
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
        FILE* file = fopen(filename.c_str(), "wb");
        if (file == nullptr) return false;


        uint32_t padding_line_bytes = (width * channel + 3) & (~3);
        uint32_t padding_image_bytes = padding_line_bytes * height;

        const uint32_t FILE_HEADER_SIZE = 14;
        const uint32_t INFO_HEADER_SIZE = 40; 

        // write file header
        struct BITMAP_FILE_HEADER
        {
            char8_t padding[2]{};

            char8_t type[2]{ 'B', 'M' };
            uint32_t file_size{};
            uint32_t reserved{ 0 };
            uint32_t databody_offset{};
        }
        file_header{ .file_size{ FILE_HEADER_SIZE + INFO_HEADER_SIZE + padding_image_bytes } };

        static_assert(sizeof(file_header) == FILE_HEADER_SIZE + 2);
        fwrite(&file_header.type, FILE_HEADER_SIZE, 1, file);


        // write info header
        struct BITMAP_INFO_HEADER
        {
            uint32_t	info_header_size{ INFO_HEADER_SIZE };

            uint32_t	width{};
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
        info_header
        { 
            .width{ (uint32_t)width },
            .height{ (uint16_t)height },
            .per_pixel_bits{ (uint16_t)(channel * 8) },
            .image_bytes{ uint32_t(padding_image_bytes) }
        };

        static_assert(sizeof(info_header) == INFO_HEADER_SIZE);
        fwrite(&info_header, INFO_HEADER_SIZE, 1, file);

        
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
        fwrite(bytes.get(), byte_num, 1, file);


        fclose(file);
        return true;
    }

private:
    int32_t width_;
    int32_t height_;

    std::unique_ptr<color_t[]> pixels_;
};


#pragma endregion

#pragma region camera

struct camera_sample_t
{
    point2_t p_film{}; // film_sample_point
};

/*
 * OpenGL-style
 * generate ray
 * 
 *
 */
class camera_t
{
public:
    virtual ~camera_t() {}
    camera_t(const vec3_t& position, const unit_vec3_t& direction, degree_t fov, vec2_t resolution):
        position_{ position },
        front_{ direction },
        resolution_{ resolution }
    {
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/transform.cpp#L394-L397
        // `front_` is a unit vector, it's length is 1
        Float tan_fov = std::tan(radians(fov) / 2);

        right_ = front_.cross(vec3_t{ 0, 1, 0 }).normalize() * get_aspect() * tan_fov;
        up_ = right_.cross(front_).normalize() * tan_fov;
    }

public:
    virtual ray_t generate_ray(const camera_sample_t& sample) const
    {
        vec3_t direction =
            front_ +
            right_ * (sample.p_film.x / resolution_.x - 0.5) +
            up_ * (sample.p_film.y / resolution_.y - 0.5);

        // TODO
        return ray_t{ position_ + direction * 140, direction.normalize() };
    }

private:
    Float get_aspect() { return resolution_.x / resolution_.y; }

private:
    vec3_t position_;
    unit_vec3_t front_;
    vec3_t right_;
    vec3_t up_;

    vec2_t resolution_;
};

// TODO: smallpt_camera

#pragma endregion



#pragma region sampling

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



struct light_sample_t
{
};

struct bsdf_sample_t
{
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


    virtual void start_sample()
    {
        current_sample_index_ = 0;
    }

    virtual bool next_sample()
    {
        current_sample_index_ += 1;
        return current_sample_index_ < samples_per_pixel_;
    }


    virtual Float get_float()
    {
        return rng_.uniform_float01();
    }

    virtual vec2_t get_vec2()
    {
        return rng_.uniform_vec2();
    }

    // TODO: coroutine
    virtual camera_sample_t get_camera_sample(point2_t p_film)
    {
        return { p_film + rng_.uniform_vec2() };
    }

protected:
    rng_t rng_{};

    int samples_per_pixel_{};
    int current_sample_index_{};
};

// for smallpt
// https://computergraphics.stackexchange.com/questions/3868/why-use-a-tent-filter-in-path-tracing
class trapezoidal_sampler_t : public sampler_t
{
public:
    trapezoidal_sampler_t(int samples_per_pixel) : sampler_t(samples_per_pixel)
    {
    }

    int ge_samples_per_pixel() override
    {
        return samples_per_pixel_ * k_sub_pixel_num;
    }

    void start_sample() override
    {
        sampler_t::start_sample();
        current_sub_pixel_index_ = 0;
    }

    bool next_sample() override
    {
        current_sample_index_ += 1;
        if (current_sample_index_ < samples_per_pixel_)
        {
            return true;
        }
        else if (current_sample_index_ == samples_per_pixel_)
        {
            current_sample_index_ = 0;
            current_sub_pixel_index_ += 1;

            return current_sub_pixel_index_ < k_sub_pixel_num;
        }
        else
        {
            LOG_ERROR("shouldn't be there");
            return false;
        }
    }

    camera_sample_t get_camera_sample(point2_t p_film) override
    {
        int sub_pixel_x = current_sub_pixel_index_ % 2;
        int sub_pixel_y = current_sub_pixel_index_ / 2;

        Float random1 = 2 * rng_.uniform_float01();
        Float random2 = 2 * rng_.uniform_float01();

        // uniform dist [0, 1) => triangle dist [-1, 1)
        Float delta_x = random1 < 1 ? sqrt(random1) - 1 : 1 - sqrt(2 - random1);
        Float delta_y = random2 < 1 ? sqrt(random2) - 1 : 1 - sqrt(2 - random2);

        point2_t sample_point
        {
            (sub_pixel_x + delta_x + 0.5) / 2,
            (sub_pixel_y + delta_y + 0.5) / 2
        };

        return { p_film + sample_point };
    }

private:
    static constexpr int k_sub_pixel_num = 4;

    int current_sub_pixel_index_{};
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


#pragma region bsdf

inline Float abs_cos_theta(const vec3_t& w) { return std::abs(w.z); }

// https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#SpecularReflection
inline vec3_t reflect(const vec3_t& wo, const normal_t& normal)
{
    return -wo + 2 * dot(wo, normal) * normal;
}

// eta = etaI/etaT
inline bool refract(const vec3_t& wi, const normal_t& normal, Float eta,
    vec3_t* wt)
{
    // Compute $\cos \theta_\roman{t}$ using Snell's law
    Float cosThetaI = dot(normal, wi);
    Float sin2ThetaI = std::max(Float(0), Float(1 - cosThetaI * cosThetaI));
    Float sin2ThetaT = eta * eta * sin2ThetaI;

    // Handle total internal reflection for transmission
    if (sin2ThetaT >= 1)
        return false;

    Float cosThetaT = std::sqrt(1 - sin2ThetaT);
    *wt = eta * -wi + (eta * cosThetaI - cosThetaT) * vec3_t(normal);

    return true;
}



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



enum class bsdf_type_e
{
    reflection,
    transmission,
    scattering = reflection | transmission,

    diffuse,
    glossy,
    specluar,
};

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
      \   |   /
       \  |  /
     wo \ | / wi, wi.direction is unknown, sampling for bsdf or light?
         \|/
       -------
        isect

   https://www.pbr-book.org/3ed-2018/Reflection_Models#x0-GeometricSetting
*/

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

    color_t sample_f(const vec3_t& world_wo, const point2_t& p_sample, 
        vec3_t* out_world_wi, Float* out_pdf, bsdf_type_e* out_bsdf_type) const
    {
        auto value = sample_f_(to_local(world_wo), p_sample, out_world_wi, out_pdf, out_bsdf_type);
        *out_world_wi = to_world(*out_world_wi);

        return value;
    }

protected: 
    virtual color_t f_(const vec3_t& wo, const vec3_t& wi) const = 0;
    virtual Float pdf_(const vec3_t& wo, const vec3_t& wi) const = 0;

    virtual color_t sample_f_(const vec3_t& wo, const point2_t& p_sample, 
        vec3_t* out_wi, Float* out_pdf, bsdf_type_e* out_bsdf_type) const = 0;

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

    // TODO
    color_t f_(const vec3_t& wo, const vec3_t& wi) const override { return R_; }
    Float pdf_(const vec3_t& wo, const vec3_t& wi) const override { return 0; } // TODO

    color_t sample_f_(const vec3_t& wo, const point2_t& sample, 
        vec3_t* out_wi, Float* out_pdf, bsdf_type_e* out_bsdf_type) const override
    {

        return R_; 
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

    color_t sample_f_(const vec3_t& wo, const point2_t&, 
        vec3_t* out_wi, Float* out_pdf, bsdf_type_e* out_bsdf_type) const override 
    {
        // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#SpecularReflection
        // https://github.com/infancy/pbrt-v3/blob/master/src/core/reflection.cpp#L181-L191

        //*out_wi = reflect(wo, vec3_t(0, 0, 1));
        *out_wi = vec3_t(-wo.x, -wo.y, wo.z);
        *out_pdf = 1;
        *out_bsdf_type = bsdf_type_e::reflection;

        return R_ / abs_cos_theta(*out_wi);
    }

private:
    color_t R_;
};



class fresnel_specular_t : public bsdf_t
{
public:
    fresnel_specular_t(const frame_t& shading_frame, const color_t& R) :
        bsdf_t(shading_frame), R_{ R }
    {
    }

    bool is_delta() const override { return true; }

    color_t f_(const vec3_t& wo, const vec3_t& wi) const override { return color_t(); }
    Float pdf_(const vec3_t& wo, const vec3_t& wi) const override { return 0; }

    color_t sample_f_(const vec3_t& wo, const point2_t&, 
        vec3_t* out_wi, Float* out_pdf, bsdf_type_e* out_bsdf_type) const override
    {
        *out_wi = vec3_t(-wo.x, -wo.y, wo.z);
        *out_pdf = 1;

        return R_ / abs_cos_theta(*out_wi);
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

    virtual surface_scattering_e get_surface_scattering_type() const = 0;
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

    surface_scattering_e get_surface_scattering_type() const override { return surface_scattering_e::diffuse; }

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

    surface_scattering_e get_surface_scattering_type() const override { return surface_scattering_e::specular; }

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
    glass_material_t(const color_t Kr, const color_t Kt) :
        Kr_{ Kr }
    {
    }

    surface_scattering_e get_surface_scattering_type() const override { return surface_scattering_e::refractive; }

    bsdf_uptr_t scattering(const isect_t& isect) const override
    {
        return std::make_unique<fresnel_specular_t>(frame_t(isect.normal), Kr_);
    }

private:
    color_t Kr_;
};

surface_scattering_e isect_t::surface_scattering_type() const
{
    return material_->get_surface_scattering_type();
}

#pragma endregion

#pragma region light

class light_t
{
public:
    // is delta distribution
    virtual bool is_delta() = 0;

    // sample a delta light or a area light
    //virtual void sample() = 0;
};

using light_sp = std::shared_ptr<light_t>;
using light_list_t = std::vector<light_sp>;

class point_light_t : public light_t
{
public:
    point_light_t(color_t Iemit) :
        Iemit_{ Iemit }
    {
    }

    bool is_delta() override { return true; }

private:
    color_t Iemit_;
};

class area_light_t : public light_t
{
public:
    area_light_t(color_t Lemit, const shape_t* shape):
        Lemit_{ Lemit },
        shape_{ shape }
    {
    }

    bool is_delta() override { return false; }

    color_t emission() const { return Lemit_; }

private:
    color_t Lemit_;
    const shape_t* shape_;
};

#pragma endregion



#pragma region primitive

struct primitive_t
{
    const shape_t* shape;
    const material_t* material;
    const area_light_t* area_light;

    bool intersect(const ray_t& ray, isect_t& isect)
    {
        bool hit = shape->intersect(ray, isect);
        if (hit)
        {
            isect.material_ = material;
            isect.bsdf_ = material->scattering(isect);
            isect.emission_ = area_light ? area_light->emission() : color_t();
        }

        return hit;
    }
};

using primitive_list_t = std::vector<primitive_t>;

#pragma endregion

#pragma region scene

enum class scene_type_e
{
    point_light_diffuse_ball,
    area_light_specular_ball,
};

class scene_t : public reference_type_t
{
public:
    scene_t() = default;
    scene_t(shape_list_t shape_list, material_list_t material_list, light_list_t light_list, primitive_list_t primitive_list) :
        shape_list_{ shape_list },
        material_list_{ material_list },
        light_list_{ light_list },
        primitive_list_{ primitive_list }
    {
    }

    bool intersect(const ray_t& ray, isect_t& isect)
    {
        static Float inf = 1e20;

        int n = primitive_list_.size();

        for (int i = n; i--;)
            primitive_list_[i].intersect(ray, isect);

        return isect.distance < inf;
    }

    //const camera_t camera() const { return camera_; }

public:
    static scene_t create_smallpt_scene(scene_type_e scene_type)
    {
        shape_sp left   = std::make_shared<sphere_t>(1e5, vec3_t(1e5 + 1, 40.8, 81.6));
        shape_sp right  = std::make_shared<sphere_t>(1e5, vec3_t(-1e5 + 99, 40.8, 81.6));
        shape_sp back   = std::make_shared<sphere_t>(1e5, vec3_t(50, 40.8, 1e5));
        shape_sp front  = std::make_shared<sphere_t>(1e5, vec3_t(50, 40.8, -1e5 + 170));
        shape_sp bottom = std::make_shared<sphere_t>(1e5, vec3_t(50, 1e5, 81.6));
        shape_sp top    = std::make_shared<sphere_t>(1e5, vec3_t(50, -1e5 + 81.6, 81.6));

        shape_sp mirror = std::make_shared<sphere_t>(16.5, vec3_t(27, 16.5, 47));
        shape_sp glass  = std::make_shared<sphere_t>(16.5, vec3_t(73, 16.5, 78));
        shape_sp light  = std::make_shared<sphere_t>(600, vec3_t(50, 681.6 - .27, 81.6));
        shape_list_t shape_list{ left, right, back, front, bottom, top, mirror, glass, light };


        material_sp red   = std::make_shared<matte_material_t>(color_t(.75, .25, .25));
        material_sp blue  = std::make_shared<matte_material_t>(color_t(.25, .25, .75));
        material_sp gray  = std::make_shared<matte_material_t>(color_t(.75, .75, .75));
        material_sp black = std::make_shared<matte_material_t>(color_t());

        material_sp mirror_mat = std::make_shared<mirror_material_t>(color_t(1, 1, 1) * 0.999);
        material_sp glass_mat  = std::make_shared<glass_material_t>(color_t(1, 1, 1) * 0.999, color_t());
        material_list_t material_list{ red, blue, gray, black, mirror_mat, glass_mat };


        std::shared_ptr<area_light_t> area_light = std::make_shared<area_light_t>(color_t(12, 12, 12), light.get());
        light_list_t light_list{ area_light };


        primitive_list_t primitive_list
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

        return scene_t{ shape_list, material_list, light_list, primitive_list };
    }

private:
    shape_list_t shape_list_;
    material_list_t material_list_;
    light_list_t light_list_;

    // TODO: std::vector<std::function<intersect(ray_t ray), result_t> primitives_;
    primitive_list_t primitive_list_;

    //camera_t camera_;
    accel_t accel_;
};

scene_t scene = scene_t::create_smallpt_scene(scene_type_e::area_light_specular_ball);

#pragma endregion

#pragma region integrater

enum class integrater_e
{
    // debug
    depth,
    normal,

    // discrete
    delta_bsdf, // + area light
    delta_light, // + diffuse brdf
    direct_lighting_point, 

    // ray casting/direct lighting
    direct_lighting_bsdf,
    direct_lighting_light,
    direct_lighting_mis,

    // stochastic ray tracing
    stochastic_ray_tracing,

    // path tracing
    path_tracing_recursion_bsdf,
    path_tracing_recursion_light,

    path_tracing_recursion_mis,
    path_tracing_iteration
};

class integrater_t
{
public:
    void render(const scene_t scene)
    {

    }

    virtual color_t Li(const ray_t& r, int depth, sampler_t* sampler) = 0;
};



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

class path_tracing_recursion_bsdf_t : public path_integrater_t
{
public:
    using path_integrater_t::path_integrater_t;

    color_t Li(const ray_t& r, int depth, sampler_t* sampler) override
    {
        isect_t isect;
        if (!scene.intersect(r, isect))
            return color_t(); // if miss, return black

        point3_t position = isect.position;
        normal_t normal = isect.normal;
        normal_t nl = isect.normal.dot(r.direction()) < 0 ? isect.normal : isect.normal * -1;

        vec3_t wi;
        Float pdf;
        bsdf_type_e bsdf_type;
        color_t bsdf = isect.get_bsdf()->sample_f(isect.wo, sampler->get_vec2(), &wi, &pdf, &bsdf_type);

        if (++depth > 5)
        {
            Float bsdf_max_comp = std::max({ bsdf.x, bsdf.y, bsdf.z }); // max refl
            if (sampler->get_float() < bsdf_max_comp)
                bsdf = bsdf * (1 / bsdf_max_comp); // importance sampling
            else
                return isect.emission(); //Russian Roulette
        }

        if (depth > max_path_depth_)
            return isect.emission(); // MILO

        // return isect.emission() + ...
        if (isect.surface_scattering_type() == surface_scattering_e::diffuse)
        {                  // Ideal DIFFUSE reflection
            Float random1 = 2 * k_pi * sampler->get_float();
            Float random2 = sampler->get_float(), r2s = sqrt(random2);
            vec3_t w = nl;
            vec3_t u = ((fabs(w.x) > 0.1 ? vec3_t(0, 1, 0) : vec3_t(1, 0, 0)).cross(w)).normalize();
            vec3_t v = w.cross(u);
            vec3_t direction = (u * cos(random1) * r2s + v * sin(random1) * r2s + w * sqrt(1 - random2)).normalize();

            return isect.emission() + bsdf.multiply(
                Li(ray_t(position, direction), depth, sampler));
        }
        else if (isect.surface_scattering_type() == surface_scattering_e::specular)            // Ideal SPECULAR reflection
        {
            ray_t ray(position, wi);
            return isect.emission() + bsdf.multiply(Li(ray, depth, sampler)) * abs_dot(wi, normal) / pdf;
        }
        else
        {
            ray_t reflRay(position, r.direction() - normal * 2 * normal.dot(r.direction()));     // Ideal dielectric REFRACTION
            bool into = normal.dot(nl) > 0;                // Ray from outside going in?
            Float nc = 1;
            Float nt = 1.5;
            Float nnt = into ? nc / nt : nt / nc;
            Float ddn = r.direction().dot(nl);
            Float cos2t;
            if ((cos2t = 1 - nnt * nnt * (1 - ddn * ddn)) < 0)    // Total internal reflection
                return isect.emission() + bsdf.multiply(
                    Li(reflRay, depth, sampler));

            vec3_t tdir = (r.direction() * nnt - normal * ((into ? 1 : -1) * (ddn * nnt + sqrt(cos2t)))).normalize();
            Float a = nt - nc, b = nt + nc;
            Float R0 = a * a / (b * b);
            Float color_ = 1 - (into ? -ddn : tdir.dot(normal));
            Float Re = R0 + (1 - R0) * color_ * color_ * color_ * color_ * color_;
            Float Tr = 1 - Re;
            Float P = .25 + .5 * Re;
            Float RP = Re / P;
            Float TP = Tr / (1 - P);

            // Russian roulette
            return isect.emission() + bsdf.multiply(depth > 2 ?
                (sampler->get_float() < P ? Li(reflRay, depth, sampler) * RP : Li(ray_t(position, tdir), depth, sampler) * TP) :
                Li(reflRay, depth, sampler) * Re + Li(ray_t(position, tdir), depth, sampler) * Tr);
        }
    }
};

/*
class path_tracing_recursion_light_t : public path_integrater_t
{
public:
    vec3_t Li(const ray_t& r, int depth, sampler_t* sampler, Float include_le = 1);
    // Le(emit), Ld(direct), Li(indirect)
};
*/

#pragma endregion



#pragma region interactive/debug

// single ray debug

#pragma endregion

#pragma region main

class option_t
{

};

class build_t_
{

};

int main(int argc, char* argv[])
{
    clock_t start = clock(); // MILO

    int width = 1024, height = 1024, samples_per_pixel = argc == 2 ? atoi(argv[1]) / 4 : 10; // # samples

    film_t film(width, height);
    std::unique_ptr<const camera_t> camera = 
        std::make_unique<camera_t>(vec3_t{ 50, 52, 295.6 }, vec3_t{ 0, -0.042612, -1 }.normalize(), 53, film.get_resolution());

    std::unique_ptr<integrater_t> integrater = std::make_unique<path_tracing_recursion_bsdf_t>(100);

#ifndef KY_DEBUG
    #pragma omp parallel for schedule(dynamic, 1) // OpenMP
#endif // !KY_DEBUG
    for (int y = 256; y < 512; y += 1) 
    {
        std::unique_ptr<sampler_t> sampler = std::make_unique<trapezoidal_sampler_t>(samples_per_pixel);
        LOG("\rRendering ({} spp) {}", sampler->ge_samples_per_pixel(), 100. * y / (height - 1));

        for (int x = 256; x < 512; x += 1)
        {
            color_t Li{};
            sampler->start_sample();

            do
            {
                auto sample = sampler->get_camera_sample({ (Float)x, (Float)y });
                auto ray = camera->generate_ray(sample);

                Li = Li + integrater->Li(ray, 0, sampler.get()) * (1. / sampler->ge_samples_per_pixel());
            }
            while (sampler->next_sample());

            auto clamp_Li = vec3_t(clamp01(Li.x), clamp01(Li.y), clamp01(Li.z));
            film.add_color(x, y, clamp_Li);
        }
    }

    LOG("\n{} sec\n", (float)(clock() - start) / CLOCKS_PER_SEC); // MILO

    film.store_image("image.bmp"s);
#ifdef KY_WINDOWS
    system("mspaint image.bmp");
#endif
    return 0;
}

#pragma endregion



#pragma region window/input

#pragma endregion

