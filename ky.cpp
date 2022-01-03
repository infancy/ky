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
    vec3_t operator+(const vec3_t& vec3) const { return vec3_t(x + vec3.x, y + vec3.y, z + vec3.z); }
    vec3_t operator-(const vec3_t& vec3) const { return vec3_t(x - vec3.x, y - vec3.y, z - vec3.z); }
    vec3_t operator*(Float scalar)       const { return vec3_t(x * scalar, y * scalar, z * scalar); }
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
    Float magnitude()    const { return sqrt(magnitude_sq()); }
    Float magnitude_sq() const { return x * x + y * y + z * z; }

    // unit_vec3_t& normlize() const { return *this * (1 / sqrt(x * x + y * y + z * z)); }
    vec3_t& normlize()   { return *this = *this * (1 / sqrt(x * x + y * y + z * z)); } // TODO: const
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

#pragma region shape

enum class surface_scattering_e { diffuse, specular, refractive };  // material types, used in radiance()

class material_t;
class area_light_t;

// surface intersection
struct isect_t
{
    Float distance{ 1e20 }; // distance from ray to intersection
    point3_t position{}; // world position of intersection
    normal_t normal{};

    color_t bsdf{};
    color_t emission{};
    surface_scattering_e surface_scattering{};

    const material_t* material{};
    const area_light_t* area_light{};
};

class shape_t
{
public:
    virtual bool intersect(const ray_t& ray, isect_t& out_isect) const = 0;

protected:
    static constexpr Float eps = 1e-4;
};

class sphere_t : public shape_t
{
public:
    sphere_t(Float radius, vec3_t center, vec3_t e_, vec3_t c_, surface_scattering_e refl_) :
        center_(center),
        radius_(radius),
        radius_sq_(radius * radius),
        emission_(e_),
        color_(c_),
        surface_scattering_(refl_)
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
            isect.distance = t;
            isect.position = ray(t);
            isect.normal = (isect.position - center_).normlize();

            isect.bsdf = color_;
            isect.emission = emission_;
            isect.surface_scattering = surface_scattering_;
        }

        return hit;
    }

private:
    vec3_t center_;
    Float radius_;
    Float radius_sq_;

    color_t emission_;
    color_t color_;
    surface_scattering_e surface_scattering_;
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

        right_ = front_.cross(vec3_t{ 0, 1, 0 }).normlize() * get_aspect() * tan_fov;
        up_ = right_.cross(front_).normlize() * tan_fov;
    }

public:
    virtual ray_t generate_ray(const camera_sample_t& sample) const
    {
        vec3_t direction =
            front_ +
            right_ * (sample.p_film.x / resolution_.x - 0.5) +
            up_ * (sample.p_film.y / resolution_.y - 0.5);

        // TODO
        return ray_t{ position_ + direction * 140, direction.normlize() };
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


#pragma region bsdf, material

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

class bsdf_t
{
public:
    virtual void is_delta() = 0;
};



// class texture_t

class material_t
{
public:
    virtual ~material_t() = default;

    virtual std::unique_ptr<bsdf_t> scattering(isect_t interse) const
    {

    }
};

class matte_material_t
{

};

class mirror_material_t
{

};

class glass_material_t
{

};

#pragma endregion

#pragma region light

class light_t
{
public:
    // is delta distribution
    virtual void is_delta() = 0;

    // sample a delta light or a area light
    virtual void sample() = 0;
};

class aera_light_t
{

};

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

  sphere_t(600, vec3_t(50,681.6 - .27,81.6),vec3_t(12,12,12),  vec3_t(), surface_scattering_e::diffuse) //Light
};

bool intersect(const ray_t& ray, isect_t& isect)
{
    static Float inf = 1e20;

    int n = sizeof(scene) / sizeof(sphere_t);

    for (int i = n; i--;)
        scene[i].intersect(ray, isect);

    return isect.distance < inf;
}

class scene_t : public reference_type_t
{
public:

    const camera_t camera() const { return camera_; }

    isect_t intersect() const { }

private:
    camera_t camera_;

    accel_t accel_;
};

#pragma endregion

#pragma region integrater

enum class integrater_e
{
    // debug
    depth,
    normal,

    // discrete
    delta_bsdf,
    delta_light,
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

    virtual color_t Li() = 0;
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

};

/*
class path_tracing_recursion_light_t : public path_integrater_t
{
public:
    vec3_t Li(const ray_t& r, int depth, sampler_t* sampler, Float include_le = 1);
};
*/

vec3_t radiance(const ray_t& r, int depth, sampler_t* sampler)
{
    isect_t isect;
    if (!intersect(r, isect))
        return vec3_t(); // if miss, return black

    point3_t intersection = isect.position;
    normal_t normal = isect.normal;
    normal_t nl = isect.normal.dot(r.direction_) < 0 ? isect.normal : isect.normal * -1, 
    bsdf = isect.bsdf;

    if (++depth > 5)
    {
        Float bsdf_max_comp = std::max({ bsdf.x, bsdf.y, bsdf.z }); // max refl
        if(sampler->get_float() < bsdf_max_comp)
            bsdf = bsdf * (1 / bsdf_max_comp);
        else
            return isect.emission; //Russian Roulette
    }

    if (depth > 100)
        return isect.emission; // MILO

    if (isect.surface_scattering == surface_scattering_e::diffuse)
    {                  // Ideal DIFFUSE reflection
        Float random1 = 2 * k_pi * sampler->get_float();
        Float random2 = sampler->get_float(), r2s = sqrt(random2);
        vec3_t w = nl;
        vec3_t u = ((fabs(w.x) > 0.1 ? vec3_t(0, 1, 0) : vec3_t(1, 0, 0)).cross(w)).normlize();
        vec3_t v = w.cross(u);
        vec3_t direction_ = (u * cos(random1) * r2s + v * sin(random1) * r2s + w * sqrt(1 - random2)).normlize();

        return isect.emission + bsdf.multiply(
            radiance(ray_t(intersection, direction_), depth, sampler));
    }
    else if (isect.surface_scattering == surface_scattering_e::specular)            // Ideal SPECULAR reflection
    {
        return isect.emission + bsdf.multiply(
            radiance(ray_t(intersection, r.direction_ - normal * 2 * normal.dot(r.direction_)), depth, sampler));
    }
    else
    {
        ray_t reflRay(intersection, r.direction_ - normal * 2 * normal.dot(r.direction_));     // Ideal dielectric REFRACTION
        bool into = normal.dot(nl) > 0;                // Ray from outside going in?
        Float nc = 1;
        Float nt = 1.5;
        Float nnt = into ? nc / nt : nt / nc;
        Float ddn = r.direction_.dot(nl);
        Float cos2t;
        if ((cos2t = 1 - nnt * nnt * (1 - ddn * ddn)) < 0)    // Total internal reflection
            return isect.emission + bsdf.multiply(
                radiance(reflRay, depth, sampler));

        vec3_t tdir = (r.direction_ * nnt - normal * ((into ? 1 : -1) * (ddn * nnt + sqrt(cos2t)))).normlize();
        Float a = nt - nc, b = nt + nc;
        Float R0 = a * a / (b * b);
        Float color_ = 1 - (into ? -ddn : tdir.dot(normal));
        Float Re = R0 + (1 - R0) * color_ * color_ * color_ * color_ * color_;
        Float Tr = 1 - Re;
        Float P = .25 + .5 * Re;
        Float RP = Re / P;
        Float TP = Tr / (1 - P);

        // Russian roulette
        return isect.emission + bsdf.multiply(depth > 2 ?
            (sampler->get_float() < P ? radiance(reflRay, depth, sampler) * RP : radiance(ray_t(intersection, tdir), depth, sampler) * TP) :
            radiance(reflRay, depth, sampler) * Re + radiance(ray_t(intersection, tdir), depth, sampler) * Tr);
    }
}

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

    int width = 256, height = 256, samples_per_pixel = argc == 2 ? atoi(argv[1]) / 4 : 10; // # samples

    film_t film(width, height);
    std::unique_ptr<const camera_t> camera = 
        std::make_unique<camera_t>(vec3_t{ 50, 52, 295.6 }, vec3_t{ 0, -0.042612, -1 }.normlize(), 53, film.get_resolution());

#ifndef KY_DEBUG
    #pragma omp parallel for schedule(dynamic, 1) // OpenMP
#endif // !KY_DEBUG
    for (int y = 0; y < height; y += 1) 
    {
        std::unique_ptr<sampler_t> sampler = std::make_unique<trapezoidal_sampler_t>(samples_per_pixel);
        LOG("\rRendering ({} spp) {}", sampler->ge_samples_per_pixel(), 100. * y / (height - 1));

        for (int x = 0; x < width; x += 1)
        {
            color_t Li{};
            sampler->start_sample();

            do
            {
                auto sample = sampler->get_camera_sample({ (Float)x, (Float)y });
                auto ray = camera->generate_ray(sample);

                Li = Li + radiance(ray, 0, sampler.get()) * (1. / sampler->ge_samples_per_pixel());
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

