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

#pragma region using/cosntant

using uint = uint32_t;
using Float = double;

constexpr Float k_pi = std::numbers::pi;
constexpr Float k_inv_pi = std::numbers::inv_pi;

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

struct intersection_t
{
    point3_t position; // world position

};

struct shape_t
{
public:
    virtual bool intersect(const ray_t ray, intersection_t& out_intersec) = 0;

private:
    color_t color_;
    surface_scattering_e surface_scattering_;
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
            = -b' +/- sqrt(discr)
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



#pragma region filter

#pragma endregion

#pragma region film

// film_option_t
struct film_desc_t
{
    int width;
    int height;
};

constexpr double clamp01(Float x) { return std::clamp(x, 0., 1.); }
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
    point2_t film_sample_point{};
};

/*
 * generate ray
 * 
 *
 */
class camera_t
{
public:
    virtual ~camera_t() {}
    camera_t(const vec3_t& position, const unit_vec3_t& direction, float focal_length):
        position_{ position },
        direction_{ direction }
    {

    }

public:
    virtual ray_t generate_ray(const camera_sample_t& sample) const
    {

    }

private:
    vec3_t position_;
    unit_vec3_t direction_;
};

#pragma endregion



#pragma region sampling

#pragma endregion

#pragma region sampler

struct RandomLCG {
    unsigned mSeed;
    RandomLCG(unsigned seed = 0) : mSeed(seed) {}
    double operator()() { mSeed = 214013 * mSeed + 2531011; return mSeed * (1.0 / 4294967296); }
};
#define RANDOM(Xi) Xi()
#define RANDOM_INIT(Xi) RandomLCG Xi;
#define RANDOM_PARAM(Xi) RandomLCG& Xi



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

    // TODO
    vec2_t vec2()
    {
        return vec2_t(uniform_float01(), uniform_float01());
    }

    vec3_t vec3()
    {
        return vec3_t(uniform_float01(), uniform_float01(), uniform_float01()); 
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

    // TODO: coroutine
    virtual camera_sample_t get_camera_sample(point2_t p_film)
    {
        return { p_film + rng_.vec2() };
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
        return { p_film + rng_.vec2() };
    }
};

#pragma endregion


#pragma region material, bsdf

class material_t
{

};

#pragma endregion

#pragma region light

#pragma endregion



#pragma region scene

class scene_t : public reference_type_t
{
public:

    const camera_t camera() const { return camera_; }

    intersection_t intersect() const { }

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

class integrater_t
{
public:
    
    void render(const scene_t scene)
    {

    }

    virtual color_t Li()
    {

    }
};

vec3_t radiance(const ray_t& r, int depth, RANDOM_PARAM(Xi))
{
    double t;                               // distance to intersection
    int id = 0;                               // id of intersected object

    if (!intersect(r, t, id))
        return vec3_t(); // if miss, return black

    const sphere_t& obj = scene[id];        // the hit object
    vec3_t x = r.origin_ + r.direction_ * t, 
        n = (x - obj.center_).normlize(), 
        nl = n.dot(r.direction_) < 0 ? n : n * -1, 
        f = obj.color_;
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
        double r1 = 2 * k_pi * RANDOM(Xi), r2 = RANDOM(Xi), r2s = sqrt(r2);
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

class option_t
{

};

int main(int argc, char* argv[])
{
    clock_t start = clock(); // MILO

    int width = 256, height = 256, samples_per_pixel = argc == 2 ? atoi(argv[1]) / 4 : 10; // # samples

    film_t film(width, height);
    ray_t camera(vec3_t(50, 52, 295.6), vec3_t(0, -0.042612, -1).normlize());

    vec3_t cx = vec3_t(width * .5135 / height);
    vec3_t cy = (cx.cross(camera.direction_)).normlize() * .5135;

    std::unique_ptr<sampler_t> sampler = std::make_unique<trapezoidal_sampler_t>(samples_per_pixel);
//#pragma omp parallel for schedule(dynamic, 1) private(sampler)       // OpenMP
    for (int y = 0; y < height; y += 1) 
    {
        // TODO
        LOG("\rRendering ({} spp) {}", sampler->ge_samples_per_pixel(), 100. * y / (height - 1));

        RandomLCG Xi;
        for (int x = 0; x < width; x += 1)
        {
            color_t Li{};
            sampler->start_sample();

            do
            {
                auto camera_sample = sampler->get_camera_sample({ (Float)x, (Float)y });
                vec3_t direction_ =
                    cx * (camera_sample.film_sample_point.x / width - .5) +
                    cy * (camera_sample.film_sample_point.y / height - .5) + camera.direction_;

                auto ray = ray_t(camera.origin_ + direction_ * 140, direction_.normlize());
                Li = Li + radiance(ray, 0, Xi) * (1. / sampler->ge_samples_per_pixel());
            }
            while (sampler->next_sample());

            auto clamp_Li = vec3_t(clamp01(Li.x), clamp01(Li.y), clamp01(Li.z));
            film.add_color(x, y, clamp_Li * 0.25);
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

