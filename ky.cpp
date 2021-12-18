#include <array>
#include <cmath>   // smallpt, a Path Tracer by Kevin Beason, 2008
#include <cstdlib> // Make : g++ -O3 -fopenmp smallpt.cpp -o smallpt
#include <cstdio>
#include <ctime>
#include <numbers>

#pragma region macro

#pragma endregion



#pragma region using/cosntant

;
using Float = double;
constexpr Float Pi = std::numbers::pi;

#pragma endregion

struct RandomLCG {
    unsigned mSeed;
    RandomLCG(unsigned seed = 0) : mSeed(seed) {}
    double operator()() { mSeed = 214013 * mSeed + 2531011; return mSeed * (1.0 / 4294967296); }
};
#define RANDOM(Xi) Xi()
#define RANDOM_INIT(Xi) RandomLCG Xi;
#define RANDOM_PARAM(Xi) RandomLCG& Xi

#pragma region geometry

struct vec3_t
{        // Usage: time ./smallpt 5000 && xv image.ppm
    union
    {
        struct { Float x, y, z; };
        struct { Float r, g, b; };
        //std::array<Float, 3> array_{};
    };

    vec3_t(Float x = 0, Float y = 0, Float z = 0) { this->x = x; this->y = y; this->z = z; }

    vec3_t operator+(const vec3_t& vec3) const { return vec3_t(x + vec3.x, y + vec3.y, z + vec3.z); }
    vec3_t operator-(const vec3_t& vec3) const { return vec3_t(x - vec3.x, y - vec3.y, z - vec3.z); }
    vec3_t operator*(Float scalar) const { return vec3_t(x * scalar, y * scalar, z * scalar); }

    vec3_t multiply(const vec3_t& b) const { return vec3_t(x * b.x, y * b.y, z * b.z); }
    vec3_t& normlize() { return *this = *this * (1 / sqrt(x * x + y * y + z * z)); } // TODO: const
    Float dot(const vec3_t& b) const { return x * b.x + y * b.y + z * b.z; } // cross:
    vec3_t operator%(const vec3_t& b) { return vec3_t(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x); } // TODO: rename cross
};

using point3_t = vec3_t;
using normal_t = vec3_t;
using color_t = vec3_t;

struct ray_t
{
    point3_t origin_;
    vec3_t direction_;

    ray_t(const point3_t& origin, const vec3_t& direction) : origin_(origin), direction_(direction) {}
};

#pragma endregion



enum class surface_scattering_t { diffuse, specular, refractive };  // material types, used in radiance()

struct sphere_t
{
    Float radius_;
    Float sq_radius_;
    vec3_t position_;

    color_t emission_;
    color_t color_;
    surface_scattering_t surface_scattering_;

    sphere_t(Float rad_, vec3_t p_, vec3_t e_, vec3_t c_, surface_scattering_t refl_) :
        radius_(rad_),
        position_(p_),
        emission_(e_),
        color_(c_),
        surface_scattering_(refl_),
        sq_radius_(rad_* rad_)
    {
    }

    Float intersect(const ray_t& r) const
    { // returns distance, 0 if nohit
        vec3_t op = position_ - r.origin_; // Solve t^2*d.d + 2*t*(o-p).d + (o-p).(o-p)-R^2 = 0
        Float t, eps = 1e-4, b = op.dot(r.direction_), det = b * b - op.dot(op) + sq_radius_; // MILO
        if (det < 0) return 0; else det = sqrt(det);
        return (t = b - det) > eps ? t : ((t = b + det) > eps ? t : 0);
    }
};

sphere_t scene[] = {//Scene: radius, position, emission, color, material
  sphere_t(1e5, vec3_t(1e5 + 1,40.8,81.6), vec3_t(),vec3_t(.75,.25,.25),surface_scattering_t::diffuse),//Left
  sphere_t(1e5, vec3_t(-1e5 + 99,40.8,81.6),vec3_t(),vec3_t(.25,.25,.75),surface_scattering_t::diffuse),//Rght
  sphere_t(1e5, vec3_t(50,40.8, 1e5),     vec3_t(),vec3_t(.75,.75,.75),surface_scattering_t::diffuse),//Back
  sphere_t(1e5, vec3_t(50,40.8,-1e5 + 170), vec3_t(),vec3_t(),           surface_scattering_t::diffuse),//Frnt
  sphere_t(1e5, vec3_t(50, 1e5, 81.6),    vec3_t(),vec3_t(.75,.75,.75),surface_scattering_t::diffuse),//Botm
  sphere_t(1e5, vec3_t(50,-1e5 + 81.6,81.6),vec3_t(),vec3_t(.75,.75,.75),surface_scattering_t::diffuse),//Top
  sphere_t(16.5,vec3_t(27,16.5,47),       vec3_t(),vec3_t(1,1,1) * .999, surface_scattering_t::specular),//Mirr
  sphere_t(16.5,vec3_t(73,16.5,78),       vec3_t(),vec3_t(1,1,1) * .999, surface_scattering_t::refractive),//Glas
  sphere_t(600, vec3_t(50,681.6 - .27,81.6),vec3_t(12,12,12),  vec3_t(), surface_scattering_t::diffuse) //Lite
};

double clamp(double x) { return x < 0 ? 0 : x>1 ? 1 : x; }

int toInt(double x) { return int(pow(clamp(x), 1 / 2.2) * 255 + .5); }

bool intersect(const ray_t& r, double& t, int& id)
{
    double n = sizeof(scene) / sizeof(sphere_t), direction_, inf = t = 1e20;
    for (int i = int(n); i--;)
        if ((direction_ = scene[i].intersect(r)) && direction_ < t)
        {
            t = direction_; id = i;
        }

    return t < inf;
}

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

    if (obj.surface_scattering_ == surface_scattering_t::diffuse)
    {                  // Ideal DIFFUSE reflection
        double r1 = 2 * Pi * RANDOM(Xi), r2 = RANDOM(Xi), r2s = sqrt(r2);
        vec3_t w = nl, u = ((fabs(w.x) > .1 ? vec3_t(0, 1) : vec3_t(1)) % w).normlize(), v = w % u;
        vec3_t direction_ = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normlize();
        return obj.emission_ + f.multiply(radiance(ray_t(x, direction_), depth, Xi));
    }
    else if (obj.surface_scattering_ == surface_scattering_t::specular)            // Ideal SPECULAR reflection
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

int main(int argc, char* argv[])
{
    clock_t start = clock(); // MILO

    int w = 256, h = 256, samps = argc == 2 ? atoi(argv[1]) / 4 : 1; // # samples

    ray_t cam(vec3_t(50, 52, 295.6), vec3_t(0, -0.042612, -1).normlize()); // cam pos, dir

    vec3_t cx = vec3_t(w * .5135 / h), cy = (cx % cam.direction_).normlize() * .5135, r, * color_ = new vec3_t[w * h];

#pragma omp parallel for schedule(dynamic, 1) private(r)       // OpenMP
    for (int y = 0; y < h; y++) {                       // Loop over image rows
        fprintf(stderr, "\rRendering (%d spp) %5.2f%%", samps * 4, 100. * y / (h - 1));
        RANDOM_INIT(Xi);// MILO
        for (unsigned short x = 0; x < w; x++)   // Loop cols
            for (int sy = 0, i = (h - y - 1) * w + x; sy < 2; sy++)     // 2x2 subpixel rows
                for (int sx = 0; sx < 2; sx++, r = vec3_t()) {        // 2x2 subpixel cols
                    for (int s = 0; s < samps; s++) {
                        double r1 = 2 * RANDOM(Xi), dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
                        double r2 = 2 * RANDOM(Xi), dy = r2 < 1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2);
                        vec3_t direction_ = cx * (((sx + .5 + dx) / 2 + x) / w - .5) +
                            cy * (((sy + .5 + dy) / 2 + y) / h - .5) + cam.direction_;
                        r = r + radiance(ray_t(cam.origin_ + direction_ * 140, direction_.normlize()), 0, Xi) * (1. / samps);
                    } // Camera rays are pushed ^^^^^ forward to start in interior
                    color_[i] = color_[i] + vec3_t(clamp(r.x), clamp(r.y), clamp(r.z)) * .25;
                }
    }

    printf("\n%f sec\n", (float)(clock() - start) / CLOCKS_PER_SEC); // MILO

    FILE* file = fopen("image.ppm", "w");         // Write image to PPM file.
    fprintf(file, "P3\n%d %d\n%d\n", w, h, 255);
    for (int i = 0; i < w * h; i++)
        fprintf(file, "%d %d %d ", toInt(color_[i].x), toInt(color_[i].y), toInt(color_[i].z));
    fclose(file);
}