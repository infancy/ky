# ky

!- [ ]()

single file pathtracing renderer, 2000 lines of C++20, PBRT-like architecture, step-by-step integrator



## feature

ky is based on smallpt at first, gradually rewritten into a PBRT style



## milestone

- [x] run smallpt
- [ ] rewrite smallpt
  - [x] geometry
  - [ ] shape
    - [ ] triangle
    - [x] sphere
  - [x] scene
  - [x] camera
  - [x] material
  - [x] light
  - [ ] integrator
  - [ ] cmd
- [ ] step-by-step integrator 
  - [ ] render depth, normal
  - [ ] render point light
  - [ ] render spuclar material
  - [ ] render direct lighting(sampling brdf, sampling lighting)
  - [ ] MIS
  - [ ] recursion style pathtracing
  - [ ] iterative style pathtracing
<br>
- [ ] bsdf/material
  - [ ] Phong
- [ ] scene
  - [ ] mis scene
<br>
- [ ] unity support
- [ ] web support



## kys

- [ ] direct lighting
- [ ] lambert/specular BRDF
- [ ] .ppm



## kye

- [ ] color_t -> spectrum_t
- [ ] matrix_t
- [ ] CUDA support



## credits

* smallpt: Global Illumination in 99 lines of C++ http://www.kevinbeason.com/smallpt/

* C# vs C++ 全局照明渲染性能比试 - Milo Yip - 博客园 https://www.cnblogs.com/miloyip/archive/2010/06/23/cpp_vs_cs_GI.html

<br>

* Scratchapixel https://www.scratchapixel.com/index.php?redirect

<br>

* https://github.com/SmallVCM/SmallVCM

* ouj/safegi: implementation of safegi: an rendering system with dimensional analysis. https://github.com/ouj/safegi

* hi2p-perim/minpt: A path tracer in 300 lines of C++ https://github.com/hi2p-perim/minpt

<br>

* mmp/pbrt-v3: Source code for pbrt, the renderer described in the third edition of "Physically Based Rendering: From Theory To Implementation", by Matt Pharr, Wenzel Jakob, and Greg Humphreys. https://github.com/mmp/pbrt-v3

* https://github.com/mitsuba-renderer/mitsuba2

* xelatihy/yocto-gl: Yocto/GL: Tiny C++ Libraries for Data-Driven Physically-based Graphics https://github.com/xelatihy/yocto-gl

