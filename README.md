# ky

[中文讲解](https://infancy.github.io/smallpt2pbrt.html)

single file ray tracing renderer, [100](./smallpt2pbrt/nanopt.cpp)/[300](./smallpt2pbrt/smallpt_comment.cpp)/[1000](./smallpt2pbrt/smallpt_rewrite.cpp)/[4000](./ky.cpp) lines of C++20, PBRT-like architecture, step-by-step integrator.

the name `ky` is meaningless, you can call this project `structured smallpt`, `smallpbrt` etc.

# feature

<!--
ky is based on smallpt at first, gradually rewritten into PBRT style...
-->

`render_debug()`:

![](./docs/images/render_debug.png)

`render_lighting_enum()`:

![](./docs/images/lighting_enum.jpg)

`render_mis_scene()`:

![](./docs/images/veach_mis.jpg)

`render_multiple_scene()`:

![](./docs/images/multi_scene_mis.jpg)



# course&renderer

* Ray Tracing in One Weekend https://raytracing.github.io/books/RayTracingInOneWeekend.html

* Scratchapixel https://www.scratchapixel.com

* Advanced Computer Graphics | RGL http://rgl.epfl.ch/courses/ACG20
  * Nori https://wjakob.github.io/nori/

* Realistic (& Advanced) Image Synthesis http://www.cim.mcgill.ca/~derek/ecse446.html
  * ecsex46/fall-2019: ECSE 446/546 & COMP 598/599: Realistic/Advanced Image Synthesis https://github.com/ecsex46/fall-2019

* COMPUTER GRAPHICS(CMU 15-462/662) http://15462.courses.cs.cmu.edu
  * CMU-Graphics/Scotty3D: Base code for 15-462/662: Computer Graphics at Carnegie Mellon University https://github.com/CMU-Graphics/Scotty3D


# resources

- smallpt: Global Illumination in 99 lines of C++ http://www.kevinbeason.com/smallpt/
    - C# vs C++ 全局照明渲染性能比试 https://www.cnblogs.com/miloyip/archive/2010/06/23/cpp_vs_cs_GI.html <br><br>


- SmallVCM/SmallVCM: A (not too) small physically based renderer that implements the vertex connection and merging algortihm https://github.com/SmallVCM/SmallVCM

- ouj/safegi: implementation of safegi: an rendering system with dimensional analysis. https://github.com/ouj/safegi

- hi2p-perim/minpt: A path tracer in 300 lines of C++ https://github.com/hi2p-perim/minpt <br><br>


- xelatihy/yocto-gl: Yocto/GL: Tiny C++ Libraries for Data-Driven Physically-based Graphics https://github.com/xelatihy/yocto-gl

- mmp/pbrt-v3: Source code for pbrt, the renderer described in the third edition of "Physically Based Rendering: From Theory To Implementation", by Matt Pharr, Wenzel Jakob, and Greg Humphreys. https://github.com/mmp/pbrt-v3

- mmp/pbrt-v4: Source code to pbrt, the ray tracer described in the forthcoming 4th edition of the "Physically Based Rendering: From Theory to Implementation" book. https://github.com/mmp/pbrt-v4

- mitsuba-renderer/mitsuba3: Mitsuba 3: A Retargetable Forward and Inverse Renderer https://github.com/mitsuba-renderer/mitsuba3

- LuisaGroup/LuisaRender: High-Performance Cross-Platform Monte Carlo Renderer Based on LuisaCompute https://github.com/LuisaGroup/LuisaRender <br><br>


- Source/Render/Cycles - Blender Developer Wiki https://wiki.blender.org/wiki/Source/Render/Cycles



# milestone

- [x] run smallpt

- [x] rewrite smallpt
  - [x] geometry
  - [x] shape
    - [x] disk
    - [x] triangle
    - [x] rectangle
    - [x] sphere
  - [x] scene
  - [x] camera
  - [x] material
  - [x] light
  - [x] integrator
  - [ ] cmd

- [ ] step-by-step integrator 
  - [x] render depth, normal
  - [ ] render point light
  - [ ] render spuclar material
  - [x] render direct lighting(sampling brdf, sampling lighting)
  - [x] MIS
  - [x] recursion style path tracing
  - [x] iterative style path tracing

- [x] bsdf/material
  - [x] Phong
- [x] scene
  - [x] mis scene

<!--
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

-->