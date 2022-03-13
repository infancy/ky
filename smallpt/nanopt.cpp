 // http://www.kevinbeason.com/smallpt
 #include "erand48.h"// pseudo-random number generator, not in <stdlib.h>
 #include <math.h>   // smallpt, a Path Tracer by Kevin Beason, 2008 
 #include <stdlib.h> // Make : g++ -O3 -fopenmp smallpt.cpp -o smallpt 
 #include <stdio.h>  //        Remove "-fopenmp" for g++ version < 4.2 
 
 #define M_PI 3.141592653589793238462643
 
 struct Vec {        // Usage: time ./smallpt 5000 && xv image.ppm 
   double x, y, z;                  // position, also color (r,g,b) 
   Vec(double x_=0, double y_=0, double z_=0){ x=x_; y=y_; z=z_; } 
  
   Vec operator+(const Vec &b) const { return Vec(x+b.x,y+b.y,z+b.z); } 
   Vec operator-(const Vec &b) const { return Vec(x-b.x,y-b.y,z-b.z); } 
   Vec operator*(double b) const { return Vec(x*b,y*b,z*b); } 
   Vec mult(const Vec &b) const { return Vec(x*b.x,y*b.y,z*b.z); } 
   Vec& norm(){ return *this = *this * (1/sqrt(x*x+y*y+z*z)); } 
   double dot(const Vec &b) const { return x*b.x+y*b.y+z*b.z; } // cross: 
   Vec operator%(Vec&b){return Vec(y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x);} 
 }; 
  
 struct Ray { Vec o, d; Ray(Vec o_, Vec d_) : o(o_), d(d_) {} }; 
  
 enum Refl_t { DIFF, SPEC };  // material types, used in radiance() 
  
 struct Sphere { 
   double rad;       // radius 
   Vec p, e, c;      // position, emission, color 
   Refl_t refl;      // reflection type (DIFFuse, SPECular, REFRactive) 
   Sphere(double rad_, Vec p_, Vec e_, Vec c_, Refl_t refl_): 
     rad(rad_), p(p_), e(e_), c(c_), refl(refl_) {} 
  
   double intersect(const Ray &r) const { // returns distance, 0 if nohit 
     Vec op = p-r.o; // Solve t^2*d.d + 2*t*(o-p).d + (o-p).(o-p)-R^2 = 0 
     double t, eps=1e-4, b=op.dot(r.d), det=b*b-op.dot(op)+rad*rad; 
     if (det<0) return 0; else det=sqrt(det); 
     return (t=b-det)>eps ? t : ((t=b+det)>eps ? t : 0); 
   } 
 }; 
  
 Sphere spheres[] = {//Scene: radius, position, emission, color, material 
   Sphere(1e5, Vec( 1e5+1,40.8,81.6), Vec(),Vec(.75,.25,.25),DIFF),//Left 
   Sphere(1e5, Vec(-1e5+99,40.8,81.6),Vec(),Vec(.25,.25,.75),DIFF),//Rght 
   Sphere(1e5, Vec(50,40.8, 1e5),     Vec(),Vec(.75,.75,.75),DIFF),//Back 
   Sphere(1e5, Vec(50,40.8,-1e5+170), Vec(),Vec(),           DIFF),//Frnt 
   Sphere(1e5, Vec(50, 1e5, 81.6),    Vec(),Vec(.75,.75,.75),DIFF),//Botm 
   Sphere(1e5, Vec(50,-1e5+81.6,81.6),Vec(),Vec(.75,.75,.75),DIFF),//Top 
   Sphere(16.5,Vec(50,16.5,62),       Vec(),Vec(1,1,1)*.999, SPEC),//Mirr 
   Sphere(600, Vec(50,681.6-.27,81.6),Vec(12,12,12),  Vec(), DIFF) //Lite 
 }; 
  
 inline double clamp(double x){ return x<0 ? 0 : x>1 ? 1 : x; } 
 inline int toInt(double x){ return int(pow(clamp(x),1/2.2)*255+.5); } 
  
 inline bool intersect(const Ray &r, double &t, int &id){ 
   double n=sizeof(spheres)/sizeof(Sphere), d, inf=t=1e20; 
   for(int i=int(n);i--;) if((d=spheres[i].intersect(r))&&d<t){t=d;id=i;} 
   return t<inf; 
 } 
  
 Vec radiance(const Ray &r, int depth, unsigned short *Xi){ 
   double t;                               // distance to intersection 
   int id=0;                               // id of intersected object 
   if (!intersect(r, t, id)) return Vec(); // if miss, return black 
  
   const Sphere &obj = spheres[id];        // the hit object 
   if (depth > 10) return obj.e;

   Vec x=r.o+r.d*t, n=(x-obj.p).norm(), nl=n.dot(r.d)<0?n:n*-1, f=obj.c; 
   double p = f.x>f.y && f.x>f.z ? f.x : f.y>f.z ? f.y : f.z; // max refl 
   if (++depth>5) if (erand48(Xi)<p) f=f*(1/p); else return obj.e; //R.R. 
  
   if (obj.refl == DIFF){                  // Ideal DIFFUSE reflection 
     double r1=2*M_PI*erand48(Xi), r2=erand48(Xi), r2s=sqrt(r2); 
     Vec w=nl, u=((fabs(w.x)>.1?Vec(0,1):Vec(1))%w).norm(), v=w%u; 
     Vec d = (u*cos(r1)*r2s + v*sin(r1)*r2s + w*sqrt(1-r2)).norm(); 
     return obj.e + f.mult(radiance(Ray(x,d),depth,Xi)); 
  
   } else if (obj.refl == SPEC)            // Ideal SPECULAR reflection 
     return obj.e + f.mult(radiance(Ray(x,r.d-n*2*n.dot(r.d)),depth,Xi)); 
 } 
  
 int main(int argc, char *argv[]){ 
   int w=1024, h=768, samps = argc==2 ? atoi(argv[1])/4 : 40;  // samples 
   Ray cam(Vec(50,52,295.6), Vec(0,-0.042612,-1).norm()); // cam pos, dir 
   Vec cx=Vec(w*.5135/h), cy=(cx%cam.d).norm()*.5135, r, *c=new Vec[w*h]; 
  
 #pragma omp parallel for schedule(dynamic, 1) private(r)       // OpenMP 
   for (int y=0; y<h; y++){                       // Loop over image rows 
     fprintf(stderr,"\rRendering (%d spp) %5.2f%%",samps*4,100.*y/(h-1)); 
     unsigned short Xi[3] = { 0,0,y * y * y };
     for (unsigned short x=0; x<w; x++){   // Loop cols 
       int i=(h-y-1)*w+x;
       r = Vec();

       for (int s=0; s<samps; s++){ 
         Vec d = cx*( (erand48(Xi) + x)/w - .5) +
                 cy*( (erand48(Xi) + y)/h - .5) + cam.d;

         r = r + radiance(Ray(cam.o+d*140,d.norm()),0,Xi)*(1./samps); 
       } // Camera rays are pushed ^^^^^ forward to start in interior 
       c[i] = c[i] + Vec(clamp(r.x),clamp(r.y),clamp(r.z))*.25; 
     } 
   } 
  
   FILE *f = fopen("image.ppm", "w");         // Write image to PPM file. 
   fprintf(f, "P3\n%d %d\n%d\n", w, h, 255); 
   for (int i=0; i<w*h; i++) 
     fprintf(f,"%d %d %d ", toInt(c[i].x), toInt(c[i].y), toInt(c[i].z)); 
 }