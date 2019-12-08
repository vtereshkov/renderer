// Vasiliy Tereshkov, 2019

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>


struct Vec
  {
  double x, y, z;

  Vec(double x0 = 0, double y0 = 0, double z0 = 0): x(x0), y(y0), z(z0) {}

  Vec operator + (const Vec &v) const {return Vec(x + v.x, y + v.y, z + v.z);}
  Vec operator - (const Vec &v) const {return Vec(x - v.x, y - v.y, z - v.z);}
  Vec operator - () const {return Vec(-x, -y, -z);};
  double operator * (const Vec &v) const {return x * v.x + y * v.y + z * v.z;}
  Vec operator * (double a) const {return Vec(x * a, y * a, z * a);}
  Vec operator / (double a) const {return Vec(x / a, y / a, z / a);}
  Vec operator % (const Vec &v) const {return Vec(x * v.x, y * v.y, z * v.z);}
  };

  
double norm(const Vec &v) {return sqrt(v * v);}
Vec normalize(const Vec &v) {return v / norm(v);}
inline double rnd() {return (double)rand() / (double)RAND_MAX - 0.5;}


typedef Vec Color;


struct Ray
  {
  Vec origin, dir;

  Ray(const Vec &origin0, const Vec &dir0): origin(origin0), dir(dir0) {}
  };


struct Body
  {
  const Vec center;
  const Color color;
  const double diffuseness;
  const bool isLamp;

  Body(const Vec &center0, const Color &color0, double diffuseness0, bool lamp = false):
    center(center0), color(color0), diffuseness(diffuseness0), isLamp(lamp) {};
    
  virtual bool intersect(const Ray &ray, Vec &point, Vec &normal) = 0;
  
  inline double lambertFactor(double lambert) {return 1.0 - (1.0 - lambert) * diffuseness;}  
  };


struct Box: Body
  {
  const Vec halfsize;

  Box(const Vec &center0, const Vec &size0, const Color &color0, double diffuseness0, bool lamp = false):
    Body(center0, color0, diffuseness0, lamp), halfsize(size0 / 2) {};

  inline bool within(double x, double y, double xmin, double ymin, double xmax, double ymax)
    {
    return (x > xmin) && (x < xmax) && (y > ymin) && (y < ymax);
    }

  bool intersect(const Ray &ray, Vec &point, Vec &normal)
    {
    if (fabs(ray.dir.z) > 1e-9)  // xy
      {
      const double side = (ray.dir.z > 0) ? -1.0 : 1.0;
      const double factor = (center.z + side * halfsize.z - ray.origin.z) / ray.dir.z;
      if (factor > 0.1)
        {
        point = ray.origin + ray.dir * factor;
        if (within(point.x, point.y,
                   center.x - halfsize.x, center.y - halfsize.y,
                   center.x + halfsize.x, center.y + halfsize.y))
          {
          normal = Vec(0, 0, side);
          return true;
          }
        }
      }

    if (fabs(ray.dir.x) > 1e-9)  // yz
      {
      const double side = (ray.dir.x > 0) ? -1.0 : 1.0;
      const double factor = (center.x + side * halfsize.x - ray.origin.x) / ray.dir.x;
      if (factor > 0.1)
        {
        point = ray.origin + ray.dir * factor;
        if (within(point.y, point.z,
                   center.y - halfsize.y, center.z - halfsize.z,
                   center.y + halfsize.y, center.z + halfsize.z))
          {
          normal = Vec(side, 0, 0);
          return true;
          }
        }
      }

    if (fabs(ray.dir.y) > 1e-9)  // zx
      {
      const double side = (ray.dir.y > 0) ? -1.0 : 1.0;
      const double factor = (center.y + side * halfsize.y - ray.origin.y) / ray.dir.y;
      if (factor > 0.1)
        {
        point = ray.origin + ray.dir * factor;
        if (within(point.z, point.x,
                   center.z - halfsize.z, center.x - halfsize.x,
                   center.z + halfsize.z, center.x + halfsize.x))
          {
          normal = Vec(0, side, 0);
          return true;
          }
        }
      }

    return false;
    }
  };


struct Sphere: Body
  {
  const double radius;

  Sphere(const Vec &center0, double radius0, const Color &color0, double diffuseness0, bool lamp = false):
    Body(center0, color0, diffuseness0, lamp), radius(radius0) {}

  bool intersect(const Ray &ray, Vec &point, Vec &normal)
    {
    const Vec displacement = center - ray.origin;
    const double proj = displacement * ray.dir;
    const double discr = radius * radius + proj * proj - displacement * displacement;

    if (discr > 0)
      {
      const double factor = proj - sqrt(discr);
      if (factor > 0.1)
        {
        point = ray.origin + ray.dir * factor;
        normal = (point - center) / radius;
        return true;
        }
      }

    return false;  
    }
  };



struct Scene
  {
  const Color ambientColor;

  std::vector <Body *> body;

  Scene(): ambientColor(0.2, 0.2, 0.2) {} 

  Color trace(const Ray &ray, int depth = 0)
    {
    if (depth > 3)
      return ambientColor;
      
    // find nearest intersection
    double bestDist = 1e9;
    int bestIndex = -1;
    Vec bestPoint, bestNormal;

    for (unsigned int b = 0; b < body.size(); b++)
      {
      Vec point, normal;
      if (body[b]->intersect(ray, point, normal))
        {
        double dist = norm(point - ray.origin);
        if (dist < bestDist)
          {
          bestDist = dist;
          bestIndex = b;
          bestPoint = point;
          bestNormal = normal;
          }
        }
      }

    // reflect rays
    if (bestIndex >= 0)
      {
      Body *bestBody = body[bestIndex];
      
      if (bestBody->isLamp)
        return bestBody->color;

      Vec specularDir(ray.dir - bestNormal * (2.0 * (ray.dir * bestNormal))),
          diffuseDir(normalize(specularDir + Vec(rnd(), rnd(), rnd()) * (2.0 * bestBody->diffuseness)));
          
      double lambert = diffuseDir * bestNormal;
      if (lambert < 0) 
        {
        diffuseDir = diffuseDir - bestNormal * (2.0 * lambert);
        lambert = -lambert;
        }
 
      Ray diffuseRay(bestPoint, diffuseDir);

      return (trace(diffuseRay, depth + 1) * bestBody->lambertFactor(lambert)) % bestBody->color;
      }

    return ambientColor;
    }
  };



int main()
  {
  srand(1337);
  
  // define scene
  Scene scene;

  Box box1(Vec(500, -100, 1200),
           Vec(400, 600, 300),
           Color(0.4, 0.7, 1.0),
           0.1);

  scene.body.push_back(&box1);

  Box box2(Vec(550, 210, 1100),
           Vec(1000, 20, 1000),
           Color(0.9, 1.0, 0.6),
           0.3);

  scene.body.push_back(&box2);

  Sphere sphere1(Vec(600, 0, 700),
                 200,
                 Color(1.0, 0.4, 0.6),
                 0.2);

  scene.body.push_back(&sphere1);
  
  Sphere sphere2(Vec(330, 150, 700),
                 50,
                 Color(1.0, 1.0, 0.3),
                 0.15);

  scene.body.push_back(&sphere2);  


  // define light
  Sphere lamp1(Vec(500, -1000, -700),
               800,
               Color(1.0, 1.0, 1.0),
               1.0,
               true);

  scene.body.push_back(&lamp1);


  // define eye
  const Vec pos(0, 0, 0);
  const double azimuth = 30.0 * M_PI / 180.0;
  const double sinAz = sin(azimuth), cosAz = cos(azimuth);

  const int width = 640, height = 480;
  const double focal = 500, antialiasing = 1.0;


  // render scene
  const int rays = 100;
  
  FILE *f = fopen("scene.ppm", "wb");
  fprintf(f, "P6\n%d %d\n255\n", width, height);
  
  for (int i = 0; i < height; i++)
    {
    for (int j = 0; j < width; j++)
      {
      Vec dir(j - width / 2, i - height / 2, focal);

      Vec rotDir( dir.x * cosAz + dir.z * sinAz,
                  dir.y,
                 -dir.x * sinAz + dir.z * cosAz);

      Color color;      
      for (int r = 0; r < rays; r++)
        {
        Vec randomDir(rotDir + Vec(rnd(), rnd(), rnd()) * antialiasing);
        Ray ray(pos, normalize(randomDir));        
        color = color + scene.trace(ray);
        }        
      color = color / rays * 255.0;
  
      putc((int)color.x, f);  putc((int)color.y, f);  putc((int)color.z, f);
      }
      
    printf("%3d / %3d\n", i + 1, height);
    }

  fclose(f);  
  return 0;
  }
