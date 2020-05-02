
#pragma once

#include "paths.hpp"

#include <atlas/core/Float.hpp>
#include <atlas/math/Math.hpp>
#include <atlas/math/Random.hpp>
#include <atlas/math/Ray.hpp>
#include <atlas/core/Timer.hpp>

#include <fmt/printf.h>
#include <stb_image.h>
#include <stb_image_write.h>

#include <thread>
#include <future>

#include <limits>

#include <math.h> 
#include <iostream>
#include <fstream>
#include <sstream>


using namespace atlas;
using atlas::core::areEqual;

using Colour = atlas::math::Vector;

void saveToFile(std::string const& filename,
    std::size_t width,
    std::size_t height,
    std::vector<Colour> const& image);


// Declarations
class BRDF;
class Camera;
class Material;
class Light;
class Shape;
class Sampler;
class Texture;
class Mapping;
class Image;
class RandomNoise; 
class Mesh; 

struct World
{
    std::size_t width, height;
    Colour background;
    std::shared_ptr<Sampler> sampler;
    std::vector<std::shared_ptr<Shape>> scene;
    std::vector<Colour> image;
    std::vector<std::shared_ptr<Light>> lights;
    std::shared_ptr<Light> ambient;
};

struct ShadeRec
{
    Colour color;
    float t;
    atlas::math::Normal normal;
    atlas::math::Ray<atlas::math::Vector> ray;
    std::shared_ptr<Material> material;
    std::shared_ptr<World> world;
};

struct Slab
{
    size_t startindex; 
    size_t endindex; 
};

// Abstract classes defining the interfaces for concrete entities

class Sampler
{
public:
    Sampler(int numSamples, int numSets);
    virtual ~Sampler() = default;

    int getNumSamples() const;

    void setupShuffledIndeces();

    virtual void generateSamples() = 0;

    atlas::math::Point sampleUnitSquare();

protected:
    std::vector<atlas::math::Point> mSamples;
    std::vector<int> mShuffledIndeces;

    int mNumSamples;
    int mNumSets;
    unsigned long mCount;
    int mJump;
};

class BRDF
{
public:
     virtual ~BRDF() = default;

    virtual Colour fn(ShadeRec const& sr,
        atlas::math::Vector const& reflected,
        atlas::math::Vector const& incoming) const = 0;
    virtual Colour rho(ShadeRec const& sr,
        atlas::math::Vector const& reflected) const = 0;
};

class Material
{
public:
    virtual ~Material() = default;

    virtual Colour shade(ShadeRec& sr) = 0;
};

// Concrete classes which we can construct and use in our ray tracer

class Shape
{
public:
    Shape();
    virtual ~Shape() = default;

    // if t computed is less than the t in sr, it and the color should be
    // updated in sr
    virtual bool hit(atlas::math::Ray<atlas::math::Vector> const& ray,
        ShadeRec& sr) const = 0;

    void setColour(Colour const& col);

    Colour getColour() const;

    void setMaterial(std::shared_ptr<Material> const& material);

    std::shared_ptr<Material> getMaterial() const;

    virtual bool intersectRay(atlas::math::Ray<atlas::math::Vector> const& ray,
        float& tMin) const = 0;

protected:
    

    Colour mColour;
    std::shared_ptr<Material> mMaterial;
};

class Sphere : public Shape
{
public:
    Sphere(atlas::math::Point center, float radius);

    bool hit(atlas::math::Ray<atlas::math::Vector> const& ray,
        ShadeRec& sr) const;

    bool intersectRay(atlas::math::Ray<atlas::math::Vector> const& ray,
        float& tMin) const;
private:
   
    atlas::math::Point mCentre;
    float mRadius;
    float mRadiusSqr;
};

class Triangle : public Shape
{
public:
	Triangle(math::Point v0, math::Point v1, math::Point v2);

	bool hit(atlas::math::Ray<atlas::math::Vector> const& ray,
		ShadeRec& sr) const;

	bool intersectRay(atlas::math::Ray<atlas::math::Vector> const& ray,
		float& tMin) const;
	
	math::Point v0;
	math::Point v1;
	math::Point v2;
	math::Vector n;
};

class Plane : public Shape
{
public: 
    Plane(atlas::math::Vector n, atlas::math::Point p);

    bool hit(atlas::math::Ray<atlas::math::Vector> const& ray,
        ShadeRec& sr) const;

    bool intersectRay(atlas::math::Ray<atlas::math::Vector> const& ray,
        float& tMin) const;


private: 
    
    atlas::math::Vector n;
    atlas::math::Point p;
};

class Regular : public Sampler {
public:
    Regular(int numSamples, int numSets);

    void generateSamples();
};

class Random : public Sampler
{
public:
    Random(int numSamples, int numSets);

    void generateSamples();
};


class Lambertian : public BRDF
{
public:
    Lambertian();
    Lambertian(Colour diffuseColor, float diffuseReflection);

    Colour fn(ShadeRec const& sr,
        atlas::math::Vector const& reflected,
        atlas::math::Vector const& incoming) const override;

    Colour rho(ShadeRec const& sr,
        atlas::math::Vector const& reflected) const override;

    void setDiffuseReflection(float kd);

    void setDiffuseColour(Colour const& colour);

private:
    Colour mDiffuseColour;
    float mDiffuseReflection;
};

class SV_Lambertian : public BRDF
{
public:
    SV_Lambertian(); 
    SV_Lambertian(Colour diffuseColor, float diffuseReflection);

    Colour rho(ShadeRec const& sr,
        atlas::math::Vector const& reflected) const override;

    Colour fn(ShadeRec const& sr,
        atlas::math::Vector const& reflected,
        atlas::math::Vector const& incoming) const override;

    void setDiffuseColour(Colour const& colour);
    void setDiffuseReflection(float kd);
    void setTexture(std::shared_ptr<Texture> cd); 
    std::shared_ptr<Texture> getTexture() const;

private: 
    Colour mDiffuseColour;
    float mDiffuseReflection;
    std::shared_ptr<Texture> mTexture;
};

class GlossySpecular : public BRDF
{
public:
    GlossySpecular(); 
    GlossySpecular(Colour cs, float ks, float exp);

    Colour fn(ShadeRec const& sr,
        atlas::math::Vector const& reflected,
        atlas::math::Vector const& incoming) const override;

    Colour rho(ShadeRec const& sr,
        atlas::math::Vector const& reflected) const override;

    void setGlossyReflection(float ks);
    void setGlossyColour(Colour const& colour);
    void setExp(float exp); 

private:
    Colour cs; 
    float ks; 
    float exp; 
};

class Matte : public Material
{
public:
    Matte();
    Matte(float kd, float ka, Colour color);
    void setDiffuseReflection(float k);
    void setAmbientReflection(float k);
    void setDiffuseColour(Colour colour);
    Colour shade(ShadeRec& sr) override;

private:
    std::shared_ptr<Lambertian> mDiffuseBRDF;
    std::shared_ptr<Lambertian> mAmbientBRDF;
};

class SV_Matte : public Material
{
public:
    SV_Matte();
    SV_Matte(float kd, float ka, std::shared_ptr<Texture> cd);
    void setDiffuseReflection(float k);
    void setAmbientReflection(float k);
    void setTexture(std::shared_ptr<Texture> cd); 
    Colour shade(ShadeRec& sr) override;

private:
    std::shared_ptr<SV_Lambertian> mDiffuseBRDF;
    std::shared_ptr<SV_Lambertian> mAmbientBRDF;
};

class Phong : public Material
{
public: 
    Phong();
    Phong(float kd, float ka, float ks, float exp, Colour color);
    void setDiffuseReflection(float k);
    void setAmbientReflection(float k);
    void setGlossyReflection(float k, float exp); 
    void setDiffuseColour(Colour colour);
    Colour shade(ShadeRec& sr) override;
private:
    std::shared_ptr<Lambertian> mDiffuseBRDF;
    std::shared_ptr<Lambertian> mAmbientBRDF; 
    std::shared_ptr<GlossySpecular> mGlossyBRDF; 
};

class Light
{
public:
    virtual atlas::math::Vector getDirection(ShadeRec& sr) = 0;

    virtual Colour L(ShadeRec& sr);

    virtual bool in_shadow(ShadeRec& sr, atlas::math::Ray<atlas::math::Vector> const& ray) = 0;

    void scaleRadiance(float b);

    void setColour(Colour const& c);

    void setCastsShadows(bool shadows); 
    bool getCastsShadows(); 
protected:
    bool casts_shadows; 
    Colour mColour;
    float mRadiance;
};

class Directional : public Light
{
public:
    Directional();
    Directional(atlas::math::Vector const& d);

    void setDirection(atlas::math::Vector const& d);

    atlas::math::Vector getDirection(ShadeRec& sr) override;
    bool in_shadow(ShadeRec& sr, atlas::math::Ray<atlas::math::Vector> const& ray) override;

private:
    atlas::math::Vector mDirection;
};

class PointLight : public Light
{
public: 
    PointLight(atlas::math::Point const& p);
    void setPoint(atlas::math::Point const& p); 
    atlas::math::Vector getDirection(ShadeRec& sr) override;
    bool in_shadow(ShadeRec& sr, atlas::math::Ray<atlas::math::Vector> const& ray) override;
    
private:
    atlas::math::Point mPoint; 
};

class Ambient : public Light
{
public:
    Ambient();

    atlas::math::Vector getDirection(ShadeRec& sr) override;
    bool in_shadow(ShadeRec& sr, atlas::math::Ray<atlas::math::Vector> const& ray) override;

private:
    atlas::math::Vector mDirection;
};

// Abstract classes defining the interfaces for concrete entities

class Camera
{
public:
    Camera();

    virtual ~Camera() = default;

    virtual void renderScene(std::shared_ptr<World> world) const = 0;

    void setEye(atlas::math::Point const& eye);

    void setLookAt(atlas::math::Point const& lookAt);

    void setUpVector(atlas::math::Vector const& up);

    void computeUVW();

protected:
    atlas::math::Point mEye;
    atlas::math::Point mLookAt;
    atlas::math::Point mUp;
    atlas::math::Vector mU, mV, mW;
};

class Pinhole : public Camera
{
public:
    Pinhole();

    void setDistance(float distance);
    void setZoom(float zoom);

    atlas::math::Vector rayDirection(atlas::math::Point const& p) const;
    void renderScene(std::shared_ptr<World> world) const;
    void renderSceneMulti(std::shared_ptr<World> world) const; 
    void renderSceneMultiIntermediate(std::shared_ptr<World> world, size_t const numOfSlabs) const;

private:
    float mDistance;
    float mZoom;
};

class Texture 
{
public:

    virtual ~Texture() = default;

    virtual Colour getColour(const ShadeRec& sr) const = 0; 

};

class ConstantColour : public Texture
{
public:
    ConstantColour(Colour const & col);

    void setColour(Colour const& col);
    Colour getColour(const ShadeRec& sr) const override;

private: 
    Colour mColour; 
};

class ImageTexture : public Texture
{
public:
    ImageTexture(
        std::shared_ptr<Image> image_ptr,
        std::shared_ptr<Mapping> mapping_ptr
    );
    Colour getColour(const ShadeRec& sr) const override; 
    std::vector<Colour> getImage() const; 

private:
    std::shared_ptr<Image> image_ptr; 
    std::shared_ptr<Mapping> mapping_ptr; 
};

class Checker : public Texture
{
public:
    Checker(float size, Colour colourA, Colour colourB);
    Colour getColour(const ShadeRec& sr) const override;
private:
    float size; 
    Colour colourA; 
    Colour colourB; 
};

class RandomNoiseTexture : public Texture
{
public: 
    RandomNoiseTexture();
    Colour getColour(const ShadeRec& sr) const override; 
};

class Mapping
{
public:
    virtual ~Mapping() = default; 

    virtual atlas::math::Vector2 getTexelCoordinates(
        atlas::math::Vector localHitPoint
    ) const = 0; 
};

class SphericalMap : public Mapping
{
public:

    SphericalMap(atlas::math::Point);

    atlas::math::Vector2 getTexelCoordinates(
        atlas::math::Vector localHitPoint
    ) const override; 

    atlas::math::Point getOrigin() const; 

private: 
    atlas::math::Point origin; 
};

class Image
{
public:
    Image(std::string filename);

    Colour getColour(float u, float v); 
    size_t getWidth(); 
    size_t getHeight(); 

    std::vector<Colour> getImage(); 

private: 
    std::vector<Colour> image;
    std::string const& filename;
    size_t width;
    size_t height; 
};

class Mesh
{
public:
    Mesh(std::string filename, size_t scalingFactor);
    std::vector <Triangle> triangleList; 
private: 
    size_t numVertex; 
    size_t numFace; 
    size_t scalingFactor; 
};