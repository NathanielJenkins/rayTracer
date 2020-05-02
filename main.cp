#include "assignment.hpp"
using namespace std; 

// ******* Function Member Implementation *******

// ***** Shape function members *****
Shape::Shape() : mColour{ 0, 0, 0 }
{}

void Shape::setColour(Colour const& col)
{
    mColour = col;
}

Colour Shape::getColour() const
{
    return mColour;
}

void Shape::setMaterial(std::shared_ptr<Material> const& material)
{
    mMaterial = material;
}

std::shared_ptr<Material> Shape::getMaterial() const
{
    return mMaterial;
}

// ***** Sampler function members *****
Sampler::Sampler(int numSamples, int numSets) :
    mNumSamples{ numSamples }, mNumSets{ numSets }, mCount{ 0 }, mJump{ 0 }
{
    mSamples.reserve(mNumSets* mNumSamples);
    setupShuffledIndeces();
}

int Sampler::getNumSamples() const
{
    return mNumSamples;
}

void Sampler::setupShuffledIndeces()
{
    mShuffledIndeces.reserve(mNumSamples * mNumSets);
    std::vector<int> indices;

    std::random_device d;
    std::mt19937 generator(d());

    for (int j = 0; j < mNumSamples; ++j)
    {
        indices.push_back(j);
    }

    for (int p = 0; p < mNumSets; ++p)
    {
        std::shuffle(indices.begin(), indices.end(), generator);

        for (int j = 0; j < mNumSamples; ++j)
        {
            mShuffledIndeces.push_back(indices[j]);
        }
    }
}

atlas::math::Point Sampler::sampleUnitSquare()
{
    if (mCount % mNumSamples == 0)
    {
        atlas::math::Random<int> engine;
        mJump = (engine.getRandomMax() % mNumSets) * mNumSamples;
    }

    return mSamples[mJump + mShuffledIndeces[mJump + mCount++ % mNumSamples]];
}

// ***** Sphere function members *****
Sphere::Sphere(atlas::math::Point center, float radius) :
    mCentre{ center }, mRadius{ radius }, mRadiusSqr{ radius * radius }
{}

bool Sphere::hit(atlas::math::Ray<atlas::math::Vector> const& ray,
    ShadeRec& sr) const
{
    atlas::math::Vector tmp = ray.o - mCentre;
    float t{ std::numeric_limits<float>::max() };
    bool intersect{ intersectRay(ray, t) };

    // update ShadeRec info about new closest hit
    if (intersect && t < sr.t)
    {
        sr.normal = (tmp + t * ray.d) / mRadius;
        sr.ray = ray;
        sr.color = mColour;
        sr.t = t;
        sr.material = mMaterial;
    }

    return intersect;
}

bool Sphere::intersectRay(atlas::math::Ray<atlas::math::Vector> const& ray,
    float& tMin) const
{
    const auto tmp{ ray.o - mCentre };
    const auto a{ glm::dot(ray.d, ray.d) };
    const auto b{ 2.0f * glm::dot(ray.d, tmp) };
    const auto c{ glm::dot(tmp, tmp) - mRadiusSqr };
    const auto disc{ (b * b) - (4.0f * a * c) };

    if (atlas::core::geq(disc, 0.0f))
    {
        const float kEpsilon{ 0.01f };
        const float e{ std::sqrt(disc) };
        const float denom{ 2.0f * a };

        // Look at the negative root first
        float t = (-b - e) / denom;
        if (atlas::core::geq(t, kEpsilon))
        {
            tMin = t;
            return true;
        }

        // Now the positive root
        t = (-b + e);
        if (atlas::core::geq(t, kEpsilon))
        {
            tMin = t;
            return true;
        }
    }

    return false;
}


// **** Plane function members *****
Plane::Plane(atlas::math::Vector n, atlas::math::Point p) :
    n{ n },
    p{ p }
{}

bool Plane::hit(atlas::math::Ray<atlas::math::Vector> const& ray,
    ShadeRec& sr) const {

    float t{ std::numeric_limits<float>::max() };
    bool intersect{ intersectRay(ray, t) };

    if (intersect && t < sr.t)
    {
        sr.normal = n;
        sr.ray = ray;
        sr.color = mColour;
        sr.t = t;
        sr.material = mMaterial;
    }

    return intersect;
}

bool Plane::intersectRay(atlas::math::Ray<atlas::math::Vector> const& ray,
    float& tMin) const
{

    const float kEpsilon{ 0.01f };
    float t = (glm::dot((p - ray.o), n)) / (glm::dot(ray.d, n));

    if (atlas::core::geq(t, kEpsilon)) {
        tMin = t;
        return true;
    }
    return false;
}

Triangle::Triangle(math::Point v0, math::Point v1, math::Point v2) : 
	v0{ v0 },
	v1{ v1 },
	v2{ v2 }
	
{
    atlas::math::Vector v = v1 - v0;
    atlas::math::Vector w = v2 - v0;
    n = -1.0f * glm::normalize(glm::cross(v, w)); 
}

bool Triangle::hit(atlas::math::Ray<atlas::math::Vector> const& ray,
	ShadeRec& sr) const {

	float t{ std::numeric_limits<float>::max() };
	bool intersect{ intersectRay(ray, t) };
    const float kEpsilon{ 0.01f };


	if (intersect && t < sr.t)
	{
		sr.normal = n;
		sr.ray = ray;
		sr.color = mColour;
		sr.t = t;
		sr.material = mMaterial;
	}

	return intersect;
}


bool Triangle::intersectRay(atlas::math::Ray<atlas::math::Vector> const& r,
	float& tMin) const
{
	
	//code taken from ray tracing from ground up chapter 19
	float a = v0.x - v1.x, b = v0.x - v2.x, c = r.d.x, d = v0.x - r.o.x;
	float e = v0.y - v1.y, f = v0.y - v2.y, g = r.d.y, h = v0.y - r.o.y;
	float i = v0.z - v1.z, j = v0.z - v2.z, k = r.d.z, l = v0.z - r.o.z;

	float m = f * k - g * j, n1 = h * k - g * l, p = f * l - h * j;
	float q = g * i - e * k, s = e * j - f * i;

	float inv_denom = 1.0f / (a * m + b * q + c * s);
    
	float e1 = d * m - b * n1 - c * p;
	float beta = e1 * inv_denom;

	float r1 = e * l - h * i;
	float e2 = a * n1 + d * q + c * r1;
	float gamma = e2 * inv_denom;

	if (beta < 0.0f || gamma < 0.0f || beta + gamma > 1.0f)
		return false;

	float e3 = a * p - b * r1 + d * s;
	float t = e3 * inv_denom;

	//calculate t
    const float kEpsilon{ 0.01f };
    if (atlas::core::geq(t, kEpsilon)) {
        tMin = t;
        return true;
    }
    return false;
}

Mesh::Mesh(std::string filename, size_t scalingFactor) : numVertex{ 0 }, numFace{ 0 }, scalingFactor (scalingFactor)
{

    /* Read in all the information from ply file */
    string path = ShaderPath + filename;
    ifstream meshFile;
    meshFile.open(path);

    string line;

    while (getline(meshFile, line)) {

        if (line.substr(0, 12) == "element vert") numVertex = (size_t)line.back() - 48;

        if (line.substr(0, 12) == "element face") numFace = (size_t)line.back() - 48;

        if (line == "end_header") break;
    }

    std::vector < atlas::math::Point > vertexList;
    for (size_t i{ 0 }; i < numVertex; ++i) {
        getline(meshFile, line);

        stringstream ss(line);
        std::vector <int> point;
        for (int j{ 0 }; j < 3; ++j) {
            int attr;
            ss >> attr;
            point.push_back(attr);
            if (ss.peek() == ' ') ss.ignore();
        }

        vertexList.push_back(atlas::math::Point{ point[0], point[1], point[2] });
    }

    for (size_t i{ 0 }; i < numFace; ++i) { //for each face
        getline(meshFile, line);

        stringstream ss(line);
        size_t numOfVertexInFace;
        ss >> numOfVertexInFace;

        //read in the indexes into a vector
        std::vector <size_t> faceIndexVec;
        for (size_t j{ 0 }; j < numOfVertexInFace; ++j) {
            int faceIndex;
            ss >> faceIndex;
            faceIndexVec.push_back(faceIndex);
            if (ss.peek() == ' ') ss.ignore();

        }

        for (size_t j{ 0 }; j < numOfVertexInFace - 2; j++) {
            //create a triangle and add to the list
            atlas::math::Point v0 = 100.0f * vertexList.at(faceIndexVec.at(0));
            atlas::math::Point v1 = 100.0f * vertexList.at(faceIndexVec.at(j + 1));
            atlas::math::Point v2 = 100.0f * vertexList.at(faceIndexVec.at(j + 2));
            triangleList.push_back(Triangle{ v0, v1, v2 });
        }
    }
}

// ***** Regular function members *****
Regular::Regular(int numSamples, int numSets) : Sampler{ numSamples, numSets }
{
    generateSamples();
}

void Regular::generateSamples()
{
    int n = static_cast<int>(glm::sqrt(static_cast<float>(mNumSamples)));

    for (int j = 0; j < mNumSets; ++j)
    {
        for (int p = 0; p < n; ++p)
        {
            for (int q = 0; q < n; ++q)
            {
                mSamples.push_back(
                    atlas::math::Point{ (q + 0.5f) / n, (p + 0.5f) / n, 0.0f });
            }
        }
    }
}

// ***** Regular function members *****
Random::Random(int numSamples, int numSets) : Sampler{ numSamples, numSets }
{
    generateSamples();
}

void Random::generateSamples()
{
    atlas::math::Random<float> engine;
    for (int p = 0; p < mNumSets; ++p)
    {
        for (int q = 0; q < mNumSamples; ++q)
        {
            mSamples.push_back(atlas::math::Point{
                engine.getRandomOne(), engine.getRandomOne(), 0.0f });
        }
    }
}



// ***** Lambertian function members *****
Lambertian::Lambertian() : mDiffuseColour{}, mDiffuseReflection{}
{}

Lambertian::Lambertian(Colour diffuseColor, float diffuseReflection) :
    mDiffuseColour{ diffuseColor }, mDiffuseReflection{ diffuseReflection }
{}

Colour
Lambertian::fn([[maybe_unused]] ShadeRec const& sr,
    [[maybe_unused]] atlas::math::Vector const& reflected,
    [[maybe_unused]] atlas::math::Vector const& incoming) const
{
    return mDiffuseColour * mDiffuseReflection * glm::one_over_pi<float>();
}

Colour
Lambertian::rho([[maybe_unused]] ShadeRec const& sr,
    [[maybe_unused]] atlas::math::Vector const& reflected) const
{
    return mDiffuseColour * mDiffuseReflection;
}

void Lambertian::setDiffuseReflection(float kd)
{
    mDiffuseReflection = kd;
}

void Lambertian::setDiffuseColour(Colour const& colour)
{
    mDiffuseColour = colour;
}


//** SV_Lambertian function members **
SV_Lambertian::SV_Lambertian() : mDiffuseColour{}, mDiffuseReflection{}
{}

SV_Lambertian::SV_Lambertian(Colour diffuseColor, float diffuseReflection) :
    mDiffuseColour{ diffuseColor }, mDiffuseReflection{ diffuseReflection }
{}


void SV_Lambertian::setDiffuseColour(Colour const& colour)
{
    mDiffuseColour = colour;
}

void SV_Lambertian::setDiffuseReflection(float kd)
{
    mDiffuseReflection = kd;
}

void SV_Lambertian::setTexture(std::shared_ptr<Texture> cd)
{

    mTexture = cd; 
}

std::shared_ptr<Texture> SV_Lambertian::getTexture() const
{
    return mTexture;
}


Colour
SV_Lambertian::rho([[maybe_unused]] ShadeRec const& sr,
    [[maybe_unused]] atlas::math::Vector const& reflected) const
{
    return mDiffuseReflection * mTexture->getColour(sr);
}

Colour SV_Lambertian::fn(
    [[maybe_unused]] ShadeRec const& sr,
    [[maybe_unused]] atlas::math::Vector const& reflected,
    [[maybe_unused]] atlas::math::Vector const& incoming) const
{
    return mDiffuseReflection * mTexture->getColour(sr) * glm::one_over_pi<float>();
}



//**** Glossy Specular Function members ****

GlossySpecular::GlossySpecular() : cs{}, ks{}, exp{}
{}

GlossySpecular::GlossySpecular(Colour cs, float ks, float exp) : 
    cs{cs}, ks{ks}, exp{exp}
{}

void GlossySpecular::setGlossyReflection(float mKs)
{
    ks = mKs;
}

void GlossySpecular::setGlossyColour([[maybe_unused]] Colour const& colour)
{
    cs = colour;
}

void GlossySpecular::setExp(float mExp) 
{
    exp = mExp; 
}

Colour
GlossySpecular::fn([[maybe_unused]] ShadeRec const& sr,
    [[maybe_unused]] atlas::math::Vector const& reflected,
    [[maybe_unused]] atlas::math::Vector const& incoming) const
{
    Colour L = Colour{ 0.0f };
    float ndotincoming = glm::dot(sr.normal, incoming);
    atlas::math::Vector r = -incoming + 2.0f * sr.normal * ndotincoming;
    float rdotreflected = glm::dot (r, reflected);


    if (rdotreflected > 0.0f) 
    {
        L = ks * pow(rdotreflected, exp) * cs; 
    }
    return L;
}


Colour
GlossySpecular::rho([[maybe_unused]] ShadeRec const& sr,
    [[maybe_unused]] atlas::math::Vector const& reflected) const
{
    return cs * ks;
}

//**** Phong function members *****
Phong::Phong() : 
    Material{},
    mDiffuseBRDF{ std::make_shared<Lambertian>() },
    mAmbientBRDF{ std::make_shared<Lambertian>() },
    mGlossyBRDF{ std::make_shared<GlossySpecular>()}
{}

Phong::Phong(float kd, float ka, float ks, float exp, Colour color) : Phong{}
{
    setDiffuseReflection(kd);
    setAmbientReflection(ka);
    setGlossyReflection(ks, exp);
    setDiffuseColour(color);
} 

void Phong::setDiffuseReflection(float k)
{
    mDiffuseBRDF->setDiffuseReflection(k);
}

void Phong::setAmbientReflection(float k)
{
    mAmbientBRDF->setDiffuseReflection(k);
}

void Phong::setGlossyReflection(float k, float exp)
{
    mGlossyBRDF->setExp(exp); 
    mGlossyBRDF->setGlossyReflection(k); 
}

void Phong::setDiffuseColour(Colour colour)
{
    mDiffuseBRDF->setDiffuseColour(colour);
    mAmbientBRDF->setDiffuseColour(colour);
    mGlossyBRDF->setGlossyColour(colour); 
}

Colour Phong::shade([[maybe_unused]] ShadeRec& sr)
{
    using atlas::math::Ray;
    using atlas::math::Vector;

    Vector wo = -sr.ray.o;
    Colour L = mAmbientBRDF->rho(sr, wo) * sr.world->ambient->L(sr);
    size_t numLights = sr.world->lights.size();

    for (size_t i{ 0 }; i < numLights; ++i)
    {
        Vector wi = sr.world->lights[i]->getDirection(sr);
        float nDotWi = glm::dot(sr.normal, wi);

        if (nDotWi > 0.0f)
        {
            bool in_shadow{ false };
            if (sr.world->lights[i]->getCastsShadows()) {
                atlas::math::Ray<atlas::math::Vector> shadowRay{ (sr.ray.o + sr.t * sr.ray.d), wi };
                in_shadow = sr.world->lights[i]->in_shadow(sr, shadowRay);
            }

            if (!in_shadow) {
                
                L += (mDiffuseBRDF->fn(sr, wo, wi) + mGlossyBRDF->fn(sr, wo, wi)) * sr.world->lights[i]->L(sr) *
                    nDotWi;
            }
        }
    }
    return L;
}



// ***** Matte function members *****
Matte::Matte() :
    Material{},
    mDiffuseBRDF{ std::make_shared<Lambertian>() },
    mAmbientBRDF{ std::make_shared<Lambertian>() }
{}

Matte::Matte(float kd, float ka, Colour color) : Matte{}
{
    setDiffuseReflection(kd);
    setAmbientReflection(ka);
    setDiffuseColour(color);
}

void Matte::setDiffuseReflection(float k)
{
    mDiffuseBRDF->setDiffuseReflection(k);
}

void Matte::setAmbientReflection(float k)
{
    mAmbientBRDF->setDiffuseReflection(k);
}

void Matte::setDiffuseColour(Colour colour)
{
    mDiffuseBRDF->setDiffuseColour(colour);
    mAmbientBRDF->setDiffuseColour(colour);
}

Colour Matte::shade(ShadeRec& sr)
{
    atlas::math::Vector wo = -sr.ray.o;
    Colour L = mAmbientBRDF->rho(sr, wo) * sr.world->ambient->L(sr);

    size_t numLights = sr.world->lights.size();

    for (size_t i{ 0 }; i < numLights; ++i)
    {
        atlas::math::Vector wi = sr.world->lights[i]->getDirection(sr);
        float nDotWi = glm::dot(sr.normal, wi);

        if (nDotWi > 0.0f)
        {

            bool in_shadow{ false }; 
            if (sr.world->lights[i]->getCastsShadows()) {
                atlas::math::Ray<atlas::math::Vector> shadowRay{(sr.ray.o + sr.t * sr.ray.d), wi };
                in_shadow = sr.world->lights[i]->in_shadow(sr, shadowRay); 
            }

            if (!in_shadow) {
                L += mDiffuseBRDF->fn(sr, wo, wi) * sr.world->lights[i]->L(sr) *
                    nDotWi;
            }
           
        }
    }

    return L;
}

// ***** Matte function members *****

SV_Matte::SV_Matte() :
    Material{},
    mDiffuseBRDF{ std::make_shared<SV_Lambertian>() },
    mAmbientBRDF{ std::make_shared<SV_Lambertian>() }
{
}

SV_Matte::SV_Matte(float kd, float ka, std::shared_ptr<Texture> cd) : SV_Matte {} 
{
    setDiffuseReflection(kd);
    setAmbientReflection(ka);
    setTexture(cd);
}

void SV_Matte::setDiffuseReflection(float k)
{
    mDiffuseBRDF->setDiffuseReflection(k);
}

void SV_Matte::setAmbientReflection(float k)
{
    mAmbientBRDF->setDiffuseReflection(k);
}

void SV_Matte::setTexture(std::shared_ptr<Texture> cd)
{
    mAmbientBRDF->setTexture(cd);
    mDiffuseBRDF->setTexture(cd); 
}


Colour SV_Matte::shade(ShadeRec& sr)
{
    atlas::math::Vector wo = - sr.ray.d; 

    Colour L = mAmbientBRDF->rho(sr, wo) * sr.world->ambient->L(sr); 

    size_t numLights = sr.world->lights.size();

    for (size_t i{ 0 }; i < numLights; ++i)
    {
        atlas::math::Vector wi = sr.world->lights[i]->getDirection(sr);
        wi = glm::normalize(wi); 

        float nDotWi = glm::dot(sr.normal, wi);
        float nDotWo = glm::dot(sr.normal, wo); 


        if (nDotWi > 0.0f && nDotWo > 0.0f)
        {

            bool in_shadow{ false };
            if (sr.world->lights[i]->getCastsShadows()) {
                atlas::math::Ray<atlas::math::Vector> shadowRay{ (sr.ray.o + sr.t * sr.ray.d), wi };
                in_shadow = sr.world->lights[i]->in_shadow(sr, shadowRay);
            }

            if (!in_shadow) {
                L += mDiffuseBRDF->fn(sr, wo, wi) * sr.world->lights[i]->L(sr) *
                    nDotWi;
            }

        }
    }

    return L;
}


// ***** Light function members *****
Colour Light::L([[maybe_unused]] ShadeRec& sr)
{
    return mRadiance * mColour;
}

void Light::scaleRadiance(float b)
{
    mRadiance = b;
}

void Light::setColour(Colour const& c)
{
    mColour = c;
}

void Light::setCastsShadows(bool shadows)
{
    casts_shadows = shadows; 
}
bool Light::getCastsShadows()
{
    return casts_shadows; 
}

// ***** Divrectional function members *****
Directional::Directional() : Light{}
{}

Directional::Directional(atlas::math::Vector const& d) : Light{}
{
    setDirection(d);
}

void Directional::setDirection(atlas::math::Vector const& d)
{
    mDirection = glm::normalize(d);
}

atlas::math::Vector Directional::getDirection([[maybe_unused]] ShadeRec& sr)
{
    return mDirection;
}

bool Directional::in_shadow([[maybe_unused]] ShadeRec& sr, [[maybe_unused]] atlas::math::Ray<atlas::math::Vector> const& ray)
{
	size_t num_objects = sr.world->scene.size();
	//float d = glm::distance(ray.o, mPoint);
	float tMin = std::numeric_limits<float>::max();

	for (int j = 0; j < num_objects; j++) {
		if (sr.world->scene[j]->intersectRay(ray, tMin)) {
			return true;
		}
	}
	return false;
}


//**** PointLight function members *****

PointLight::PointLight(atlas::math::Point const& p) : Light{}
{
    setPoint(p); 
}

void PointLight::setPoint(atlas::math::Point const& p)
{
    mPoint = p; 
}

atlas::math::Vector PointLight::getDirection([[maybe_unused]] ShadeRec& sr)
{
    return glm::normalize(mPoint - sr.ray.o);
}

bool PointLight::in_shadow([[maybe_unused]]ShadeRec& sr, [[maybe_unused]] atlas::math::Ray<atlas::math::Vector> const& ray)
{
    size_t num_objects = sr.world->scene.size();
    float tMin = std::numeric_limits<float>::max();

    for (int j = 0; j < num_objects; j++) {
        if (sr.world->scene[j]->intersectRay(ray, tMin) ) {
            return true; 
        }
    }
    return false; 
}

// ***** Ambient function members *****
Ambient::Ambient() : Light{}
{}

atlas::math::Vector Ambient::getDirection([[maybe_unused]] ShadeRec& sr)
{
    return atlas::math::Vector{ 0.0f };
}

bool Ambient::in_shadow([[maybe_unused]] ShadeRec& sr, [[maybe_unused]] atlas::math::Ray<atlas::math::Vector> const& ray)
{
    return false;
}


// ***** Camera function members *****
Camera::Camera() :
    mEye{ 0.0f, 0.0f, 500.0f },
    mLookAt{ 0.0f },
    mUp{ 0.0f, 1.0f, 0.0f },
    mU{ 1.0f, 0.0f, 0.0f },
    mV{ 0.0f, 1.0f, 0.0f },
    mW{ 0.0f, 0.0f, 1.0f }
{}

void Camera::setEye(atlas::math::Point const& eye)
{
    mEye = eye;
}

void Camera::setLookAt(atlas::math::Point const& lookAt)
{
    mLookAt = lookAt;
}

void Camera::setUpVector(atlas::math::Vector const& up)
{
    mUp = up;
}

void Camera::computeUVW()
{
    mW = glm::normalize(mEye - mLookAt);
    mU = glm::normalize(glm::cross(mUp, mW));
    mV = glm::cross(mW, mU);

    if (areEqual(mEye.x, mLookAt.x) && areEqual(mEye.z, mLookAt.z) &&
        mEye.y > mLookAt.y)
    {
        mU = { 0.0f, 0.0f, 1.0f };
        mV = { 1.0f, 0.0f, 0.0f };
        mW = { 0.0f, 1.0f, 0.0f };
    }

    if (areEqual(mEye.x, mLookAt.x) && areEqual(mEye.z, mLookAt.z) &&
        mEye.y < mLookAt.y)
    {
        mU = { 1.0f, 0.0f, 0.0f };
        mV = { 0.0f, 0.0f, 1.0f };
        mW = { 0.0f, -1.0f, 0.0f };
    }
}

// ***** Pinhole function members *****
Pinhole::Pinhole() : Camera{}, mDistance{ 500.0f }, mZoom{ 1.0f }
{}

void Pinhole::setDistance(float distance)
{
    mDistance = distance;
}

void Pinhole::setZoom(float zoom)
{
    mZoom = zoom;
}

atlas::math::Vector Pinhole::rayDirection(atlas::math::Point const& p) const
{
    const auto dir = p.x * mU + p.y * mV - mDistance * mW;
    return glm::normalize(dir);
}

void Pinhole::renderScene(std::shared_ptr <World> world) const
{
    using atlas::math::Point;
    using atlas::math::Ray;
    using atlas::math::Vector;

    Point samplePoint{}, pixelPoint{};
    Ray<atlas::math::Vector> ray{};

    ray.o = mEye;
    float avg{ 1.0f / world->sampler->getNumSamples() };

    for (int r{ 0 }; r < world->height; ++r)
    {
        for (int c{ 0 }; c < world->width; ++c)
        {
            Colour pixelAverage{ 0, 0, 0 };

            for (int j = 0; j < world->sampler->getNumSamples(); ++j)
            {
                ShadeRec trace_data{};

                trace_data.world = world; 
                trace_data.t = std::numeric_limits<float>::max();
                samplePoint = world->sampler->sampleUnitSquare();
                pixelPoint.x = c - 0.5f * world->width + samplePoint.x;
                pixelPoint.y = r - 0.5f * world->height + samplePoint.y;
                ray.d = rayDirection(pixelPoint);

                bool hit{ false };

                for (auto const& obj : world->scene)
                {
                    hit |= obj->hit(ray, trace_data);
                }

                if (hit)
                    pixelAverage += trace_data.material->shade(trace_data);
            }

            //out of gammut          
            pixelAverage *= avg; 
            
            
            float M = glm::compMax(pixelAverage); 
            if (M > 1.0f) pixelAverage /= M;          
            
            world->image.push_back(pixelAverage);
            
        }
    }
}

void Pinhole::renderSceneMultiIntermediate(std::shared_ptr <World> world, size_t const numOfSlabs) const
{
    using atlas::math::Point;
    using atlas::math::Ray;
    using atlas::math::Vector;

    const size_t rowsPerSlab{ world->height / numOfSlabs };
    const size_t slabsize{ rowsPerSlab * world->width };
    const size_t numOfThreads{ std::thread::hardware_concurrency() };
    std::vector<std::thread> workers;

    // image
    world->image = std::vector<Colour>(world->width * world->height, Colour{ 1.0f, 0, 0 } );

    //create all of the slabs
    std::vector < std::vector <Slab> > threadList (numOfThreads); 

    for (size_t slabnum{ 0 }; slabnum < numOfSlabs; slabnum++) {
        size_t startindex{ slabnum * slabsize };

        size_t endindex;
        if (slabnum == (numOfSlabs - 1) && (world->height % numOfSlabs) != 0)
            endindex =
            startindex +
            slabsize +
            ((world->height % numOfSlabs) * world->width);
        else endindex = startindex + slabsize;

        Slab s = {
            startindex, 
            endindex
        };

        threadList.at(slabnum % numOfThreads).push_back(s);
    }

    //set up the threads from the threadlist
    for (std::vector <Slab> sList : threadList) {

        workers.emplace_back(std::thread([this, &world, sList]() {
            for (size_t i{ 0 }; i < sList.size(); i++ ) {

                Slab s = sList[i];
                Point samplePoint{};
                Point pixelPoint{};

                Ray<atlas::math::Vector> ray{};
                ray.o = mEye;
                float avg{ 1.0f / world->sampler->getNumSamples() };

                //slab work
                for (size_t slab_i{ s.startindex }; slab_i < s.endindex; slab_i++) {

                    Colour pixelAverage{ 0, 0, 0 };
                    size_t c{ slab_i % world->width };
                    size_t r{ slab_i / world->width };

                    for (int j = 0; j < world->sampler->getNumSamples(); ++j) {
                        ShadeRec trace_data{};

                        trace_data.world = world;
                        trace_data.t = std::numeric_limits<float>::max();
                        samplePoint = world->sampler->sampleUnitSquare();
                        pixelPoint.x = c - 0.5f * world->width + samplePoint.x;
                        pixelPoint.y = r - 0.5f * world->height + samplePoint.y;
                        ray.d = rayDirection(pixelPoint);

                        bool hit{ false };

                        for (auto const& obj : world->scene)
                        {
                            hit |= obj->hit(ray, trace_data);
                        }

                        if (hit)
                            pixelAverage += trace_data.material->shade(trace_data);
                    }

                    //out of gammut          
                    pixelAverage *= avg;

                    float M = glm::compMax(pixelAverage);
                    if (M > 1.0f) pixelAverage /= M;

                    world->image[slab_i] = pixelAverage;
                }
            }

        }));
   
    }
    for (auto& th : workers) th.join();

}

void Pinhole::renderSceneMulti(std::shared_ptr <World> world) const
{
    using atlas::math::Point;
    using atlas::math::Ray;
    using atlas::math::Vector;

    const size_t numOfSlabs{ std::thread::hardware_concurrency() };

    const size_t rowsPerSlab{ world->height / numOfSlabs };
    const size_t extra{ world->height % numOfSlabs };
    const size_t slabsize{rowsPerSlab * world->width}; 
    std::vector<std::thread> workers; 
    
    world->image = std::vector<Colour> (world->width * world->height);

    for (std::size_t slabnum { 0 }; slabnum < numOfSlabs; slabnum++) {

        workers.emplace_back(std::thread([slabnum, this, &world, &slabsize, &numOfSlabs]()
            {
                Point samplePoint{}; 
                Point pixelPoint{}; 

                Ray<atlas::math::Vector> ray{};
                ray.o = mEye;
                float avg{ 1.0f / world->sampler->getNumSamples() };
                
                size_t startindex{slabnum*slabsize}; 

                //slab work (ensure that extra rows are accounted for) 
                size_t endindex; 

                if (slabnum == (numOfSlabs-1) && (world->height % numOfSlabs) != 0) 
                    endindex = 
                        startindex + 
                        slabsize + 
                        ((world->height % numOfSlabs) * world->width);
                else endindex = startindex + slabsize;

                for (size_t i{ startindex }; i < endindex; i++) {
                    
                    Colour pixelAverage{ 0, 0, 0 };
                    size_t c{ i % world->width }; 
                    size_t r{ i / world->width };
                    
                    for (int j = 0; j < world->sampler->getNumSamples(); ++j)
                    {
                        ShadeRec trace_data{};

                        trace_data.world = world;
                        trace_data.t = std::numeric_limits<float>::max();
                        samplePoint = world->sampler->sampleUnitSquare();
                        pixelPoint.x = c - 0.5f * world->width + samplePoint.x;
                        pixelPoint.y = r - 0.5f * world->height + samplePoint.y;
                        ray.d = rayDirection(pixelPoint);

                        bool hit{ false };

                        for (auto const& obj : world->scene)
                        {
                            hit |= obj->hit(ray, trace_data);
                        }

                        if (hit)
                            pixelAverage += trace_data.material->shade(trace_data);
                    }

                    //out of gammut          
                    pixelAverage *= avg;

                    float M = glm::compMax(pixelAverage);
                    if (M > 1.0f) pixelAverage /= M;
                     
                    world->image[i] = pixelAverage;

                }
            }
        ));
    }

    for (auto& th : workers) th.join();
}

//*************** Textures *******************



//** ConstantColour function members **
ConstantColour::ConstantColour(Colour const& col) : mColour {col}
{}

void ConstantColour::setColour(Colour const& col)
{
    mColour = col; 
}

Colour ConstantColour::getColour([[maybe_unused]]const ShadeRec& sr) const
{
    return mColour;
}


ImageTexture::ImageTexture(
    std::shared_ptr<Image> image_ptr,
    std::shared_ptr<Mapping> mapping_ptr
) : image_ptr{ image_ptr }, mapping_ptr{ mapping_ptr }
{}

Colour ImageTexture::getColour([[maybe_unused]] const ShadeRec& sr) const
{
    atlas::math::Vector hitPoint{ sr.ray.o + sr.t * sr.ray.d };

    atlas::math::Vector2 textule = mapping_ptr->getTexelCoordinates(hitPoint);

    return image_ptr->getColour(textule.x, textule.y);
}

std::vector<Colour> ImageTexture::getImage() const
{
    return image_ptr->getImage(); 
}


SphericalMap::SphericalMap(atlas::math::Point origin) : origin{ origin }
{}

atlas::math::Point SphericalMap::getOrigin() const
{
    return origin;
}

atlas::math::Vector2 SphericalMap::getTexelCoordinates(
    [[maybe_unused]] atlas::math::Vector localHitPoint
) const
{
    //find the origin of the sphere
    atlas::math::Vector n = glm::normalize(localHitPoint - getOrigin());
    float u = 0.5f + glm::atan(n.x, n.z) / (2.0f * glm::pi<float>()); 
    //float v = 0.5f - glm::asin(n.z) / glm::pi<float>(); 
    float v = n.y * 0.5f + 0.5f;
    return atlas::math::Vector2{u, v};
}

Image::Image(std::string filename) : image{}, filename{ filename }, width{}, height{}
{
    width = 0, height = 0;
    int x, y, n;
    
    std::string path = ShaderPath + filename;

    unsigned char* data = stbi_load(path.c_str(), &x, &y, &n, 0);

    // process data if not NULL
    if (data != nullptr && x > 0 && y > 0 && n == 3)
    {
        width = static_cast<size_t>(x); 
        height = static_cast<size_t>(y); 

        for (std::size_t i{ 0 }; i < (width * height) * 3; i+=3)
        {
            Colour pixel {1.0};
            pixel.r = data[i]   / 255.0f; 
            pixel.g = data[i+1] / 255.0f;
            pixel.b = data[i+2] / 255.0f;

            image.push_back(pixel); 
        }      
    }
    else
    {
        std::cout << "Some error\n";
    }

    stbi_image_free(data);
}

Colour Image::getColour([[maybe_unused]] float u, [[maybe_unused]] float v)
{
    size_t col = (size_t)glm::round(u * getWidth()); 
    size_t row = (size_t)glm::round(v * getHeight()); 

    size_t pixel = row * getWidth() + col;

    return image[ pixel ];
}

size_t Image::getWidth()
{
    return width;
}

size_t Image::getHeight()
{
    return height; 
}

std::vector<Colour> Image::getImage()
{
    return image; 
}

Checker::Checker(float size, Colour colourA, Colour colourB) : size{ size }, colourA{ colourA}, colourB {colourB}
{}

Colour Checker::getColour([[maybe_unused]] const ShadeRec& sr) const
{
    // some value to prevent artifacating, choose a random eps between 0 and 0.00001
    float eps{ 0.0005f + static_cast <float> (rand()) / static_cast <float> (RAND_MAX / 0.0005f) };
    atlas::math::Vector hitPoint{ (sr.ray.o + sr.t * sr.ray.d) - eps };
    
    if ((
        (size_t)glm::floor(hitPoint.x / size) + 
        (size_t)glm::floor(hitPoint.y / size) + 
        (size_t)glm::floor(hitPoint.z / size)) % 2 == 0)
        return colourA;
    else
        return colourB;
    
}

RandomNoiseTexture::RandomNoiseTexture()
{}
 

Colour 
RandomNoiseTexture::getColour([[maybe_unused]] const ShadeRec& sr) const
{
    float noise = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    return (Colour{ noise, noise, noise });
}



// ******* Driver Code *******

int main()
{
    
    using atlas::math::Point;
    using atlas::math::Ray;
    using atlas::math::Vector;

    std::shared_ptr<World> world{ std::make_shared<World>() };

    world->width = 1000;
    world->height = 1000;
    world->background = { 0.0f, 0.0f, 0.0f };
    world->sampler = std::make_shared<Random>(4, 83);
    
    // Ground Plane Solid Color
    world->scene.push_back(
        std::make_shared<Plane>(atlas::math::Vector{ 0.0f,-1.0f, 0.0f }, atlas::math::Point{ 0.0f, 300.0f, 0.0f }));
    world->scene[0]->setMaterial(std::make_shared<Matte>(0.9f, 0.05f, Colour {1.0f, 1.0f, 1.0f}));
   
    // Sphere with regular image texturing
    atlas::math::Point worldSphereOrigin{ -300.0f, 0.0f, 0.0f };
    world->scene.push_back(
        std::make_shared<Sphere>(worldSphereOrigin, 100.0f));
    world->scene[1]->setMaterial(std::make_shared<SV_Matte>(
        0.9f, 0.05f,
        std::make_shared<ImageTexture>(
            ImageTexture{
                std::make_shared<Image>("EarthHighRes.jpg"),
                std::make_shared<SphericalMap>(worldSphereOrigin)
            })
        )
    );
 
    // Sphere with random noise texturings
    world->scene.push_back(
        std::make_shared<Sphere>(atlas::math::Point{ 300.0f, 0.0f, 100.0f }, 100.0f)
    );
    world->scene[2]->setMaterial(std::make_shared<SV_Matte>(
        0.9f, 0.05f,
        std::make_shared<RandomNoiseTexture>(RandomNoiseTexture{}))
    );
 
   
    // Plane with Procedural Texturing
    world->scene.push_back(
        std::make_shared<Plane>(atlas::math::Vector{ 0.0f, 0.0f, 1.0f }, atlas::math::Point{ 0.0f, 300.0f, -1000.0f }));
    world->scene[3]->setMaterial(std::make_shared<SV_Matte>(
        0.9f, 0.05f,
        std::make_shared<Checker>(Checker{ 100.0f, Colour {1.0f, 0.0f, 1.0f}, Colour {0.1f, 0.1f, 0.1f} })
        )
    );

    
    // Render a Mesh
    Mesh m = Mesh("cube.ply", 100);
    for (Triangle t : m.triangleList) {
        t.setMaterial(std::make_shared<Matte>(
            0.9f, 0.05f, Colour{ 1.0f, 0.6f, 0.1f })
        );
        world->scene.push_back(std::make_shared<Triangle>(t));
    }

    world->ambient = std::make_shared<Ambient>();
    world->ambient->setColour({ 1.0f, 1.0f, 1.0f });
    world->ambient->scaleRadiance(1.0f);
    world->ambient->setCastsShadows(false);
    
    world->lights.push_back(
        std::make_shared<PointLight>(PointLight{ {-500.0f, -300.0f, 800.0f} }));
    world->lights[0]->setColour({ 1.0f, 1.0f, 1.0f });
    world->lights[0]->scaleRadiance(2.0f);
    world->lights[0]->setCastsShadows(true); 
    
    // set up camera
    Pinhole camera{};

    // change camera position here
    camera.setEye({ -300.0f, 0.0f, 500.0f });
    camera.setLookAt({ 0.0f, 0.0f, 0.0f });

    camera.computeUVW();

    atlas::core::Timer <float> timer; 

   
    timer.start();
    camera.renderScene(world);
    fmt::print("single thread time elapsed : {} \n", timer.elapsed()); 
    world->image.clear();
    
    //comment out for simple
    /*
    timer.start();
    camera.renderSceneMulti(world);
    fmt::print("simple multi thread time elapsed : {} \n", timer.elapsed());
    */
    //world->image.clear();
    
    
    timer.start();
    camera.renderSceneMultiIntermediate(world, 25); 
    fmt::print("multi thread intermediate time elapsed : {} \n", timer.elapsed());
    
    saveToFile("raytrace.bmp", world->width, world->height, world->image);
    

    return 0;
}

void saveToFile(std::string const& filename,
    std::size_t width,
    std::size_t height,
    std::vector<Colour> const& image)
{
    std::vector<unsigned char> data(image.size() * 3);

    for (std::size_t i{ 0 }, k{ 0 }; i < image.size(); ++i, k += 3)
    {
        Colour pixel = image[i];
        data[k + 0] = static_cast<unsigned char>(pixel.r * 255);
        data[k + 1] = static_cast<unsigned char>(pixel.g * 255);
        data[k + 2] = static_cast<unsigned char>(pixel.b * 255);
    }

    stbi_write_bmp(filename.c_str(),
        static_cast<int>(width),
        static_cast<int>(height),
        3,
        data.data());
}