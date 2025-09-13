//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType
{
    DIFFUSE,
    MICROFACET
};

class Material
{

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0)
        {
            cosi = -cosi;
        }
        else
        {
            std::swap(etai, etat);
            n = -N;
        }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0)
        {
            std::swap(etai, etat);
        }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1)
        {
            kr = 1;
        }
        else
        {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N)
    {
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y))
        {
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x * invLen);
        }
        else
        {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y * invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

    float D(const Vector3f &h, const Vector3f &n, float roughness);
    float G(const Vector3f &n, const Vector3f &v, const Vector3f &l, float roughness);

public:
    MaterialType m_type;
    // Vector3f m_color;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float specularExponent;
    // Texture tex;

    inline Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0, 0, 0));
    inline MaterialType getType();
    // inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    // 按照该材质的性质，给定入射方向与法向量，用某种分布采样一个出射方向
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    // 给定一对入射、出射方向与法向量，计算 sample 方法得到该出射方向的概率密度
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    // 给定一对入射、出射方向与法向量，计算这种情况下的 f_r 值
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
};

Material::Material(MaterialType t, Vector3f e)
{
    m_type = t;
    // m_color = c;
    m_emission = e;
}

MaterialType Material::getType() { return m_type; }
/// Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() { return m_emission; }
bool Material::hasEmission()
{
    if (m_emission.norm() > EPSILON)
        return true;
    else
        return false;
}

Vector3f Material::getColorAt(double u, double v)
{
    return Vector3f();
}

Vector3f Material::sample(const Vector3f &wi, const Vector3f &N)
{
    switch (m_type)
    {
    case DIFFUSE:
    case MICROFACET:   
    {
        // uniform sample on the hemisphere
        float x_1 = get_random_float(), x_2 = get_random_float();
        float z = std::fabs(1.0f - 2.0f * x_1);
        float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
        Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
        return toWorld(localRay, N);

        break;
    }
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N)
{
    switch (m_type)
    {
    case DIFFUSE:
    case MICROFACET:
    {
        // uniform sample probability 1 / (2 * PI)
        if (dotProduct(wo, N) > 0.0f)
            return 0.5f / M_PI;
        else
            return 0.0f;
        break;
    }
    }
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N)
{
    switch (m_type)
    {
    case DIFFUSE:
    {
        // calculate the contribution of diffuse   model
        float cosalpha = dotProduct(N, wo);
        if (cosalpha > 0.0f)
        {
            Vector3f diffuse = Kd / M_PI;
            return diffuse;
        }
        else
            return Vector3f(0.0f);
        break;
    }
    case MICROFACET:
    {
        float cosalpha = dotProduct(N, wo);
        if (cosalpha > 0.0f)
        {
            float roughness = 0.4f;
            Vector3f V = -wi;
            Vector3f L = wo;
            Vector3f H = normalize(V + L);

            float etat = 1.85f;
            float F_term;
            fresnel(wi, N, etat, F_term);

            float G_term = G(N, V, L, roughness);

            float D_term = D(H,N,roughness);

            float nom = F_term * G_term * D_term;
            float denom = 4.f * std::max(dotProduct(N, V), 0.f) * std::max(dotProduct(N, L), 0.f);
            Vector3f specular(nom/std::max(denom,0.001f));

            float ks_ = F_term;
            float kd_ = 1.f - ks_;

            Vector3f diffuse = Kd / M_PI;

            return ks_ * specular + kd_ * diffuse;
        }
        else
            return Vector3f(0.0f);
        break;
    }
    }
}

inline float Material::D(const Vector3f &h, const Vector3f &n,float roughness)
{
    float a = roughness * roughness;
    float a2 = a * a;
    float nom = a2;
    float NdotH = std::max(dotProduct(n, h),0.f);
    float NdotH2 = NdotH * NdotH;
    float denom = NdotH2 * NdotH2 * (a2 - 1) + 1;
    denom = denom * denom * M_PI;

    return nom / std::max(denom, 0.0000001f);
}

inline float Material::G(const Vector3f &n, const Vector3f &v, const Vector3f &l, float roughness)
{
    float a = (roughness + 1) / 2;
    a *= a;
    float k = a/2;
    auto G_dir = [](const Vector3f &N, const Vector3f &V, float k)
    {
        float NdotV = std::max(dotProduct(N, V),0.f);
        float nom = NdotV;
        float denom = NdotV * (1.f - k) + k;
        return nom / denom;
    };
    return G_dir(n, v, k) * G_dir(n, l, k);
}

#endif // RAYTRACING_MATERIAL_H
