/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Diffuse / Lambertian BRDF model
 */
class Diffuse : public BSDF {
public:
    Diffuse(const PropertyList &propList) {
        // float albedo;
        // if (!m_albedo){
        //     if (propList.hasFloat("albedo", albedo)){
        //         m_albedo = static_cast<Texture<Color3f> *>(NoriObjectFactory::createInstance("constant", PropertyList()));
        //         // i so sorry for who have to read this shitty
        //         // i will remove this soon, it just temporary before i got smarter
        //     }
        //     else m_albedo = static_cast<Texture<Color3f> *>(NoriObjectFactory::createInstance("constant", PropertyList()));
        // }
        // else{
        //     if (propList.hasFloat("albedo", albedo)) {
        //         throw NoriException("There can only be one albedo texture in diffuse material!");
        //     }
        // }
    }

    /// Evaluate the BRDF model
    Color3f eval(const BSDFQueryRecord &bRec, const Intersection &si) const {
        /* This is a smooth BRDF -- return zero if the measure
           is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);

        /* The BRDF is simply the albedo / pi */
        return m_albedo->eval(si) * INV_PI;
    }

    /// Compute the density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec, const Intersection &si) const {
        /* This is a smooth BRDF -- return zero if the measure
           is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return 0.0f;


        /* Importance sampling density wrt. solid angles:
           cos(theta) / pi.

           Note that the directions in 'bRec' are in local coordinates,
           so Frame::cosTheta() actually just returns the 'z' component.
        */
        return INV_PI * Frame::cosTheta(bRec.wo);
    }

    /// Draw a a sample from the BRDF model
    Color3f sample(BSDFQueryRecord &bRec, const Intersection &si, const Point2f &sample) const {
        if (Frame::cosTheta(bRec.wi) <= 0)
            return Color3f(0.0f);

        bRec.measure = ESolidAngle;

        /* Warp a uniformly distributed sample on [0,1]^2
           to a direction on a cosine-weighted hemisphere */
        bRec.wo = Warp::squareToCosineHemisphere(sample);

        /* Relative index of refraction: no change */
        bRec.eta = 1.0f;

        /* eval() / pdf() * cos(theta) = albedo. There
           is no need to call these functions. */
        return m_albedo->eval(si);
    }

    bool isDiffuse() const {
        return true;
    }

    /// Return a human-readable summary
    std::string toString() const {
        return tfm::format(
            "Diffuse[\n"
            "  albedo = %s\n"
            "]", 
            m_albedo ? indent(m_albedo->toString()) : std::string("null")
            );
    }

    EClassType getClassType() const { return EBSDF; }

    void addChild(NoriObject *obj) {
        switch (obj->getClassType()) {
            case ETexture: {
                    if (m_albedo)
                        throw NoriException("There can only be one albedo texture in diffuse material!");
                    m_albedo = static_cast<Texture<Color3f> *>(obj);
                }
                break;
            default:
                throw NoriException("Scene::addChild(<%s>) is not supported!",
                    classTypeName(obj->getClassType()));
        }
    }
private:
    //Color3f m_albedo;
    Texture<Color3f>* m_albedo = nullptr;
};

NORI_REGISTER_CLASS(Diffuse, "diffuse");
NORI_NAMESPACE_END
