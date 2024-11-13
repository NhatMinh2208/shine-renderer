/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/common.h>
#include <algorithm>
NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    Color3f eval(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    float pdf(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
        //throw NoriException("Unimplemented!");
        bRec.measure = EDiscrete;
        auto R = fresnel(Frame::cosTheta(bRec.wi), m_extIOR, m_intIOR);
        if(sample[0] < R){
            bRec.wo = Vector3f(
            -bRec.wi.x(),
            -bRec.wi.y(),
             bRec.wi.z());

            return Color3f(1.0f);
        }
        else{
            auto n  = Vector3f(0.f, 0.f, 1.f);
            auto eta = m_intIOR / m_extIOR;
            auto cosTheta_i = Frame::cosTheta(bRec.wi);
            if (cosTheta_i < 0.f){
                eta = 1.f / eta;
                n.z() = -1.f;
                cosTheta_i = -cosTheta_i;
            }

            auto sin2Theta_i = std::max(0.f, 1.f - cosTheta_i * cosTheta_i);
            auto sin2Theta_t = sin2Theta_i / (eta * eta);
            auto cosTheta_t = sqrt(1 - sin2Theta_t); 
            bRec.wo = - (bRec.wi / eta) + (cosTheta_i / eta - cosTheta_t) * n; 
            bRec.eta = eta;
            return Color3f(1.0f);
        }

    }

    std::string toString() const {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }
private:
    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
