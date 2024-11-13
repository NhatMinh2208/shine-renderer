/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Microfacet : public BSDF {
public:
    Microfacet(const PropertyList &propList) {
        /* RMS surface roughness */
        m_alpha = propList.getFloat("alpha", 0.1f);

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = propList.getColor("kd", Color3f(0.5f));

        /* To ensure energy conservation, we must scale the 
           specular component by 1-kd. 

           While that is not a particularly realistic model of what 
           happens in reality, this will greatly simplify the 
           implementation. Please see the course staff if you're 
           interested in implementing a more realistic version 
           of this BRDF. */
        m_ks = 1 - m_kd.maxCoeff();
    }

    /// Evaluate the shadowing term
    float G1(const Vector3f wv, const Vector3f wh) const { //G1 must be a const function since it been called by a const function
        auto tanTheta_v = Frame::tanTheta(wv);
        auto b = 1.f / (m_alpha * tanTheta_v);
        if (wv.dot(wh) / Frame::cosTheta(wv) > 0){
            if (b < 1.6) {
                return (3.535 * b + 2.181 * b * b) / (1 + 2.276 * b + 2.577 * b * b);
            }
            else return 1;
        }
        else return 0;
    }

    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord &bRec) const {
    	//throw NoriException("MicrofacetBRDF::eval(): not implemented!");
        Vector3f wh = (bRec.wi + bRec.wo) / (bRec.wi + bRec.wo).norm();
        auto cosTheta_i = Frame::cosTheta(bRec.wi);
        auto cosTheta_o = Frame::cosTheta(bRec.wo);
        auto cosTheta_h = Frame::cosTheta(wh);
        auto F = fresnel(wh.dot(bRec.wi), m_extIOR, m_intIOR);
        auto D = Warp::squareToBeckmannPdf(wh, m_alpha);
        return m_kd * INV_PI + ( m_ks * D * F * G1(bRec.wi, wh) * G1(bRec.wo, wh) ) / (4 * cosTheta_i * cosTheta_o * cosTheta_h); 
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
    	// throw NoriException("MicrofacetBRDF::pdf(): not implemented!");
        
		if (Frame::cosTheta(bRec.wi) <= 0 || Frame::cosTheta(bRec.wo) <= 0)
			return 0.0f;
        Vector3f wh = (bRec.wi + bRec.wo) / (bRec.wi + bRec.wo).norm();
        auto cosTheta_o = Frame::cosTheta(bRec.wo);
        auto J_h = 1.f / (4 * wh.dot(bRec.wo));
        return m_ks * Warp::squareToBeckmannPdf(wh, m_alpha) * J_h + (1 - m_ks) * cosTheta_o * INV_PI; 
    }

    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
    	// throw NoriException("MicrofacetBRDF::sample(): not implemented!");

        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
        // return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
        if (Frame::cosTheta(bRec.wi) <= 0) return Color3f(0.f);
        auto u = _sample[0], v = _sample[1];
        if(u < m_ks) {
            u /= m_ks;
            Vector3f normal = Warp::squareToBeckmann(Point2f(u, v), m_alpha);
            bRec.wo = -bRec.wi + 2 * (normal.dot(bRec.wi)) * normal;
        }
        else {
            u = (u - m_ks) / (1.f - m_ks);
            bRec.wo = Warp::squareToCosineHemisphere(Point2f(u, v));
        }
        bRec.measure = ESolidAngle;
        bRec.eta = m_intIOR / m_extIOR;
        return pdf(bRec) > 0.f ? eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec) : Color3f(0.f);
    }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    std::string toString() const {
        return tfm::format(
            "Microfacet[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "  ks = %f\n"
            "]",
            m_alpha,
            m_intIOR,
            m_extIOR,
            m_kd.toString(),
            m_ks
        );
    }
private:
    float m_alpha;
    float m_intIOR, m_extIOR;
    float m_ks;
    Color3f m_kd;
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
