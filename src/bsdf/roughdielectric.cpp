#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/common.h>
#include <nori/warp.h>
#include <algorithm>
NORI_NAMESPACE_BEGIN

/// Rough dielectric BSDF
class RoughDielectric : public BSDF {
public:
    RoughDielectric(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* RMS surface roughness */
        m_alpha = propList.getFloat("alpha", 0.1f);
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


    /// Masking function depend on omega
    float D(Vector3f w, Vector3f wh) const {
        return G1(w, wh) / abs(Frame::cosTheta(w)) * Warp::squareToBeckmannPdf(wh, m_alpha) * abs(w.dot(wh)); 
    }
    Color3f eval(const BSDFQueryRecord &bRec) const {
        /*  */
        // Calculate the half-vector
        Vector3f wh;
        float cosTheta_h;
        auto cosTheta_i = Frame::cosTheta(bRec.wi);
        auto cosTheta_o = Frame::cosTheta(bRec.wo);

        auto eta = m_intIOR / m_extIOR;
        // reflect or refract
        bool reflect = cosTheta_i * cosTheta_o > 0;
        if(!reflect){
            eta = cosTheta_o > 0.f ? eta : (1.f / eta);
            wh = (bRec.wo * eta + bRec.wi).normalized();
            cosTheta_h = Frame::cosTheta(wh);
        }
        else{
            wh = (bRec.wi + bRec.wo) / (bRec.wi + bRec.wo).norm();
            cosTheta_h = Frame::cosTheta(wh);
        }
        if (wh.dot(bRec.wi) * cosTheta_i < 0 || wh.dot(bRec.wo) * cosTheta_o < 0) return Color3f(0.f);
        auto F = fresnel(wh.dot(bRec.wi), m_extIOR, m_intIOR);
        auto D = Warp::squareToBeckmannPdf(wh, m_alpha);
        if(reflect){
            return D * F * G1(bRec.wi, wh) * G1(bRec.wo, wh)  / (4 * cosTheta_i * cosTheta_o); 
        }
        else {
            float denom = (bRec.wo.dot(wh) + bRec.wi.dot(wh) / eta) *  (bRec.wo.dot(wh) + bRec.wi.dot(wh) / eta) *
                    cosTheta_i * cosTheta_o;
            return D * (1 - F) * G1(bRec.wi, wh) * G1(bRec.wo, wh) * abs(bRec.wo.dot(wh) * bRec.wi.dot(wh) / denom);
        }
    }

    float pdf(const BSDFQueryRecord &bRec) const {
        /* */
         // Calculate the half-vector
        Vector3f wh;
        float cosTheta_h;
        auto cosTheta_i = Frame::cosTheta(bRec.wi);
        auto cosTheta_o = Frame::cosTheta(bRec.wo);

        auto eta = m_intIOR / m_extIOR;
        // reflect or refract
        bool reflect = cosTheta_i * cosTheta_o > 0;
        if(!reflect){
            eta = cosTheta_o > 0.f ? eta : (1.f / eta);
            wh = (bRec.wo * eta + bRec.wi).normalized();
            cosTheta_h = Frame::cosTheta(wh);
        }
        else{
            wh = (bRec.wi + bRec.wo) / (bRec.wi + bRec.wo).norm();
            cosTheta_h = Frame::cosTheta(wh);
        }
        if (wh.dot(bRec.wi) * cosTheta_i < 0 || wh.dot(bRec.wo) * cosTheta_o < 0) return 0.f;
        auto R = fresnel(wh.dot(bRec.wi), m_extIOR, m_intIOR);
        auto T = 1.f - R;
        float pdf;
        if(reflect){
            return D(bRec.wi,wh) / (4 * abs(bRec.wi.dot(wh))) * R; 
        }
        else{
            float denom = (bRec.wo.dot(wh) + bRec.wi.dot(wh) / eta);
            float dwh_dwo = abs(bRec.wo.dot(wh)) / (denom * denom);
            return D(bRec.wi,wh) * dwh_dwo * T; 
        }
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
    float m_intIOR, m_extIOR, m_alpha;
};

NORI_REGISTER_CLASS(RoughDielectric, "roughdielectric");
NORI_NAMESPACE_END
