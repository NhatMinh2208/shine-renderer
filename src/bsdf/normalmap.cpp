#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief BSDF that has Normal Map
 */
class NormalMap : public BSDF {
public:
    NormalMap(const PropertyList &propList) {
    }

    /// Evaluate the BRDF model
    Color3f eval(const BSDFQueryRecord &bRec, const Intersection &si) const {
        auto prev_cosTheta = si.shFrame.n.dot(bRec.wo);
        if (prev_cosTheta == 0.f) return 0.f; 
        Intersection pertubed_si(si);
        pertubed_si.shFrame = perturbate(si);
        //BSDFQueryRecord bRec_(pertubed_si.toLocal(si.toWorld(bRec.wi)).normalized());
        BSDFQueryRecord bRec_(pertubed_si.toLocal(bRec.wi).normalized(), 
                       pertubed_si.toLocal(bRec.wo).normalized(), ESolidAngle);
        if (Frame::cosTheta(bRec_.wo) * Frame::cosTheta(bRec.wo) <= 0) return 0.f;
        auto result = m_nested_bsdf->eval(bRec_, pertubed_si);
        // if (result[0] < 0 || result[1] < 0 || result[2] < 0) return 0.f;
        // Assert( pertubed_si.shFrame.n.dot(bRec_.wo) >= 0);
        Assert(result[0]  >= 0.f || result[1]  >= 0.f|| result[2]  >= 0.f);
        return result * pertubed_si.shFrame.n.dot(bRec_.wo) / prev_cosTheta;
    }

    /// Compute the density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec, const Intersection &si) const {
        Intersection pertubed_si(si);
        pertubed_si.shFrame = perturbate(si);
        BSDFQueryRecord bRec_(pertubed_si.toLocal(bRec.wi).normalized(), 
                       pertubed_si.toLocal(bRec.wo).normalized(), ESolidAngle);
        if (Frame::cosTheta(bRec_.wo) * Frame::cosTheta(bRec.wo) <= 0) return 0.f;
        auto pdf_ = m_nested_bsdf->pdf(bRec_, pertubed_si);
        Assert(pdf_ >= 0);
        return pdf_;
    }

    /// Draw a a sample from the BRDF model
    Color3f sample(BSDFQueryRecord &bRec, const Intersection &si, const Point2f &sample) const {
        Intersection pertubed_si(si);
        pertubed_si.shFrame = perturbate(si);
        // Normal map return normal in local space
        BSDFQueryRecord bRec_(pertubed_si.toLocal(bRec.wi).normalized());
        auto result = m_nested_bsdf->sample(bRec_, pertubed_si, sample);
        bRec.wo = pertubed_si.toWorld(bRec_.wo);
        if(Frame::cosTheta(bRec_.wo) * Frame::cosTheta(bRec.wo) <= 0) result = Color3f(0.f);
        bRec.eta = 1.0f;
        bRec.measure = ESolidAngle;
        Assert(result[0] >= 0.f);
        return result;
    }

    bool isDiffuse() const {
        return m_nested_bsdf->isDiffuse();
    }

    /// Return a human-readable summary
    std::string toString() const {
        return tfm::format(
            "Normal Map[\n"
            "  normal map = %s,\n"
            "  nested bsdf = %s,\n"
            "]", 
            m_normalmap ? indent(m_normalmap->toString()) : std::string("null"),
            m_nested_bsdf ? indent(m_nested_bsdf->toString()) : std::string("null")
            );
    }

    EClassType getClassType() const { return EBSDF; }

    void addChild(NoriObject *obj) {
        switch (obj->getClassType()) {
            case ETexture: {
                    if (m_normalmap)
                        throw NoriException("There can only be one normal map!");
                    m_normalmap = static_cast<Texture<Color3f> *>(obj);
                }
            
                break;
            case EBSDF: {
                    if (m_nested_bsdf)
                        throw NoriException("There can be only one nested BSDF allowed!");
                    m_nested_bsdf = static_cast<BSDF *>(obj);
                }
            
                break;
            default:
                throw NoriException("Scene::addChild(<%s>) is not supported!",
                    classTypeName(obj->getClassType()));
        }
    }

    Frame perturbate(const Intersection &si) const {
        Color3f c = m_normalmap->eval(si) * 2 - 1.f;
        Normal3f n(c[0], c[1], c[2]); // 
        return Frame(n.normalized());
    }
private:
    Texture<Color3f>* m_normalmap = nullptr;
    BSDF* m_nested_bsdf = nullptr;
};

NORI_REGISTER_CLASS(NormalMap, "normalmap");
NORI_NAMESPACE_END
