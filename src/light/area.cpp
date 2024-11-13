#include <nori/emitter.h>
#include <nori/mesh.h>
NORI_NAMESPACE_BEGIN
class AreaLight : public Emitter {
    public:
    AreaLight(const PropertyList &props){
        radiance = props.getColor("radiance");
    }

    Color3f sample(EmitterQueryRecord &eRec, const Point3f p, Sampler* sampler) const {
        eRec = mesh->sample(sampler);
        Vector3f wi = (eRec.p - p).normalized();
        return eval(eRec, wi);
    }
    Color3f eval(const EmitterQueryRecord &eRec, const Vector3f wi) const {
        return wi.dot(eRec.n) < 0.f ? radiance : Color3f(0.f);
    }
    float pdf(const EmitterQueryRecord &eRec) const {
        return eRec.pdf;
    }

    std::string toString() const {
        return "AreaLight[]";
        
    }
    protected:
    Color3f radiance;
};

NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END