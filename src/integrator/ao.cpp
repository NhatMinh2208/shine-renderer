#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>
NORI_NAMESPACE_BEGIN
class AOIntegrator : public Integrator {
    public:
    AOIntegrator(const PropertyList &props){
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        auto shFrame = its.shFrame;
        Vector3f dir = shFrame.toWorld(Warp::squareToCosineHemisphere(sampler->next2D()));
        Vector3f dir_norm = dir.normalized();
        float cosTheta = std::max(0.f, dir_norm.dot(shFrame.n));
        Color3f c = cosTheta * INV_PI;
        return !scene->rayIntersect(Ray3f(its.p, dir_norm)) ? c : Color3f(0.0f);
    }

    std::string toString() const {
        return "AOIntegrator[]";
        
    }
};

NORI_REGISTER_CLASS(AOIntegrator, "ao");
NORI_NAMESPACE_END