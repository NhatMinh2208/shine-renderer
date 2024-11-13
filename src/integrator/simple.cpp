#include <nori/integrator.h>
#include <nori/scene.h>
NORI_NAMESPACE_BEGIN
class SimpleIntegrator : public Integrator {
    public:
    SimpleIntegrator(const PropertyList &props){
        // m_myProperty = props.getString("myProperty");
        // std:: cout << "Parameter value was : " << m_myProperty << std::endl;
        p = props.getPoint("position");
        phi = props.getColor("energy");
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);
        Vector3f dir = p - its.p;
        Vector3f dir_norm = dir.normalized();
        float cosTheta = std::max(0.f, dir_norm.dot(its.shFrame.n));
        Color3f c = (phi * INV_PI * INV_PI / 4) * (cosTheta /dir.squaredNorm()); //the dir here must not been normalized
        return !scene->rayIntersect(Ray3f(its.p, dir_norm)) ? c : Color3f(0.0f);
    }

    std::string toString() const {
        return "SimpleIntegrator[]";
        
    }
    protected:
    std::string m_myProperty;
    Point3f p;   // light position 
    Color3f phi; // energy
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END